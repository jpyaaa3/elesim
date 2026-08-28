"""Pull checkpoints from a training server and evaluate them here.

Training runs on the GPU box; evaluation runs wherever this is invoked.  The
Mac initiates every connection, so the server needs no inbound access and no
knowledge of this machine.

Password authentication works without storing the password: an SSH
`ControlMaster` socket is opened once, interactively, and every later `rsync`
reuses it.  Key authentication is better and makes the first step silent --
`ssh-copy-id <host>` once, and nothing here changes.

Two things this refuses to do, both because they produce numbers that look
fine and are not:

* Evaluate a checkpoint whose training commit differs from the code here.  The
  observation vector keeps its width while its channels change meaning, so a
  mismatch scores the wrong thing silently.  It happened: an eval was reading
  an object "yaw" the training had stopped producing.
* Vary the evaluation settings between checkpoints of one run.  The point of
  the curve is comparing iterations to each other, which only works if
  everything else is pinned.

Run::

    python -m elesim_sim.rl.watch_eval \\
        --host 147.46.175.23 \\
        --remote-run ~/elesim/sim/rl_runs/wrap_grasp/stage2_srv_v5 \\
        --interval 100
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

_REPO_ROOT = Path(__file__).resolve().parents[4]
_CSV_COLUMNS = (
    "iteration",
    "episodes",
    "success_rate",
    "collision",
    "topple",
    "retention",
    "no_wrap",
    "no_reach",
    "phi_mean_deg",
    "phi_max_deg",
    "checkpoint",
)


# --------------------------------------------------------------------------
# ssh / rsync
# --------------------------------------------------------------------------


def control_path(host: str) -> Path:
    return Path.home() / ".ssh" / f"elesim-watch-{re.sub(r'[^A-Za-z0-9]', '_', host)}"


def ssh_options(host: str) -> list[str]:
    """Options that make every call reuse one authenticated connection."""
    return [
        "-o", "ControlMaster=auto",
        "-o", f"ControlPath={control_path(host)}",
        "-o", "ControlPersist=8h",
        "-o", "ServerAliveInterval=30",
    ]


def open_master(host: str) -> None:
    """Authenticate once, then leave a socket behind for rsync to reuse.

    With password authentication this is the only prompt; it is interactive on
    purpose, so no password is ever written down.
    """
    sock = control_path(host)
    check = subprocess.run(
        ["ssh", "-O", "check", *ssh_options(host), host],
        capture_output=True, text=True,
    )
    if check.returncode == 0:
        print(f"[watch] 기존 SSH 연결 재사용: {sock}")
        return
    print(f"[watch] {host} 에 연결합니다. 비밀번호 인증이면 여기서 한 번 물어봅니다.")
    made = subprocess.run(["ssh", "-MNf", *ssh_options(host), host])
    if made.returncode != 0:
        raise SystemExit(f"[watch] SSH 연결 실패: {host}")
    print("[watch] 연결됨. 이후 rsync는 이 소켓을 재사용합니다.")


def check_remote_path(remote_run: str) -> None:
    """Catch a `~` the local shell expanded before the script ever saw it.

    `--remote-run ~/elesim/...` is the natural thing to type and the shell turns
    it into *this* machine's home, which is then sent to a server that has no
    such directory.  rsync's "change_dir failed" is easy to read past, and the
    watcher then just reports no metadata and waits, so the mistake costs a poll
    interval per attempt to notice.
    """
    home = str(Path.home())
    if remote_run.startswith(home):
        tail = remote_run[len(home):].lstrip("/")
        raise SystemExit(
            f"[watch] --remote-run 이 이 컴퓨터의 홈으로 확장되었습니다:\n"
            f"         {remote_run}\n"
            f"         셸이 ~ 를 먼저 펼쳤습니다. 서버 쪽 경로를 주세요:\n"
            f"           --remote-run '~/{tail}'      (따옴표로 감싸 원격에서 펼치게)\n"
            f"         또는 절대경로로:\n"
            f"           --remote-run /home/<user>/{tail}"
        )


def pull(host: str, remote_run: str, local_run: Path) -> bool:
    """Mirror the checkpoints and the run metadata, nothing else.

    Tensorboard event files are excluded: they are large, they grow every
    iteration, and nothing here reads them.

    Returns whether the remote directory was there at all, so a wrong path is
    reported as a wrong path rather than as a run that has not started.
    """
    local_run.mkdir(parents=True, exist_ok=True)
    remote = f"{host}:{remote_run.rstrip('/')}/"
    ssh = "ssh " + " ".join(ssh_options(host))
    cmd = [
        "rsync", "-az", "--partial",
        "-e", ssh,
        "--include=metadata.json",
        "--include=model_*.pt",
        "--include=model_*.curriculum.json",
        "--exclude=*",
        remote, str(local_run) + "/",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        return True
    err = result.stderr.strip()
    if "change_dir" in err or "No such file or directory" in err:
        print(
            f"[watch] 서버에 그 디렉터리가 없습니다:\n"
            f"         {remote_run}\n"
            f"         학습이 아직 시작되지 않았거나 경로가 틀렸습니다. 서버에서 확인:\n"
            f"           ls -d {remote_run}"
        )
        return False
    print(f"[watch] rsync 실패 (재시도합니다): {err[:200]}")
    return True


# --------------------------------------------------------------------------
# guards
# --------------------------------------------------------------------------


def local_commit() -> Optional[str]:
    out = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=_REPO_ROOT, capture_output=True, text=True
    )
    return out.stdout.strip() if out.returncode == 0 else None


def check_commit(local_run: Path, *, allow_mismatch: bool) -> None:
    """Refuse to evaluate a run trained at a different revision.

    A warning is not enough.  The observation width does not change when its
    meaning does, so a mismatch produces a plausible-looking success rate for
    a policy that is being fed inputs it never saw.
    """
    meta_path = local_run / "metadata.json"
    if not meta_path.is_file():
        print("[watch] metadata.json 이 아직 없습니다. 다음 주기에 다시 봅니다.")
        raise _NotReady
    meta = json.loads(meta_path.read_text())
    trained = (meta.get("git") or {}).get("commit")
    here = local_commit()
    if trained is None:
        message = (
            "학습 런에 커밋 기록이 없습니다 (train.py 가 git 정보를 쓰기 전 버전). "
            "관측 의미가 같은지 직접 확인해야 합니다."
        )
    elif trained == here:
        return
    else:
        message = (
            f"학습 커밋 {trained[:10]} != 이곳 {str(here)[:10]}. "
            "관측 채널의 의미가 다르면 평가가 조용히 틀린 값을 냅니다."
        )
    if allow_mismatch:
        print(f"[watch] 경고: {message}")
        return
    raise SystemExit(
        f"[watch] 중단: {message}\n"
        f"         같은 커밋으로 맞추거나, 알고서도 진행하려면 --allow-commit-mismatch"
    )


class _NotReady(Exception):
    """The run directory has not appeared yet."""


# --------------------------------------------------------------------------
# evaluation
# --------------------------------------------------------------------------


def checkpoints(local_run: Path, interval: int) -> list[tuple[int, Path]]:
    found = []
    for path in local_run.glob("model_*.pt"):
        m = re.fullmatch(r"model_(\d+)\.pt", path.name)
        if not m:
            continue
        it = int(m.group(1))
        if interval > 0 and it % interval != 0:
            continue
        found.append((it, path))
    return sorted(found)


def evaluate(
    ckpt: Path,
    out_dir: Path,
    *,
    n_envs: int,
    episodes: int,
    render: int,
    extra: Sequence[str],
) -> Optional[dict]:
    out_dir.mkdir(parents=True, exist_ok=True)
    report = out_dir / f"{ckpt.stem}.md"
    cmd = [
        sys.executable, "-m", "elesim_sim.rl.eval",
        "--checkpoint", str(ckpt),
        "--set", f"runtime.n_envs={n_envs}",
        "--set", f"eval.episodes_per_condition={episodes}",
        "--out", str(report),
    ]
    if render > 0:
        cmd += ["--render", str(render), "--render-episodes", "2"]
    cmd += list(extra)
    print(f"[watch] 평가 {ckpt.name} ...", flush=True)
    result = subprocess.run(cmd, cwd=_REPO_ROOT, capture_output=True, text=True)
    if result.returncode != 0:
        tail = (result.stderr or result.stdout).strip().splitlines()[-3:]
        print(f"[watch] 평가 실패 {ckpt.name}: {' / '.join(tail)}")
        return None
    payload = report.with_suffix(".json")
    return json.loads(payload.read_text()) if payload.is_file() else None


def summarise(iteration: int, ckpt: Path, data: dict) -> dict:
    import math

    conds = data["conditions"]
    total = sum(c["episodes"] for c in conds)
    ok = sum(c["successes"] for c in conds)
    row = {
        "iteration": iteration,
        "episodes": total,
        "success_rate": round(ok / max(total, 1), 4),
        "phi_mean_deg": round(
            math.degrees(
                sum(c["phi_mean_rad"] * c["episodes"] for c in conds) / max(total, 1)
            ), 1
        ),
        "phi_max_deg": round(math.degrees(max(c["phi_max_rad"] for c in conds)), 1),
        "checkpoint": ckpt.name,
    }
    for key in ("collision", "topple", "retention", "no_wrap", "no_reach"):
        hit = sum(c["failures"].get(key, 0) for c in conds)
        row[key] = round(hit / max(total, 1), 4)
    return row


def append_row(csv_path: Path, row: dict) -> None:
    """One line per evaluated checkpoint, so the curve lives in one file.

    This is the artefact worth keeping.  Reading a plateau off individual
    reports means opening them one at a time and holding the numbers in your
    head; reading it off one column does not.
    """
    fresh = not csv_path.is_file()
    with csv_path.open("a", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(_CSV_COLUMNS))
        if fresh:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in _CSV_COLUMNS})


def done_iterations(csv_path: Path) -> set[int]:
    """Iterations already in the curve, so a restart does not redo them."""
    if not csv_path.is_file():
        return set()
    with csv_path.open(newline="", encoding="utf-8") as fh:
        return {
            int(r["iteration"])
            for r in csv.DictReader(fh)
            if (r.get("iteration") or "").isdigit()
        }


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="ssh host or alias")
    parser.add_argument("--remote-run", required=True,
                        help="run directory on the server")
    parser.add_argument("--interval", type=int, default=100,
                        help="evaluate checkpoints whose iteration divides this")
    parser.add_argument("--period", type=float, default=300.0,
                        help="seconds between polls")
    parser.add_argument("--n-envs", type=int, default=64)
    parser.add_argument("--episodes", type=int, default=64,
                        help="episodes per condition; the eval spreads these "
                             "over envs as a per-env quota")
    parser.add_argument("--render", type=int, default=0,
                        help="record this many macro steps for each checkpoint")
    parser.add_argument("--out", default=None,
                        help="local output directory (default: sim/rl_runs/eval/<run>)")
    parser.add_argument("--latest-only", action="store_true",
                        help="if several checkpoints are pending, evaluate only "
                             "the newest -- for when evaluation cannot keep up")
    parser.add_argument("--allow-commit-mismatch", action="store_true")
    parser.add_argument("--once", action="store_true", help="one pass, then exit")
    parser.add_argument("--set", action="append", default=[], dest="overrides",
                        help="passed through to eval, e.g. --set object.radius_m=0.06")
    args = parser.parse_args(argv)

    if shutil.which("rsync") is None:
        raise SystemExit("[watch] rsync 를 찾을 수 없습니다.")
    check_remote_path(args.remote_run)

    run_name = Path(args.remote_run.rstrip("/")).name
    local_run = _REPO_ROOT / "sim" / "rl_runs" / "wrap_grasp" / f"remote_{run_name}"
    out_dir = (
        Path(args.out) if args.out
        else _REPO_ROOT / "sim" / "rl_runs" / "eval" / run_name
    )
    csv_path = out_dir / "curve.csv"
    lock = out_dir / ".watch.lock"

    out_dir.mkdir(parents=True, exist_ok=True)
    if lock.exists():
        raise SystemExit(
            f"[watch] 이미 실행 중인 것 같습니다: {lock}\n"
            f"         아니라면 지우고 다시 실행하세요."
        )
    lock.write_text(str(os.getpid()))
    print(f"[watch] 서버   {args.host}:{args.remote_run}")
    print(f"[watch] 로컬   {local_run}")
    print(f"[watch] 출력   {out_dir}")
    print(f"[watch] 간격   {args.interval} iteration 마다, {args.period:.0f}초 주기")

    extra = []
    for item in args.overrides:
        extra += ["--set", item]

    try:
        open_master(args.host)
        while True:
            if not pull(args.host, args.remote_run, local_run):
                if args.once:
                    return 1
                time.sleep(args.period)
                continue
            try:
                check_commit(local_run, allow_mismatch=args.allow_commit_mismatch)
            except _NotReady:
                if args.once:
                    return 0
                time.sleep(args.period)
                continue

            already = done_iterations(csv_path)
            pending = [(i, p) for i, p in checkpoints(local_run, args.interval)
                       if i not in already]
            if args.latest_only and pending:
                skipped = len(pending) - 1
                pending = pending[-1:]
                if skipped:
                    # Say what was dropped.  A silent skip reads as "nothing to
                    # do" and the gap in the curve looks like a stall.
                    print(f"[watch] 대기 중 {skipped}개는 건너뜁니다 (--latest-only)")

            if not pending:
                print(f"[watch] 새 체크포인트 없음 ({len(already)}개 평가 완료)")
            for iteration, ckpt in pending:
                data = evaluate(
                    ckpt, out_dir,
                    n_envs=args.n_envs, episodes=args.episodes,
                    render=args.render, extra=extra,
                )
                if data is None:
                    continue
                row = summarise(iteration, ckpt, data)
                append_row(csv_path, row)
                print(
                    f"[watch] iter {iteration:6d}  성공 {100*row['success_rate']:5.1f}%  "
                    f"토플 {100*row['topple']:5.1f}%  housing {100*row['collision']:5.1f}%  "
                    f"Φ평균 {row['phi_mean_deg']:5.1f}deg"
                )

            if args.once:
                return 0
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("\n[watch] 중단")
        return 130
    finally:
        lock.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
