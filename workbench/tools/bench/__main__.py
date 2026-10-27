"""Run, verify and collect versioned EleSim benchmarks (repository-only)."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

from .catalog import CATALOG, build
from .evidence import index, pack, unpack, write_json
from .runner import LocalExecutor, exit_code, report, run, verify


def _bindings(values, *, json_values=False):
    result = {}
    for value in values:
        key, separator, text = value.partition("=")
        if not key or not separator or key in result:
            raise ValueError(f"expected unique NAME=VALUE: {value}")
        result[key] = json.loads(text) if json_values else text
    return result


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    commands = ap.add_subparsers(dest="command", required=True)
    commands.add_parser("list", help="list the five benchmark families and recipes")
    for verb in ("plan", "run"):
        cmd = commands.add_parser(verb)
        cmd.add_argument("benchmark", choices=tuple(r.name for r in CATALOG))
        cmd.add_argument("--input", action="append", default=[], metavar="NAME=PATH")
        cmd.add_argument("--param", action="append", default=[], metavar="NAME=JSON")
        if verb == "run":
            cmd.add_argument("--output", type=Path, required=True, help="new, empty run path")
            cmd.add_argument("--install", type=Path, help="use this installation's pinned Compose wrapper")
            cmd.add_argument("--instance", help="registered instance providing release images")
            cmd.add_argument("--gpu", help="physical GPU UUID; Compose exposes it as logical device 0")
    for verb in ("verify", "report"):
        commands.add_parser(verb).add_argument("run", type=Path)
    cmd = commands.add_parser("pack")
    cmd.add_argument("run", type=Path)
    cmd.add_argument("--output", type=Path, required=True)
    cmd = commands.add_parser("collect", help="verify and unpack a local/transferred evidence archive")
    cmd.add_argument("archive", type=Path)
    cmd.add_argument("--sha256", required=True)
    cmd.add_argument("--output", type=Path, required=True)
    cmd.add_argument("--host", help="saved topology host ID; fetch archive through its SSH endpoint")
    cmd.add_argument("--install", type=Path)
    cmd.add_argument("--manager", help="explicit running manager container owned by this installation")
    cmd.add_argument("--topology", type=Path)
    cmd = commands.add_parser("index", help="inventory legacy evidence without deleting or moving it")
    cmd.add_argument("root", type=Path)
    cmd.add_argument("--output", type=Path, required=True)
    return ap


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "list":
            for recipe in CATALOG:
                print(f"{recipe.family:19} {recipe.name:24} {recipe.description}")
                print(f"  inputs={','.join(recipe.inputs)} parameters={json.dumps(recipe.defaults)}")
            return 0
        if args.command in ("plan", "run"):
            plan = build(args.benchmark, _bindings(args.input), _bindings(args.param, json_values=True))
            if args.command == "plan":
                print(json.dumps(plan.document(), indent=2))
                return 0
            if bool(args.install) != bool(args.instance):
                raise ValueError("--install and --instance are required together")
            if args.install:
                from .compose import ComposeExecutor
                executor = ComposeExecutor(args.install, args.instance, gpu=args.gpu)
            else:
                executor = LocalExecutor(gpu=args.gpu)
            manifest = run(plan, args.output, executor)
            print(report(args.output))
            return exit_code(manifest)
        if args.command in ("verify", "report"):
            manifest = verify(args.run)
            print(report(args.run))
            return exit_code(manifest) if args.command == "verify" else 0
        if args.command == "pack":
            print(json.dumps(pack(args.run, args.output)))
        elif args.command == "collect":
            archive = args.archive
            if args.host:
                if not all((args.install, args.manager, args.topology)):
                    raise ValueError("remote collection needs --install, --manager, --topology and --host")
                from .remote import fetch
                archive = fetch(prefix=args.install, manager=args.manager, topology=args.topology,
                                host=args.host, archive=args.archive,
                                destination=args.output.with_name(args.output.name + ".zip"))
            elif any((args.install, args.manager, args.topology)):
                raise ValueError("--host is required for remote collection")
            manifest = unpack(archive, args.output, args.sha256)
            print(json.dumps({"collected": str(args.output), "complete": manifest["complete"],
                              "accepted": manifest["accepted"]}))
        elif args.command == "index":
            write_json(args.output, index(args.root))
            print(args.output)
        return 0
    except (ValueError, OSError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"Benchmark error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
