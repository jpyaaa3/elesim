"""Versioned benchmark recipes. Commands are argv arrays, never shell snippets.

Calculation workers stay in research; this catalog owns inputs, scope and gates.
Every exposed parameter is consumed here, before any process is started.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
EXPERIMENTS = "workbench.research.experiments."
ANALYSIS = "workbench.research.analysis."
SCHEMA_VERSION = 1
FAMILIES = ("walking", "sim-performance", "visual-calibration", "multiview", "completion")


def source_closure(*modules: str) -> tuple[Path, ...]:
    """Mount the consumer's declared Python imports, not private scene code/data."""
    import ast
    from importlib.util import resolve_name
    pending, seen, files = list(modules), set(), set()
    while pending:
        module = pending.pop()
        if module in seen or not module.startswith("workbench"):
            continue
        seen.add(module)
        path = ROOT.joinpath(*module.split(".")).with_suffix(".py")
        if not path.is_file():
            path = ROOT.joinpath(*module.split("."), "__init__.py")
        if not path.is_file():
            continue
        files.add(path)
        for parent in path.parents:
            if parent == ROOT:
                break
            if (parent / "__init__.py").is_file():
                files.add(parent / "__init__.py")
        package = module if path.name == "__init__.py" else module.rpartition(".")[0]
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                pending.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                base = resolve_name("."*node.level + (node.module or ""), package) if node.level else node.module
                if base:
                    pending.append(base)
                    pending.extend(base + "." + alias.name for alias in node.names)
    return tuple(sorted(files))


@dataclass(frozen=True)
class Stage:
    name: str
    module: str
    args: tuple[str, ...]
    role: str = "pilot"
    inputs: tuple[str, ...] = ()
    group: str = ""
    phase: str = "run"
    gpu: bool = False
    timeout_s: float = 1800
    # Only these output subtrees are mounted, keeping private truth out of Pilot.
    reads: tuple[str, ...] = ()
    writes: tuple[str, ...] = ()


@dataclass(frozen=True)
class Recipe:
    name: str
    family: str
    description: str
    inputs: tuple[str, ...]
    defaults: dict[str, Any]
    scope: dict[str, Any]


@dataclass
class Plan:
    name: str
    family: str
    scope: dict[str, Any]
    inputs: dict[str, str]
    parameters: dict[str, Any]
    stages: list[Stage]
    result: str
    gate: str = "accepted"
    schema_version: int = SCHEMA_VERSION

    def document(self) -> dict:
        return asdict(self)


CATALOG = (
    Recipe("walking-core", "walking", "Production locomotion, progress and repeated stop gates",
           ("config", "data"), {"duration": 30., "repeat": 5, "cycles": 3,
           "command": [.8, 0., 0.]},
           {"clock": "simulation", "physics": True, "mppi": True, "dds": False,
            "webrtc": False, "cameras": False, "arm": "hold", "hardware": False}),
    Recipe("walking-gaze", "walking", "DDS walking/gaze scenarios, isolated simulation only",
           ("config", "pilot_config"), {"suite": "full", "duration": 30., "trials": 5,
           "trial_start": 1},
           {"clock": "wall", "physics": True, "mppi": True, "dds": True,
            "webrtc": "Sim media service; no viewer connected", "cameras": "from Sim config", "hardware": False}),
    Recipe("sim-step", "sim-performance", "Production physics/MPPI loop; optional render workers",
           ("config", "data"), {"duration": 10., "repeat": 3, "cycles": 1,
           "command": [.15, 0., 0.], "cameras": False, "arm_motion": False, "budget_ms": 10.},
           {"clock": "simulation", "physics": True, "mppi": True, "dds": False,
            "webrtc": False, "hardware": False,
            "step_timer": "command + arm hold + physics + camera dispatch; diagnostic reads excluded",
            "throughput_timer": "entire trial including diagnostic reads; scene startup excluded"}),
    Recipe("visual-tilt", "visual-calibration", "Four-DOF error and plane-tilt matrix; preserve FK origin",
           ("config", "data"), {"backend": "analytic", "poses": 24, "error_seeds": 2, "seed": 217},
           {"clock": "stages", "dds": False, "webrtc": False, "hardware": False,
            "truth": "producer/evaluator only", "height_constraint": False, "cameras": True}),
    Recipe("visual-tilt-replay", "visual-calibration", "Re-evaluate existing tilt evidence without recapture",
           ("truth", "predictions"), {}, {"clock": "wall", "replay": True, "hardware": False}),
    Recipe("multiview-scan", "multiview", "Continuous self-collision checks and sensor-only scan fusion",
           ("data",), {"backend": "analytic", "max_views": 12, "view_policy": "information",
           "candidates": 12000},
           {"clock": "stages", "dds": False, "webrtc": False, "hardware": False,
            "truth": "producer/evaluator only", "motion": "kinematic; fixed base; six fixtures", "cameras": True}),
    Recipe("multiview-compare", "multiview", "Fewer captures with paired quality and collision gates",
           ("baseline", "candidate"), {}, {"clock": "wall", "replay": True, "hardware": False}),
    Recipe("completion-adapointr", "completion", "Sensor-only AdaPoinTr inference, then independent evaluation",
           ("clouds", "truth", "vendor", "checkpoint", "model_config"),
           {"profile": "pcn", "normalization": "observed-radius", "seed": 0},
           {"clock": "wall", "gpu": True, "hardware": False, "truth": "evaluator only"}),
    Recipe("completion-replay", "completion", "Geometry, latency and observation-preservation gates",
           ("truth", "predictions"), {}, {"clock": "wall", "replay": True, "hardware": False}),
)


def _parameters(recipe: Recipe, values: dict) -> dict:
    unknown = set(values) - recipe.defaults.keys()
    if unknown:
        raise ValueError(f"unused/unknown parameters: {sorted(unknown)}")
    params = {**recipe.defaults, **values}
    choices = {"backend": ("analytic", "genesis"), "view_policy": ("information", "angular"),
               "profile": ("pcn", "shapenet55", "projected55"),
               "normalization": ("observed-radius", "center-only", "native"),
               "suite": ("full", "resume", "ablation", "preview-sign")}
    for name, value in params.items():
        default = recipe.defaults[name]
        if name in choices:
            if value not in choices[name]:
                raise ValueError(f"{name} must be one of {choices[name]}")
        elif type(default) is bool:
            if type(value) is not bool:
                raise ValueError(f"{name} must be a JSON boolean")
        elif name == "command":
            import math
            if (not isinstance(value, list) or len(value) != 3 or
                    any(type(v) not in (float, int) or not math.isfinite(v) for v in value)):
                raise ValueError("command must be three finite numbers")
        elif isinstance(default, (float, int)):
            import math
            if (type(value) not in (int, float) or not math.isfinite(value) or
                    (type(default) is int and type(value) is not int) or
                    (value < 0 if name == "seed" else value <= 0)):
                raise ValueError(f"invalid numeric parameter: {name}={value}")
    for name, lower, upper in (("poses", 4, 128), ("error_seeds", 1, 4),
                               ("max_views", 4, 24), ("candidates", 100, 30000)):
        if name in params and not lower <= params[name] <= upper:
            raise ValueError(f"{name} must be {lower}..{upper}")
    return params


def build(name: str, inputs: dict[str, str], parameters: dict | None = None) -> Plan:
    recipe = next((r for r in CATALOG if r.name == name), None)
    if recipe is None:
        raise ValueError(f"unknown benchmark: {name}")
    if set(inputs) != set(recipe.inputs):
        raise ValueError(f"inputs required={recipe.inputs}; supplied={tuple(inputs)}")
    p = _parameters(recipe, parameters or {})
    stages = []

    def stage(label, module, args, **kwargs):
        stages.append(Stage(label, module, tuple(map(str, args)), **kwargs))

    result, gate = "evaluation.json", "accepted"
    if name in ("walking-core", "sim-step"):
        args = ["{config}", "{data}/models/assemblies/zed-mini", "--duration", p["duration"],
                "--repeat", p["repeat"], "--cycles", p["cycles"], "--command", *p["command"],
                "--output", "{run}/probe/walking.jsonl"]
        if p.get("cameras"):
            args += ["--cameras"]
        if p.get("arm_motion"):
            args += ["--arm-motion"]
        stage("probe", "workbench.research.debug.profile_sim_walking", args, role="sim",
              inputs=recipe.inputs, gpu=True, writes=("probe",), timeout_s=7200)
        result, gate = "probe/walking.jsonl", "walking" if name == "walking-core" else "performance"
    elif name == "walking-gaze":
        stage("suite", EXPERIMENTS + "walking_suite", ["--pilot-config", "{pilot_config}",
              "--sim-config", "{config}", "--suite", p["suite"], "--duration", p["duration"],
              "--trials", p["trials"], "--trial-start", p["trial_start"], "--output", "{run}/suite"],
              role="workspace", inputs=recipe.inputs, gpu=True, writes=("suite",), timeout_s=14400)
        result = "suite/result.json"
    elif name == "visual-tilt":
        stage("scene", EXPERIMENTS + "vision_arm_scene", ["{run}/scene", "--config", "{config}",
              "--data-root", "{data}", "--backend", p["backend"], "--poses", p["poses"],
              "--error-seeds", p["error_seeds"], "--seed", p["seed"], "--joint-errors"],
              role="sim", inputs=recipe.inputs, gpu=p["backend"] == "genesis", writes=("scene",))
        stage("reconstruct", EXPERIMENTS + "vision_reconstruct", ["{run}/scene/sensors", "{run}/reconstructed",
              "--arm-model", "{data}/models/arm/default.json",
              "--hand-eye", "{data}/calibration/cameras/zed_mini.hand_eye.json"],
              inputs=("data",), reads=("scene/sensors",), writes=("reconstructed",))
        stage("evaluate", EXPERIMENTS + "vision_tilt_evaluate", ["{run}/scene/truth", "{run}/reconstructed",
              "--output", "{run}/evaluation/result.json"], phase="evaluate",
              reads=("scene/truth", "reconstructed"), writes=("evaluation",))
        result = "evaluation/result.json"
    elif name == "multiview-scan":
        stage("scene", EXPERIMENTS + "arm_scan_scene", ["{run}/requests", "{run}/scene",
              "--data-root", "{data}", "--backend", p["backend"]], role="sim", inputs=("data",),
              gpu=p["backend"] == "genesis", group="capture", reads=("requests",), writes=("scene",))
        stage("scan", EXPERIMENTS + "arm_scan", ["{run}/requests", "{run}/scene/sensors", "{run}/scans",
              "--arm-model", "{data}/models/arm/default.json",
              "--hand-eye", "{data}/calibration/cameras/zed_mini.hand_eye.json",
              "--collision-model", "{data}/models/planning/zed-mini", "--max-views", p["max_views"],
              "--view-policy", p["view_policy"], "--candidates", p["candidates"]], inputs=("data",),
              group="capture", reads=("scene/sensors",), writes=("requests", "scans"))
        stage("evaluate", EXPERIMENTS + "arm_scan_evaluate", ["{run}/scene/truth", "{run}/scans",
              "--data-root", "{data}", "--output", "{run}/evaluation/result.json"],
              inputs=("data",), phase="evaluate", reads=("scene/truth", "scans"), writes=("evaluation",))
        result = "evaluation/result.json"
    elif name == "multiview-compare":
        stage("evaluate", ANALYSIS + "arm_scan_compare", ["{baseline}", "{candidate}", "--output",
              "{run}/evaluation/result.json"], inputs=recipe.inputs, phase="evaluate", writes=("evaluation",))
        result = "evaluation/result.json"
    elif name == "completion-adapointr":
        stage("infer", EXPERIMENTS + "adapointr_infer", ["{clouds}", "{run}/predictions", "--vendor",
              "{vendor}", "--checkpoint", "{checkpoint}", "--checkpoint-sha256", "{checkpoint_sha256}",
              "--config", "{model_config}", "--profile", p["profile"], "--normalization", p["normalization"],
              "--seed", p["seed"]], inputs=tuple(n for n in recipe.inputs if n != "truth"),
              gpu=True, writes=("predictions",))
        stage("evaluate", EXPERIMENTS + "adapointr_evaluate", ["{truth}", "{run}/predictions",
              "--output", "{run}/evaluation/result.json"], inputs=("truth",), phase="evaluate",
              reads=("predictions",), writes=("evaluation",))
        result = "evaluation/result.json"
    else:
        module = "vision_tilt_evaluate" if name == "visual-tilt-replay" else "adapointr_evaluate"
        stage("evaluate", EXPERIMENTS + module, ["{truth}", "{predictions}", "--output",
              "{run}/evaluation/result.json"], inputs=recipe.inputs, phase="evaluate", writes=("evaluation",))
        result = "evaluation/result.json"
    scope = {**recipe.scope}
    if "backend" in p:
        scope["backend"] = p["backend"]
    if name == "sim-step":
        scope.update(cameras=p["cameras"], arm="motion" if p["arm_motion"] else "hold")
    return Plan(name, recipe.family, scope, {n: str(Path(v).expanduser().resolve()) for n, v in inputs.items()},
                p, stages, result, gate)
