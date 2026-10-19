"""Real Genesis mesh Ghost visibility and alpha probe; no DDS or physical I/O."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    import genesis as gs
    from elesim_sim.simulation.arm_ghost import ArmGhost
    from elesim_sim.vision.sim_camera.mount import ObserverCamera, Node9EyeInHandCamera
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    gs.init(backend=gs.cpu, logging_level="warning")
    try:
        scene = gs.Scene(show_viewer=False)
        scene.add_entity(gs.morphs.Plane())
        observer = ObserverCamera.create(scene, res=(320, 240),
            pos=(.7, -1., .7), lookat=(0., 0., .25))
        eye = Node9EyeInHandCamera.create(scene, res=(320, 240))
        scene.build()
        eye.camera.set_pose(pos=(.7, -1., .7), lookat=(0., 0., .25))
        ghost = ArmGhost(Path(__file__).resolve().parents[3]/"payload/data/models/assemblies/zed-mini/arm.urdf")
        base = np.eye(4); base[2,3] = .2

        def capture(camera, name):
            rgb, _, _, _ = camera.render(rgb=True, depth=False, force_render=True)
            rgb = np.asarray(rgb)
            cv2.imwrite(str(args.output_dir/name), rgb[:, :, ::-1])
            green = ((rgb[:, :, 1] > 100) & (rgb[:, :, 1] > rgb[:, :, 0]*1.5)
                     & (rgb[:, :, 1] > rgb[:, :, 2]*1.5))
            return int(green.sum())

        ghost.update(scene, (-.1,0,.2,-.2), base)
        shown = capture(observer.camera, "ghost-visible.png")
        clean = capture(eye.camera, "ghost-hand-eye.png")
        materials = [h.primitives[0].material for h in ghost.handles]
        assert all(m.alphaMode == "BLEND" for m in materials)
        ghost.update(scene, (-.13,.5,-.1,.3), base)
        moved = capture(observer.camera, "ghost-moved.png")
        ghost.update(scene, None, None)
        hidden = capture(observer.camera, "ghost-hidden.png")
        assert shown > 100 and moved > 100 and clean < shown/5 and hidden < shown/5, (shown,moved,clean,hidden)
        result = dict(observer_green_pixels=shown, moved_green_pixels=moved,
                      hand_eye_green_pixels=clean, hidden_green_pixels=hidden,
                      alpha_mode="BLEND", opacity=85/255,
                      scope="real Genesis; no DDS or hardware")
        (args.output_dir/"result.json").write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(result))
    finally:
        gs.destroy()


if __name__ == "__main__":
    main()
