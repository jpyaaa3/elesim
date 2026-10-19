"""Real Genesis marker visibility probe; no DDS or physical I/O."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    import genesis as gs
    from elesim_sim.simulation.ik_marker import IkTargetMarker
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
        marker = IkTargetMarker()

        def capture(camera, name):
            rgb, _, _, _ = camera.render(rgb=True, depth=False, force_render=True)
            rgb = np.asarray(rgb)
            cv2.imwrite(str(args.output_dir/name), rgb[:, :, ::-1])
            green = ((rgb[:, :, 1] > 100) & (rgb[:, :, 1] > rgb[:, :, 0]*1.5)
                     & (rgb[:, :, 1] > rgb[:, :, 2]*1.5))
            return int(green.sum())

        marker.update(scene, (0., 0., .25), (0., 0., 1.))
        shown = capture(observer.camera, "ik-marker-visible.png")
        clean = capture(eye.camera, "ik-marker-hand-eye.png")
        marker.update(scene, (0., 0., .25), visible=False)
        hidden = capture(observer.camera, "ik-marker-hidden.png")
        assert shown > 15 and clean < shown/5 and hidden < shown/5, (shown, clean, hidden)
        result = dict(observer_green_pixels=shown, hand_eye_green_pixels=clean,
                      hidden_green_pixels=hidden, scope="real Genesis; no DDS or hardware")
        (args.output_dir/"result.json").write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(result))
    finally:
        gs.destroy()


if __name__ == "__main__":
    main()
