"""URDF visual meshes for a planned arm; never creates a physics entity."""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def sample_preview(preview, elapsed):
    """Interpolate visual samples in wall time, with a short final-pose pause."""
    if preview is None or elapsed > 120:
        return None
    times = np.asarray(preview["times_s"], dtype=float)
    positions = np.asarray(preview["positions"], dtype=float)
    t = min(max(0., elapsed) % (times[-1]+.75), times[-1])
    return tuple(float(np.interp(t, times, positions[:, i])) for i in range(4))


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        transform[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")).as_matrix()
    return transform


class ArmGhost:
    def __init__(self, urdf):
        self.path = Path(urdf)
        self.meshes = None
        self.handles = []
        self.signature = None

    def _load(self):
        import trimesh
        tree = ET.parse(self.path).getroot()
        self.joints = list(tree.findall("joint"))
        revolute = [j.get("name") for j in self.joints if j.get("type") == "revolute"]
        self.roll, self.bends = revolute[0], revolute[1:]
        if not self.bends or len(self.bends) % 2:
            raise ValueError("Ghost requires the coupled arm's two equal bend segments")
        children = {j.find("child").get("link") for j in self.joints}
        self.root = next(link.get("name") for link in tree.findall("link") if link.get("name") not in children)
        self.meshes = []
        cache = {}
        for link in tree.findall("link"):
            for visual in link.findall("visual"):
                node = visual.find("geometry/mesh")
                if node is None:
                    continue
                path = (self.path.parent/node.get("filename")).resolve()
                if path not in cache:
                    cache[path] = trimesh.load(path, force="mesh", process=False)
                mesh = cache[path].copy()
                mesh.apply_scale(np.fromstring(node.get("scale", "1 1 1"), sep=" "))
                mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, face_colors=(25, 255, 75, 85))
                self.meshes.append((link.get("name"), mesh, origin(visual.find("origin"))))

    def transforms(self, q, base):
        if self.meshes is None:
            self._load()
        values = {"j_plate_housing":q[0], self.roll:q[1]}
        values.update({name:q[2 if i < len(self.bends)//2 else 3] for i,name in enumerate(self.bends)})
        transforms = {self.root:np.asarray(base, dtype=float)}
        pending = list(self.joints)
        while pending:
            remaining = []
            for joint in pending:
                parent = joint.find("parent").get("link")
                if parent not in transforms:
                    remaining.append(joint)
                    continue
                movement = np.eye(4)
                value = values.get(joint.get("name"), 0.)  # planned claws are open
                axis = joint.find("axis")
                axis = np.fromstring(axis.get("xyz", "1 0 0"), sep=" ") if axis is not None else np.array([1.,0,0])
                if joint.get("type") == "prismatic":
                    movement[:3,3] = axis*value
                elif joint.get("type") in {"revolute", "continuous"}:
                    movement[:3,:3] = Rotation.from_rotvec(axis*value).as_matrix()
                transforms[joint.find("child").get("link")] = transforms[parent] @ origin(joint.find("origin")) @ movement
            if len(remaining) == len(pending):
                raise ValueError("Ghost URDF contains disconnected or cyclic joints")
            pending = remaining
        return transforms

    def update(self, scene, q, base, *, visible=True):
        signature = None if not visible or q is None else tuple(np.r_[q,np.asarray(base).reshape(-1)])
        if signature == self.signature:
            return
        for handle in self.handles:
            scene.clear_debug_object(handle)
        self.handles.clear()
        self.signature = None
        if signature is None:
            return
        transforms = self.transforms(q,base)
        for link, mesh, visual in self.meshes:
            self.handles.append(scene.draw_debug_mesh(mesh, T=transforms[link] @ visual))
        self.signature = signature
