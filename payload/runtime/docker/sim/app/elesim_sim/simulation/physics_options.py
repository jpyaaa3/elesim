"""Explicit engine options that belong in the replay compatibility profile."""


def urdf_options():
    # Match the teacher's collision decomposition, rather than Genesis' inf
    # default (one convex hull can fill a concave grasping surface).
    return dict(default_armature=0.,decompose_robot_error_threshold=.15)


def rigid_options():
    # Preserve production Newton convergence while reserving the teacher's
    # contact capacity for the complete robot and object.
    return dict(iterations=50,noslip_iterations=0,max_collision_pairs=256,max_contacts=512)
