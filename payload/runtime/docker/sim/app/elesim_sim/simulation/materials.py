"""Link-specific contact materials; foot traction must not change arm friction."""
import math


def _checked(value):
    if not math.isfinite(value) or not .01<=value<=5:raise ValueError('invalid link friction')
    return value


def apply_foot_friction(entity,friction):
    friction=_checked(friction)
    for link in entity.links:
        if link.name.lower().endswith('_foot'):link.set_friction(friction)


def apply_link_friction(entity,*,arm=.5,feet=.8):
    arm,feet=_checked(arm),_checked(feet)
    for link in entity.links:
        link.set_friction(feet if link.name.lower().endswith('_foot') else arm)
