"""Synthetic preparation declarations for routing tests, never physics evidence."""
import json
from workbench.research.experiments.hug_c.configure import asset_digest,load_mechanism


def preparation_contract():
    presets=load_mechanism().presets
    return dict(schema='elesim-mapper-preparation-envelope',schema_version=1,
                model_sha256=asset_digest(),home_command=list(presets['home']),
                extend_command=list(presets['extend_arm']),clearance_mm=1.,sampling_padding_mm=.1)


def with_preparation(html,blocked='false'):
    declaration='const PREPARATION_COLLISION='+json.dumps(preparation_contract())+';\n'
    declaration+='function preparationReport(c,D){return {blocked:'+blocked+',minimum_gap_mm:12,required_clearance_mm:1.1};}\n'
    declaration+='function preparationCollision(c,D){return preparationReport(c,D).blocked;}\n'
    return html.replace('const CONTACT_OVERLAP_TOL_MM=1;','const CONTACT_OVERLAP_TOL_MM=1;\n'+declaration)
