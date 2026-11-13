"""Explicit immutable reverse evidence reuse; never rewrite old provenance."""
from pathlib import Path
import json
import shutil
from .contracts import verify_files


def compatible(study,case,old):
    from workbench.research.experiments.hug_reverse_ga.prepare import digest
    if (old.get('schema')!='hug-reverse-ga-plan' or old.get('version')!=1 or old.get('source_sha256')!=digest()
        or old['settings']!=study['settings'] or old['home']!=study['home']
        or old['simulation_context']['model_sha256']!=study['shared_context']['model_sha256']):return False
    fields=('diameter_mm','q_deg','xy_mm','feature','root','goal_command','object_xyz_m','roll_deg','band')
    return all(old['cases'][0].get(k)==case.get(k) for k in fields)


def reuse_reverse(study,case,output):
    from .reverse import prepare_reference as prepare
    for item in study.get('reverse_inputs',[]):
        origin=Path(item['directory']);verify_files(origin,item['artifacts'])
        old=json.loads((origin/'plan.json').read_text())
        if not compatible(study,case,old):continue
        # Revalidate all three releases, winner validity and source/model before
        # reuse. Original JSON/NPZ bytes and simulation context are kept intact.
        prepare(origin,output/'reuse-validation')
        shutil.copytree(origin,output/'plan')
        return dict(accepted=True,status='reverse_path_verified',replays=[True]*3,
            reused_from=str(origin),old_provenance_rewritten=False,
            new_case_reset_validation_required=True)
    return None
