"""Derive an interpolation-only study from verified frozen anchor evidence."""
import argparse
import json
import shutil
from pathlib import Path
from .contracts import canonical, digest, save, sha, shared, verify_files
from .coordinator import Coordinator, lock

SUPPORTED_SOURCE = 'cedf8ab344896f2be524ecf8ef0f2963dcc50e2dc40e27ab08088da1e0a2a2a9'


def prepare(source, output):
    source=Path(source).resolve();output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    old=json.loads((source/'plan.json').read_text())
    manifest=json.loads((source/'manifest.json').read_text())
    if old['source_sha256']!=SUPPORTED_SOURCE or manifest['plan_sha256']!=sha(source/'plan.json'):
        raise ValueError('Unsupported or changed source study')
    expected=shared(old['settings'])
    if old['shared_context']!=dict(expected,code_sha256=SUPPORTED_SOURCE):
        raise ValueError('Physics, control or reward contract changed')
    if sha(source/'mapper.html')!=old['mapper_sha256']:raise ValueError('Mapper changed')
    selection=json.loads((source/'selection.json').read_text())
    if old.get('fixed_selection_sha256')!=sha(source/'selection.json'):
        # The coordinator reserializes the pinned selection; compare contents.
        pinned=source/'fixed-selection.json'
        if sha(pinned)!=old['fixed_selection_sha256'] or json.loads(pinned.read_text())!=selection:
            raise ValueError('Selection changed')
    anchors=[]
    for case in selection['cases'][:3]:
        key=case['case_id']
        stages={kind:manifest['stages'][key+'/'+kind] for kind in ('train','policy','replay')}
        for kind,s in stages.items():
            if s['status']!='complete' or s['result'].get('accepted') is not True:
                raise ValueError('Unverified anchor '+key)
            verify_files(Path(s['output']),s['artifacts'])
            if kind!='train':
                r=s['result']
                if r.get('case_sha256')!=canonical(case) or r.get('repetitions')!=3 or len(r.get('results',[]))!=3 or not all(x['accepted'] for x in r['results']):
                    raise ValueError('Incomplete physical gate')
        anchors.append(dict(case=case,policy=str(Path(stages['train']['output'])/'training/policy.npz'),
                            trajectory=str(Path(stages['replay']['output'])/'rollout-1.npz')))
    output.mkdir(parents=True)
    shutil.copyfile(source/'mapper.html',output/'mapper.html')
    provenance=dict(source=str(source),source_plan_sha256=sha(source/'plan.json'),
                    source_manifest_sha256=sha(source/'manifest.json'),source_code_sha256=SUPPORTED_SOURCE,
                    operation='alignment-v2; no training; inherited verified anchor policies',anchors=[])
    for a in anchors:
        folder=output/'anchors'/a['case']['case_id'];folder.mkdir(parents=True)
        record=dict(case_id=a['case']['case_id'])
        for field,name in (('policy','policy.npz'),('trajectory','rollout.npz')):
            src=Path(a[field]);dst=folder/name
            record[field]=dict(source=str(src),sha256=sha(src))
            shutil.copyfile(src,dst);a[field]=str(dst)
        provenance['anchors'].append(record)
    plan=dict(old,source_sha256=digest(),shared_context=expected)
    save(output/'plan.json',plan);save(output/'selection.json',selection)
    save(output/'provenance.json',provenance)
    return anchors,selection['cases'][3:]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();anchors,held=prepare(a.source,a.output)
    with lock(a.output):
        c=Coordinator(a.output)
        try:c.interpolate(anchors,held)
        except Exception as exc:
            c.state.update(status='execution_error',error=repr(exc));c.write();raise


if __name__=='__main__':main()
