"""Additive v6 Sim-only bounded timed-arm proposal; never creates authority."""
import math

CAPABILITY = 'simulation.timed_arm.v1'
MAX_SAMPLES = 2401


def validate(raw):
    if not isinstance(raw,dict) or set(raw)-{'id','operation','dt_s','q'}:
        raise ValueError('invalid timed arm fields')
    ident=raw.get('id')
    if not isinstance(ident,str) or not 1<=len(ident)<=64 or not ident.replace('-','').isalnum():
        raise ValueError('invalid timed arm id')
    op=raw.get('operation')
    if op not in {'start','keepalive','cancel'}:raise ValueError('invalid timed arm operation')
    if op!='start':
        if set(raw)!={'id','operation'}:raise ValueError('timed arm heartbeat has extra fields')
        return dict(raw)
    dt=raw.get('dt_s');q=raw.get('q')
    if isinstance(dt,bool) or not isinstance(dt,(float,int)) or not math.isfinite(dt) or not .005<=dt<=.05:
        raise ValueError('invalid timed arm dt')
    if not isinstance(q,list) or not 2<=len(q)<=MAX_SAMPLES or (len(q)-1)*dt>24.:
        raise ValueError('invalid timed arm size / duration')
    out=[]
    for row in q:
        if not isinstance(row,(list,tuple)) or len(row)!=4:raise ValueError('invalid timed arm q')
        if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in row):
            raise ValueError('invalid timed arm number')
        if abs(row[0])>.3 or any(abs(v)>math.pi for v in row[1:]):raise ValueError('timed arm outside structural limits')
        out.append([float(v) for v in row])
    return dict(id=ident,operation=op,dt_s=float(dt),q=out)
