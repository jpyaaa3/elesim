"""Compose a real-time demo from recorded UI streams and measured stage data."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import textwrap


def probe(path):
    result = subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
        '-show_entries','stream=width,height,codec_name,avg_frame_rate:format=duration',
        '-of','json',str(path)],text=True)
    return json.loads(result)


def stamp(seconds):
    ticks = max(0,round(seconds*100))
    return f'{ticks//360000}:{ticks//6000%60:02d}:{ticks//100%60:02d}.{ticks%100:02d}'


def safe(value):
    return str(value).replace('\\','/').replace('{','(').replace('}',')').replace('\n',' ')


def label(sample):
    stage,phase = sample['stage'],sample['phase']
    if stage == 'multi-view scan':
        index = (sample.get('view') or 0)+1
        return {'loading_models':'Prepare 12,000 view candidates',
            'moving':f'Move to view {index}/3', 'capturing':f'Capture RGB-D: view {index}/3',
            'choosing_view':'Choose the next safe viewpoint', 'reconstructing':'Fuse points and build mesh',
            'completed':'Scan complete'}.get(phase,safe(phase).replace('_',' ').capitalize())
    return {'demo initial view':'Original virtual object', 'steady simulation feedback':'Check live feedback',
        'torque_on':'Enable virtual motors', 'Sim torque feedback':'Confirm virtual motor state',
        'scan prerequisites':'Ready to scan', 'post-scan control authority':'Return arm control',
        'send_sim_target_xyz':'Remove original fixture', 'original fixture hidden':'Remove original fixture',
        'apply reconstructed scene':'Load reconstructed mesh', 'fresh WebRTC after rebuild':'Wait for new live frames',
        'demo final view':'Reconstructed mesh in the world', 'complete':'Cycle complete'}.get(stage,safe(stage))


def subtitles(path,rows,duration,ui,evaluation,sim,cache_note):
    header = '''[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 720
WrapStyle: 0
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Title,DejaVu Sans,25,&H00FFFFFF,&H00FFFFFF,&H00101827,&H00000000,-1,0,0,0,100,100,0,0,1,1,0,7,16,16,12,1
Style: Label,DejaVu Sans,17,&H00FFFFFF,&H00FFFFFF,&H00000000,&H00000000,-1,0,0,0,100,100,0,0,1,1,0,7,12,12,12,1
Style: Stage,DejaVu Sans,22,&H00E5F2FF,&H00FFFFFF,&H00101827,&H00000000,-1,0,0,0,100,100,0,0,1,0,0,7,12,16,12,1
Style: Data,DejaVu Sans,17,&H00CFD8E3,&H00FFFFFF,&H00101827,&H00000000,0,0,0,0,100,100,0,0,1,0,0,7,12,16,12,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
'''
    events = []
    def event(start,end,style,x,y,text):
        events.append(f'Dialogue: 0,{stamp(start)},{stamp(end)},{style},,0,0,0,,{{\\pos({x},{y})}}{text}')
    shape = sim.get('truth',{}).get('shape','')
    event(0,duration,'Title',16,15,'EleSim | '+(shape.upper()+' | ' if shape else '')+'RGB-D -> mesh -> world')
    event(0,duration,'Label',14,78,'HAND-EYE / real WebRTC')
    event(0,duration,'Label',974,78,'OBSERVER / real WebRTC')
    event(0,duration,'Data',16,640,'Baseline: HSV + FK | observed surfaces only | 20 mm voxel mesh')
    event(0,duration,'Data',16,678,'Actual wall time / 1x playback | recorded at 10 fps | Genesis 1.4.5 | GPU 2')
    segments = []
    for row in rows:
        key = (label(row),row.get('accepted_views',0),row.get('observed_points',0))
        if not segments or segments[-1][1] != key:
            segments.append((row['t_s'],key,row))
    if segments:
        segments[0] = (0.,segments[0][1],segments[0][2])
    for index,(start,key,row) in enumerate(segments):
        end = min(duration,segments[index+1][0] if index+1<len(segments) else duration)
        title,accepted,points = key
        event(start,end,'Stage',980,282,'\\N'.join(textwrap.wrap(title,23)))
        event(start,end,'Data',980,382,f'Accepted views: {accepted}/3\\NObserved points: {points:,}\\NFixed arm / Sim only\\NHeadless UI backend')
        if row['stage'] in ('demo final view','complete') and evaluation['complete']:
            metrics = evaluation['objects'][0]
            error = metrics.get('observed_surface_error_p95_m',metrics.get('observed_radial_error_p95_m'))*1000
            elapsed = sim['edits'][-1]['elapsed_s']
            note = '\\N'.join(textwrap.wrap(safe(cache_note),30))
            event(start,end,'Data',980,490,f'Point p95: {error:.2f} mm\\N(vs Sim truth)\\NMesh import: {elapsed:.2f} s\\N'+note)
    path.write_text(header+'\n'.join(events)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cache-note',default='Cache condition not characterized')
    args = parser.parse_args()
    root = args.input.resolve()
    ui = json.loads((root/'ui-output/ui.json').read_text())
    timeline = json.loads((root/'ui-output/video-timeline.json').read_text())
    evaluation = json.loads((root/'sim-output/evaluation.json').read_text())
    sim = json.loads((root/'sim-output/sim.json').read_text())
    assert ui['complete'] and timeline['metadata']['complete'] and evaluation['complete']
    assert ui['hidden_red_pixels'] == {'observer':0,'hand_eye_preview':0}
    paths = [root/'ui-output'/timeline['metadata']['files'][name] for name in ('hand_eye_preview','observer')]
    probes = [probe(path) for path in paths]
    duration = min(float(value['format']['duration']) for value in probes)
    assert duration > 10 and not args.output.exists()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    subtitle = root/'demo.ass'
    subtitles(subtitle,timeline['samples'],duration,ui,evaluation,sim,args.cache_note)
    escaped = str(subtitle).replace('\\','\\\\').replace(':','\\:').replace("'","\\'")
    graph = ('[0:v]setpts=PTS-STARTPTS,scale=960:540:flags=lanczos,setsar=1[eye];'
        '[1:v]setpts=PTS-STARTPTS,scale=320:180:flags=lanczos,setsar=1[observer];'
        f'color=c=0x101827:s=1280x720:r=10:d={duration:.3f}[canvas];'
        '[canvas][eye]overlay=0:72:shortest=1[base];'
        f"[base][observer]overlay=960:72:shortest=1,ass='{escaped}'[video]")
    command = ['ffmpeg','-nostdin','-hide_banner','-loglevel','warning','-n',
        '-i',str(paths[0]),'-i',str(paths[1]),'-filter_complex_threads','2','-filter_complex',graph,
        '-map','[video]','-t',f'{duration:.3f}','-r','10','-c:v','libx264','-threads','2',
        '-preset','medium','-crf','24','-pix_fmt','yuv420p','-movflags','+faststart','-an',str(args.output)]
    subprocess.run(command,check=True,timeout=300)
    metadata = dict(scope='actual UI WebRTC recording, composed with measured stage captions',
        playback_speed=1,mesh_cache_note=args.cache_note,rendered=probe(args.output),bytes=args.output.stat().st_size,
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        source={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in paths})
    args.output.with_suffix('.metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata),flush=True)


if __name__ == '__main__':
    main()
