"""Run fixed seeds and controlled sensor perturbations; retain failures."""
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
cases=[(seed,.006,0.) for seed in range(6)]+[(0,.012,0.),(0,.006,4.),(1,.012,0.),(1,.006,4.)]
results=[]
for seed,noise,dropout in cases:
    path=ROOT/'outputs/monocular'/(f'room_seed{seed}_noise{noise:g}_drop{dropout:g}'.replace('.', 'p')+'.json')
    if path.exists():path.unlink() # Never treat a stale result as a new run.
    run=subprocess.run([sys.executable,str(ROOT/'main.py'),'--headless','--seed',str(seed),
                        '--lidar-sigma',str(noise),'--camera-dropout',str(dropout)],cwd=ROOT)
    path=ROOT/'outputs/monocular'/(f'room_seed{seed}_noise{noise:g}_drop{dropout:g}'.replace('.', 'p')+'.json')
    result=json.loads(path.read_text()) if path.exists() else {'seed':seed,'passed':False,'error':'no result'}
    result['exit_code']=run.returncode;result['passed']=bool(result.get('passed') and run.returncode==0);results.append(result)
(ROOT/'outputs/monocular/validation_summary.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(f"{sum(r['passed'] for r in results)}/{len(results)} passed")
sys.exit(0 if all(r['passed'] for r in results) else 1)
