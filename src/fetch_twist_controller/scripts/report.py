#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('csv'); args=p.parse_args()
    with open(args.csv) as f: rows=list(csv.DictReader(f))
    if len(rows)<2: raise SystemExit('Need at least two measured samples; no results generated.')
    keys=['vx','vy','vz','wx','wy','wz']
    command=np.array([[float(r['command_'+k]) for k in keys] for r in rows])
    measured=np.array([[float(r['measured_'+k]) for k in keys] for r in rows])
    t=np.array([float(r['time']) for r in rows]); t-=t[0]
    error=measured-command; active=np.linalg.norm(command,axis=1)>1e-8
    def rms(e): return {'linear_rms_m_s':float(np.sqrt(np.mean(np.sum(e[:,:3]**2,axis=1)))),
                        'angular_rms_rad_s':float(np.sqrt(np.mean(np.sum(e[:,3:]**2,axis=1))))}
    result={'samples':len(rows),'duration_s':float(t[-1]),'measurement_frame':rows[0]['frame'],
            'effective_sample_hz':float((len(rows)-1)/t[-1]) if t[-1]>0 else None,
            'all_samples':rms(error),'active_samples':rms(error[active]) if active.any() else None,
            'per_axis_rms':dict(zip(keys,np.sqrt(np.mean(error**2,axis=0)).tolist())),
            'note':'Unshifted tracking error; includes actuator lag and limiting. Measured from J(q)*measured qdot.'}
    out=Path(args.csv).with_suffix(''); out.with_suffix('.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(3,2,figsize=(12,8),sharex=True)
    for i,ax in enumerate(axes.T.flatten()):
        ax.plot(t,command[:,i],label='Command',lw=1.2); ax.plot(t,measured[:,i],label='Measured',lw=.8)
        ax.set_ylabel(keys[i]+(' (m/s)' if i<3 else ' (rad/s)')); ax.grid(alpha=.3)
    axes[0,0].legend(); axes[2,0].set_xlabel('Time (s)'); axes[2,1].set_xlabel('Time (s)')
    fig.suptitle('Fetch twist tracking — '+Path(args.csv).stem); fig.tight_layout()
    fig.savefig(out.with_suffix('.png'),dpi=160); print(json.dumps(result,indent=2))