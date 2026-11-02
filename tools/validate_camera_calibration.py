"""Offline known-pose rendered validation. Truth is used here only as a test fixture."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import json
import mujoco
import numpy as np
from src.discovery.scene import load
from src.discovery.association import observe
from src.perception.camera import CAMERA_NAMES,TRANSFORMS,K,FOVY
from src.simulation.robot import advance,yaw
from src.estimation.ekf import wrap

root=Path(__file__).resolve().parents[1]
m,d,layout=load(0);records=[]
with mujoco.Renderer(m,height=480,width=640) as renderer:
    for distance in (.16,.20,.40,.80,1.):
        for offset in (-15.,0.,15.):
            heading=np.arctan2(-layout['normal'][1],-layout['normal'][0])+np.deg2rad(offset)
            d.qpos[:2]=layout['dock']+distance*layout['normal']
            d.qpos[3:7]=[np.cos(heading/2),0,0,np.sin(heading/2)];d.qvel[:]=0
            advance(m,d,.1);mujoco.mj_forward(m,d)
            R=np.empty(9);mujoco.mju_quat2Mat(R,d.qpos[3:7]);R=R.reshape(3,3)
            images=[]
            for name,T in zip(CAMERA_NAMES,TRANSFORMS):
                cid=m.camera(name).id
                np.testing.assert_allclose(d.cam_xpos[cid],d.qpos[:3]+R@T[:3,3],atol=1e-8)
                np.testing.assert_allclose(d.cam_xmat[cid].reshape(3,3)@np.diag([1,-1,-1]),R@T[:3,:3],atol=1e-8)
                renderer.update_scene(d,camera=name);images.append(renderer.render().copy())
            obs,status=observe(*images)
            delta=layout['dock']-d.qpos[:2];r=np.linalg.norm(delta)
            b=wrap(np.arctan2(delta[1],delta[0])-yaw(d))
            row=dict(distance=distance,heading_offset_deg=offset,status=status,detected=obs is not None)
            if obs is not None:
                row.update(range_error_m=float(obs.distance-r),bearing_error_deg=float(np.degrees(wrap(obs.bearing-b))),reprojection_rms_px=obs.residual)
            records.append(row)
accepted=[r for r in records if r['detected']]
mandatory=[r for r in records if r['heading_offset_deg']==0.]
report=dict(detected_count=len(accepted),total_count=len(records),rejected_count=len(records)-len(accepted),scope='Known simulation calibration; rendered development-fixture verification, not hardware calibration',
            samples=records,passed=all(r['detected'] for r in mandatory) and all(abs(r['range_error_m'])<.03 and abs(r['bearing_error_deg'])<2 for r in accepted))
(root/'calibration/monocular_validation.json').write_text(json.dumps(report,indent=2))
(root/'calibration/monocular.json').write_text(json.dumps(dict(K=K.tolist(),fovy_deg=FOVY,marker_size_m=.18,
    T_BC=TRANSFORMS[0].tolist(),source='Exact model settings; not fitted from hardware'),indent=2))
print(json.dumps(report,indent=2));sys.exit(0 if report['passed'] else 1)

