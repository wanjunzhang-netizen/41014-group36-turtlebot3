"""Offline simulated wall experiment. Known distances are used only in calibration."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import json
from pathlib import Path
import mujoco
import numpy as np
from src.perception.lidar import SimLidar

ROOT=Path(__file__).resolve().parents[1]

def sample(distances,seed):
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom name="wall" type="box" pos="6.05 0 0" size="0.05 10 10"/>
      <body name="robot"><site name="lidar_frame" pos="0 0 0"/></body>
      </worldbody></mujoco>''')
    data=mujoco.MjData(model)
    sensor=SimLidar(model,seed=seed,dropout=0,calibrated=False)
    records=[]
    for distance in distances:
        # Front face is x=6m. Known fixture distance, not online robot truth.
        model.body_pos[model.body('robot').id,0]=6.-distance
        sensor.ray_model.body_pos[:]=model.body_pos
        mujoco.mj_forward(model,data)
        for _ in range(100):
            raw=sensor.read(data).ranges[180]
            records.append([distance,float(raw)])
    return np.asarray(records)

def main():
    train=sample([.4,.8,1.5,2.5,4.,5.],2026)
    test=sample([.6,1.2,2.,3.,4.5],2027)
    scale,offset=np.linalg.lstsq(np.column_stack((train[:,0],np.ones(len(train)))),train[:,1],rcond=None)[0]
    corrected=(test[:,1]-offset)/scale
    report={'method':'raw = scale * known_distance + offset; corrected = (raw-offset)/scale',
            'scope':'MuJoCo simulated LiDAR; not physical hardware calibration',
            'scale':float(scale),'offset_m':float(offset),'train_samples':len(train),'holdout_samples':len(test),
            'holdout_raw_rmse_m':float(np.sqrt(np.mean((test[:,1]-test[:,0])**2))),
            'holdout_corrected_rmse_m':float(np.sqrt(np.mean((corrected-test[:,0])**2))),
            'holdout_corrected_mean_error_m':float(np.mean(corrected-test[:,0]))}
    assert .95<scale<1.05 and abs(offset)<.03
    assert report['holdout_corrected_rmse_m']<.02
    (ROOT/'calibration/lidar.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    np.savetxt(ROOT/'calibration/lidar_train.csv',train,delimiter=',',header='known_distance_m,raw_distance_m',comments='')
    np.savetxt(ROOT/'calibration/lidar_holdout.csv',test,delimiter=',',header='known_distance_m,raw_distance_m',comments='')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
