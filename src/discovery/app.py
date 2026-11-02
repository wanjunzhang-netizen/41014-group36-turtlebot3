"""3x3m random-room monocular landmark discovery; LiDAR avoidance. Truth is evaluation only."""
import argparse
import csv
import json
from pathlib import Path
import secrets
import time
import cv2
import mujoco
import numpy as np
from src.discovery.scene import load
from src.discovery.docking import zone_center, dock_target, wheels_in_zone, ZONE_RADIUS
from src.discovery.association import observe
from src.discovery.estimator import LocalEKF
from src.discovery.navigation import Navigator
from src.perception.lidar import SimLidar,points_body
from src.perception.encoders import WheelEncoders
from src.perception.camera import WIDTH,HEIGHT
from src.simulation.robot import drive,yaw
from src.estimation.ekf import motion,control_increment,wrap

ROOT=Path(__file__).resolve().parents[2]/'outputs/monocular'

class Trial:
    def __init__(self,seed,limit,lidar_sigma=.006,dropout_duration=0.):
        self.seed=seed;self.limit=limit;self.lidar_sigma=lidar_sigma;self.dropout_duration=dropout_duration
        self.model,self.data,self.evaluation_layout=load(seed)
        self.t0=float(self.data.time)
        self.encoders=WheelEncoders(self.model,self.data,seed,scale_error=(.001,-.001))
        self.lidar=SimLidar(self.model,sigma=lidar_sigma,dropout=.01,seed=seed+100)
        self.navigator=Navigator(seed);self.scan=self.lidar.read(self.data)
        self.filter=None;self.search_pose=np.zeros(3);self.steps=0
        self.v=self.w=0.;self.last_update=-np.inf;self.anchored_at=None
        self.frame=np.zeros((HEIGHT,WIDTH,3),np.uint8);self.rng=np.random.default_rng(seed+1000)
        self.status='SEARCH';self.sensor_status='NOT SEEN';self.rows=[];self.collision=False
        self.evaluation_origin=None;self.initial_landmark=None;self.last_obs=None;self.pending=None

    def step(self,renderer):
        drive(self.data,self.v,self.w)
        for _ in range(25):
            mujoco.mj_step(self.model,self.data)
            for c in self.data.contact[:self.data.ncon]:
                a,b=int(c.geom1),int(c.geom2)
                if self.lidar.robot_geoms[a]!=self.lidar.robot_geoms[b]:
                    other=b if self.lidar.robot_geoms[a] else a
                    if self.model.geom(other).name!='floor':
                        if not self.collision:print(f'COLLISION: {self.model.geom(other).name} t={self.data.time-self.t0:.2f}',flush=True)
                        self.collision=True
        self.steps+=1;t=float(self.data.time-self.t0)
        reading=self.encoders.read(self.data);(ds,da),_=control_increment(reading)
        self.search_pose=motion(self.search_pose,ds,da)
        if self.filter is not None:self.filter.predict(reading)
        if self.steps%2==0:self.scan=self.lidar.read(self.data)
        if self.steps%4==0:
            renderer.update_scene(self.data,camera='front_camera');rgb=renderer.render()
            rgb=np.clip(rgb.astype(float)+self.rng.normal(0,1.5,rgb.shape),0,255).astype(np.uint8)
            obs,self.sensor_status=observe(rgb)
            if self.anchored_at is not None and 5 <= t-self.anchored_at < 5+self.dropout_duration:
                obs=None;self.sensor_status='DECLARED CAMERA DROPOUT'
                rgb[:]=25
            self.last_obs=obs
            if obs is not None:
                if self.filter is None:
                    # First fully associated measurement defines origin. No map coordinates supplied.
                    self.filter=LocalEKF(obs);self.anchored_at=t;self.initial_landmark=obs.point.copy()
                    self.last_update=t
                    # World pose is saved ONLY to express evaluation truth in the new frame.
                    self.evaluation_origin=np.array([*self.data.qpos[:2],yaw(self.data)])
                    print(f'[{t:.1f}s] ANCHORED local robot=(0,0,0), station={obs.point.round(3)}',flush=True)
                elif self.filter.update(obs):self.last_update=t
            if obs is not None:
                cv2.polylines(rgb,[obs.corners.astype(np.int32)],True,(0,255,0),2)
                center=tuple(obs.corners.mean(axis=0).astype(int))
                cv2.circle(rgb,center,5,(255,180,0),-1)
            for im,label in ((rgb,'SINGLE RGB CAMERA'),):
                cv2.rectangle(im,(0,0),(WIDTH,58),(25,25,25),-1)
                cv2.putText(im,label,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(240,240,240),1)
                cv2.putText(im,self.sensor_status,(8,46),cv2.FONT_HERSHEY_SIMPLEX,.4,(240,240,240),1)
            self.frame=rgb
        self.v,self.w,self.status=self.navigator.command(self.filter,self.search_pose,self.scan,t-self.last_update)
        if t>=self.limit:self.v=self.w=0.;self.status='TIMEOUT'
        row={'time':t,'state':self.status,'v':self.v,'omega':self.w,'anchored':int(self.filter is not None)}
        if self.filter is not None:
            p=self.evaluation_origin;c,s=np.cos(p[2]),np.sin(p[2]);rotation=np.array([[c,-s],[s,c]])
            truth=np.r_[(self.data.qpos[:2]-p[:2])@rotation,wrap(yaw(self.data)-p[2])]
            for key,value in zip(('x','y','yaw','landmark_x','landmark_y'),self.filter.x):row[key]=float(value)
            for key,value in zip(('true_x','true_y','true_yaw'),truth):row[key]=float(value)
            for i in range(5):
                for j in range(5):row[f'P{i}{j}']=float(self.filter.P[i,j])
            row['landmark_z']=self.filter.height
            row['position_error_m']=float(np.linalg.norm(self.filter.x[:2]-truth[:2]))
            row['vision_updates']=self.filter.accepted
            row['vision_age_s']=float(t-self.last_update)
            row['navigation_phase']=self.navigator.state
            row['staging_distance_m']=float(np.linalg.norm(self.filter.x[3:]+.40*self.filter.normal-self.filter.x[:2]))
            row['estimated_wheel_zone']=int(np.any(wheels_in_zone(self.filter.x[:3],self.filter.x[3:])))
            row['camera_residual_px']=float(self.last_obs.residual) if self.last_obs is not None else float('nan')
        self.rows.append(row)
        return self.navigator.done or t>=self.limit

    def wheel_contacts(self):
        # EVALUATION ONLY: real floor contact points, never fed to the controller.
        points=[]
        for contact in self.data.contact[:self.data.ncon]:
            g1,g2=int(contact.geom1),int(contact.geom2)
            for wheel,floor in ((g1,g2),(g2,g1)):
                name=self.model.body(int(self.model.geom_bodyid[wheel])).name
                if name in ('wheel_left_link','wheel_right_link') and self.model.geom(floor).name=='floor':
                    points.append(contact.pos[:2].copy())
        return points

    def save(self):
        ROOT.mkdir(parents=True,exist_ok=True);stem=ROOT/(f'room_seed{self.seed}_noise{self.lidar_sigma:g}_drop{self.dropout_duration:g}'.replace('.', 'p'))
        keys=list(dict.fromkeys(k for row in self.rows for k in row))
        with stem.with_suffix('.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(self.rows)
        layout=self.evaluation_layout
        # Dock error relative to actual board-normal reference, evaluation only.
        goal=dock_target(layout['dock'],layout['normal'])
        contacts=self.wheel_contacts()
        contact_distances=[float(np.linalg.norm(p-layout['dock'])) for p in contacts]
        entered=bool(contact_distances and min(contact_distances)<=ZONE_RADIUS)
        error=float(np.linalg.norm(self.data.qpos[:2]-goal))
        heading=np.arctan2(-layout['normal'][1],-layout['normal'][0])
        angle=abs(np.degrees(wrap(yaw(self.data)-heading)))
        errors=[r['position_error_m'] for r in self.rows if 'position_error_m' in r]
        result=dict(charging_zone_entered=entered,wheel_contact_distances_m=contact_distances,zone_center_distance_m=float(np.linalg.norm(self.data.qpos[:2]-layout['dock'])),zone_radius_m=ZONE_RADIUS,seed=self.seed,lidar_sigma=self.lidar_sigma,camera_dropout_duration=self.dropout_duration,completed=self.navigator.done,collision=self.collision,duration_s=self.rows[-1]['time'],
                    anchor_time_s=self.anchored_at,final_dock_error_m=error,final_heading_error_deg=float(angle),
                    initial_landmark_local_xyz=None if self.initial_landmark is None else self.initial_landmark.tolist(),
                    position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                    updates=0 if self.filter is None else self.filter.accepted,
                    passed=bool(self.navigator.done and entered and not self.collision and error<.04 and angle<8))
        stem.with_suffix('.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        def serial(v):return v.tolist() if isinstance(v,np.ndarray) else v
        (ROOT/f'room_seed{self.seed}_scene.json').write_text(json.dumps(layout,default=serial,indent=2),encoding='utf-8')
        print(json.dumps(result,indent=2),flush=True);return result,stem

class Plot:
    def __init__(self):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        self.plt=plt;self.fig,self.axes=plt.subplots(2,2,figsize=(11,8));a=self.axes.ravel()
        self.image=a[0].imshow(np.zeros((HEIGHT,WIDTH,3),np.uint8));a[0].axis('off');a[0].set_title('Single camera: marker pixels')
        self.points=a[1].scatter([],[],s=4);a[1].scatter([0],[0],marker='^',c='k')
        a[1].set(xlim=(-2,2),ylim=(-1,3),aspect='equal',title='Obstacles around the robot',xlabel='Right of robot (m)',ylabel='Ahead of robot (m)')
        self.truth,=a[2].plot([],[],'k',label='Actual path (evaluation only)');self.estimate,=a[2].plot([],[],'b',label='Estimated path (EKF)')
        self.zone=Circle((0,0),ZONE_RADIUS,color='purple',alpha=.18,label='Estimated charging zone');self.zone.set_visible(False);a[2].add_patch(self.zone)
        self.station=a[2].scatter([],[],marker='s',c='purple',label='Estimated station')
        a[2].set(xlim=(-3,3),ylim=(-3,3),aspect='equal',title='Robot path and charging station',xlabel='X position (m)',ylabel='Y position (m)');a[2].legend(fontsize=8)
        self.error,=a[3].plot([],[],'b',label='Position error')
        self.sigma,=a[3].plot([],[],'m--',label='2 sigma (from P)')
        a[3].set(title='Position error and estimated uncertainty',xlabel='Time (s)',ylabel='Distance (m)');a[3].legend(fontsize=8)
        for ax in a[1:]:ax.grid(True)
        self.paused=False
        def key(event):
            if event.key in (' ','p'):self.paused=not self.paused
        self.fig.canvas.mpl_connect('key_press_event',key);self.fig.tight_layout(rect=(0,0,1,.95))

    def update(self,trial):
        self.image.set_data(trial.frame);pts=points_body(trial.scan);self.points.set_offsets(np.c_[-pts[:,1],pts[:,0]])
        rows=[r for r in trial.rows if r['anchored']]
        if rows:
            self.truth.set_data([r['true_x'] for r in rows],[r['true_y'] for r in rows])
            self.estimate.set_data([r['x'] for r in rows],[r['y'] for r in rows])
            self.station.set_offsets([trial.filter.x[3:]])
            self.zone.center=zone_center(trial.filter.x[3:],trial.filter.normal);self.zone.set_visible(True)
            t=[r['time'] for r in rows];self.error.set_data(t,[r['position_error_m'] for r in rows])
            
            # 2 sigma uncertainty from EKF covariance P, evaluation only
            uncertainty=[2*np.sqrt(max(0.,r['P00']+r['P11'])) for r in rows]
            self.sigma.set_data(t,uncertainty)
            self.axes[1,1].relim();self.axes[1,1].autoscale_view();self.axes[1,1].set_ylim(bottom=0)
        if trial.status == 'DONE':stage='Docked'
        elif trial.status == 'TIMEOUT':stage='Time limit reached'
        elif trial.status == 'COLLISION STOP':stage='Stopped: collision'
        elif 'INVALID SCAN' in trial.status or 'STOP' in trial.status:stage='Stopped for safety'
        elif 'WAIT' in trial.status or 'REACQUIRE' in trial.status:stage='Finding marker again'
        elif trial.status.startswith('SEARCH'):stage='Searching'
        elif trial.status.startswith('ALIGN'):stage='Aligning'
        elif trial.status.startswith('FINAL'):stage='Docking'
        else:stage='Approaching'
        self.fig.suptitle(f'Time: {trial.rows[-1]["time"]:.1f} s  |  {stage}')
        self.fig.canvas.draw_idle()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seed',type=int);p.add_argument('--headless',action='store_true');p.add_argument('--limit',type=float,default=360.)
    p.add_argument('--lidar-sigma',type=float,default=.006);p.add_argument('--camera-dropout',type=float,default=0.)
    args=p.parse_args();seed=secrets.randbelow(1000000) if args.seed is None else args.seed
    print(f'Random room seed: {seed}',flush=True)
    if args.headless:
        import matplotlib;matplotlib.use('Agg')
    if args.lidar_sigma<0 or args.camera_dropout<0 or args.limit<=0:p.error('Noise/dropout must be nonnegative and limit positive')
    trial=Trial(seed,args.limit,args.lidar_sigma,args.camera_dropout);plot=Plot()
    with mujoco.Renderer(trial.model,height=HEIGHT,width=WIDTH) as renderer:
        if args.headless:
            while not trial.step(renderer):pass
        else:
            from mujoco import viewer as mj_viewer
            plot.plt.show(block=False)
            with mj_viewer.launch_passive(trial.model,trial.data) as viewer:
                viewer.cam.lookat[:]=[0,0,0];viewer.cam.distance=5;viewer.cam.azimuth=90;viewer.cam.elevation=-65
                done=False
                while viewer.is_running() and plot.plt.fignum_exists(plot.fig.number):
                    tick=time.perf_counter()
                    if not done and not plot.paused:
                        done=trial.step(renderer)
                        if trial.steps%4==0 or done:plot.update(trial)
                    viewer.sync();plot.plt.pause(.001);time.sleep(max(0,.05-(time.perf_counter()-tick)))
    result,stem=trial.save();plot.update(trial);plot.fig.savefig(stem.with_suffix('.png'),dpi=120);plot.plt.close(plot.fig)
    if args.headless and not result['passed']:raise SystemExit(1)

if __name__=='__main__':main()

