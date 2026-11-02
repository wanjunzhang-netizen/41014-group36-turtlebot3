"""Only local measurements/estimates enter this controller; never scene metadata."""
import numpy as np
from src.estimation.ekf import wrap
from src.discovery.docking import dock_target, wheels_in_zone
from src.perception.lidar import points_body
from src.control.avoidance import LocalAvoidance

from src.control.lqr import SegmentTracker

class Navigator:
    def __init__(self,seed):
        self.rng=np.random.default_rng(seed+400)
        self.tracker=SegmentTracker()
        self.planner=LocalAvoidance(seed);self.planner.radius=.22
        self.search_target=None;self.spin_remaining=2*np.pi
        self.state='SEARCH';self.hold=0.;self.target=None;self.done=False
        self.visited=[];self.stall=0.

    # 没找到标记时，观察周围并选择安全探索方向
    def search(self,pose,scan,dt):
        pts=points_body(scan)[:,:2]
        if not len(pts):return 0.,0.,'INVALID SCAN'
        if self.spin_remaining>0:
            self.spin_remaining-=.45*dt
            return 0.,.45,'SEARCH: LOOK AROUND'
        if self.search_target is not None and np.linalg.norm(self.search_target-pose[:2])>.09:
            c,s=np.cos(pose[2]),np.sin(pose[2]);end=(self.search_target-pose[:2])@np.array([[c,-s],[s,c]])
            if self.planner.clearance(pts,end)>.20 and not self.planner.heads_into_visible_wall(scan,end):
                return *self.tracker.command(pose,self.search_target),'SEARCH: RANDOM SAFE MOVE'
        self.search_target=None
        options=[];c,s=np.cos(pose[2]),np.sin(pose[2]);rotation=np.array([[c,-s],[s,c]])
        for angle in np.deg2rad(np.arange(-180,180,15)):
            for length in (.35,.55):
                end=length*np.array([np.cos(angle),np.sin(angle)])
                clear=self.planner.clearance(pts,end)
                if clear<.23 or self.planner.heads_into_visible_wall(scan,end):continue
                target=pose[:2]+rotation@end
                novelty=min([np.linalg.norm(target-p) for p in self.visited[-20:]] or [1.])
                options.append((novelty+.1*clear+.2*self.rng.random(),target))
        if not options:
            return 0.,.35,'SEARCH: ROTATE IN PLACE'
        self.search_target=max(options,key=lambda x:x[0])[1]
        self.visited.append(pose[:2].copy())
        self.spin_remaining=np.pi/2
        return 0.,.45,'SEARCH: LOOK AROUND'

    # 找到标记后，根据估计状态安排接近、对正、停靠
    def command(self,ekf,search_pose,scan,age,dt=.05):
        if self.done:return 0.,0.,'DONE'
        if ekf is None:return self.search(search_pose,scan,dt)
        # ekf，navigation system uses estimated vehicle status and charging station locations
        pose=ekf.x[:3];landmark=ekf.x[3:];normal=ekf.normal
        pts=points_body(scan)[:,:2]
        if not len(pts) or np.mean(~np.isnan(scan.ranges))<.8:return 0.,0.,'INVALID SCAN'
        heading=np.arctan2(-normal[1],-normal[0]);yaw_error=wrap(heading-pose[2])
        if self.state=='FINAL':
            # 停车判定
            goal=dock_target(landmark,normal)
            distance=np.linalg.norm(goal-pose[:2])
            entered=np.any(wheels_in_zone(pose,landmark))
            if distance<.012 and entered and abs(yaw_error)<np.deg2rad(5) and age<.6:
                self.hold+=dt
                if self.hold>=2:self.done=True
                return 0.,0.,'DONE' if self.done else 'FINAL: WHEEL IN DISK, HOLD'
            self.hold=0.
            if age>.6:
                aim=wrap(np.arctan2(*(landmark-pose[:2])[::-1])-pose[2])
                return 0.,float(np.clip(1.5*aim,-.3,.3)),'FINAL: WAIT FOR CAMERA'
            # All LiDAR returns remain obstacles, including the marker board.
            # Rectangle encloses this robot plus a low-speed margin; inspect a 2cm sweep.
            swept=(pts[:,0]>-.13)&(pts[:,0]<.12)&(abs(pts[:,1])<.115)
            if np.any(swept):return 0.,0.,'FINAL: LIDAR FOOTPRINT STOP'
            if distance<.012:
                return 0.,float(np.clip(1.8*yaw_error,-.3,.3)),'FINAL: FACE MARKER'
            v,w=self.tracker.command(pose,goal)
            return min(v,.025),float(np.clip(w,-.3,.3)),'FINAL: CAMERA WHEEL DOCK'
        staging=landmark+.40*normal
        staging_distance=np.linalg.norm(staging-pose[:2])
        # Hysteresis: estimate updates must not toggle opposing approach/align commands.
        if self.state=='ALIGN' and staging_distance>.12:
            self.state='APPROACH'
            self.tracker.target=None
        if staging_distance<.05 or self.state=='ALIGN':
            self.state='ALIGN'
            if age>.6:
                aim=wrap(np.arctan2(*(landmark-pose[:2])[::-1])-pose[2])
                return 0.,float(np.clip(1.5*aim,-.3,.3)),'ALIGN: REACQUIRE CAMERA'
            if abs(yaw_error)<np.deg2rad(6) and age<.6:
                self.state='FINAL';return 0.,0.,'FINAL: ENTER'
            return 0.,float(np.clip(1.8*yaw_error,-.4,.4)),'ALIGN TO MARKER'
        self.state='APPROACH'
        if np.min(np.linalg.norm(pts,axis=1))<.16:return 0.,0.,'CLEARANCE STOP'
        c,s=np.cos(pose[2]),np.sin(pose[2]);rotation=np.array([[c,-s],[s,c]])
        endpoint=(staging-pose[:2])@rotation
        if np.linalg.norm(endpoint)<.75 and self.planner.clearance(pts,endpoint)>.22:
            self.target=staging.copy();status='DIRECT SAFE STAGING'
        else:self.target,status=self.planner.plan(scan,pose,staging)
        if self.target is None:return 0.,.30,'APPROACH: NO SAFE WAYPOINT'
        v,w=self.tracker.command(pose,self.target);v=min(v,.08)
        if np.linalg.norm(landmark-pose[:2])<.7:
            v=min(v,.035)
            if age>8.:
                angle=wrap(np.arctan2(*(landmark-pose[:2])[::-1])-pose[2])
                return 0.,float(np.clip(1.8*angle,-.4,.4)),'APPROACH: REACQUIRE CAMERA'
        if self.planner.clearance(pts,np.array([.16,0]))<.18:v=0.
        return v,w,'APPROACH: '+status
