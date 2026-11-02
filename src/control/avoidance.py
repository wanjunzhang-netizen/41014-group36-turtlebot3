"""LiDAR waypoint avoidance uses scans, estimated pose and targets, not scene geometry."""
import numpy as np
from src.perception.lidar import points_body
from src.estimation.ekf import wrap


class LocalAvoidance:
    def __init__(self,seed=7):
        self.target=None
        self.side=1 if np.random.default_rng(seed).random()>.5 else -1
        self.radius=.24  # Includes body size, range error and a low-speed reaction margin.

    # Check clearance between a candidate path and obstacle points.
    def clearance(self,points,endpoint):
        length=np.linalg.norm(endpoint)
        if length<1e-6:
            return float('inf')
        unit=endpoint/length
        along=np.clip(points@unit,0,length)
        return float(np.min(np.linalg.norm(points-along[:,None]*unit,axis=1))) if len(points) else float('inf')

    def heads_into_visible_wall(self,scan,endpoint):
        """Reject exploratory headings toward a nearby, directly visible long wall.

        Uses only a contiguous angular scan sector, not world geometry. A nearer
        object/gap breaks the straight-wall evidence. Parallel travel is allowed.
        """
        length=np.linalg.norm(endpoint)
        if length<1e-6:return False
        direction=endpoint/length
        angle=np.arctan2(direction[1],direction[0])
        offsets=wrap(scan.angles-angle)
        indices=np.where(abs(offsets)<np.deg2rad(25))[0]
        indices=indices[np.argsort(offsets[indices])]
        ranges=scan.ranges[indices]
        finite=np.isfinite(ranges)
        if len(indices)<12 or np.mean(finite)<.9:return False
        angles=scan.angles[indices[finite]];ranges=ranges[finite]
        points=np.c_[ranges*np.cos(angles),ranges*np.sin(angles)]
        from src.perception.lidar import T_BL_TRANSLATION
        points+=T_BL_TRANSLATION[:2]
        if np.max(np.linalg.norm(np.diff(points,axis=0),axis=1))>.18:return False
        center=points.mean(axis=0)
        _,_,vh=np.linalg.svd(points-center,full_matrices=False)
        tangent,normal=vh[0],vh[1]
        residual=abs((points-center)@normal)
        if np.ptp(points@tangent)<.65 or np.sqrt(np.mean(residual**2))>.025 or residual.max()>.065:return False
        if normal@center<0:normal=-normal
        distance=float(normal@center)
        return bool(.15<distance<1.20 and normal@direction>.70)

    # Choose a safe nearby waypoint.
    def plan(self,scan,pose,goal):
        pts=points_body(scan)[:,:2]
        if len(pts)==0 or np.mean(~np.isnan(scan.ranges))<.8:
            return None,'INVALID SCAN'
        if np.min(np.linalg.norm(pts,axis=1))<.17:
            return None,'CLEARANCE STOP'
        c,s=np.cos(pose[2]),np.sin(pose[2])
        R=np.array([[c,-s],[s,c]])
        delta=np.asarray(goal)-pose[:2]
        bearing=wrap(np.arctan2(delta[1],delta[0])-pose[2])
        # Keep a safe waypoint to reduce left-right switching.
        if self.target is not None:
            end=(self.target-pose[:2])@R
            if np.linalg.norm(end)>.12 and self.clearance(pts,end)>self.radius:
                return self.target.copy(),'FOLLOW LOCAL WAYPOINT'
        best=None
        for length in (.30,.45,.70,1.0):
            for angle in np.deg2rad(np.arange(-100,101,5)):
                # Allow sideways detours and up to 10 degrees of temporary motion away from the goal;
                # turn in place first, then drive forward; never reverse the body.
                if abs(wrap(angle-bearing))>np.deg2rad(100):
                    continue
                end=length*np.array([np.cos(angle),np.sin(angle)])
                clear=self.clearance(pts,end)
                if clear<self.radius:
                    continue
                # Require enough valid readings around the candidate direction.
                sector=np.abs(wrap(scan.angles-angle))<np.deg2rad(10)
                if np.mean(~np.isnan(scan.ranges[sector]))<.8:
                    continue
                world=pose[:2]+R@end
                score=np.linalg.norm(np.asarray(goal)-world)+.12*abs(angle)-.04*min(clear,.8)
                score-=.015*self.side*np.sin(angle)  # Repeatable preference when safe candidates have similar scores.
                if best is None or score<best[0]:
                    best=(score,world)
        if best is None:
            self.target=None
            return None,'NO SAFE DIRECTION'
        self.target=best[1]
        return self.target.copy(),'NEW SAFE WAYPOINT'

