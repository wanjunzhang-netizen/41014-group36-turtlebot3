"""Random world generation only. Sampled world poses must not enter navigation."""
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from src.simulation import robot
from src.simulation.scene import add_station
from collections import deque
from src.discovery.docking import zone_center, ZONE_RADIUS

def sample_layout(seed):
    rng=np.random.default_rng(seed)
    for attempt in range(1000):
        # charging station
        dock=rng.uniform(-1.05,1.05,2);yaw=rng.uniform(-np.pi,np.pi)
        normal=np.array([np.cos(yaw),np.sin(yaw)])
        approach=dock+.50*normal
        if np.max(np.abs(approach))>1.13:continue
        # robot start position and heading
        start=rng.uniform(-1.15,1.15,2);heading=rng.uniform(-np.pi,np.pi)
        if np.linalg.norm(start-dock)<.65:continue
        obstacles=[]
        for _ in range(300):
            # Obstacle positions and sizes.
            point=rng.uniform(-1.15,1.15,2);radius=rng.uniform(.09,.13)
            if np.linalg.norm(point-start)<radius+.32:continue
            if np.linalg.norm(point-dock)<radius+.36 or np.linalg.norm(point-approach)<radius+.33:continue
            if any(np.linalg.norm(point-p)<radius+r+.18 for p,r,_ in obstacles):continue
            # Randomly select a cylinder or a cube
            obstacles.append((point,radius,bool(rng.integers(2))))
            # Number of obstacles.
            if len(obstacles)==6:break
        if len(obstacles)<6:continue
        # Fixed demo: move one of the six obstacles onto the start-to-station line.
        # This is generation-only information, never supplied to navigation.
        if seed == 0:
            point=start+.30*(dock-start)
            radius=.12
            if any(np.linalg.norm(point-p)<radius+r+.18 for p,r,_ in obstacles[1:]):continue
            if np.linalg.norm(point-start)<radius+.32:continue
            if np.linalg.norm(point-dock)<radius+.36 or np.linalg.norm(point-approach)<radius+.33:continue
            obstacles[0]=(point,radius,False) # fixed orange box; keep six total
        # Reject disconnected layouts, using generation-only geometry.
        grid=np.arange(-1.25,1.251,.05);X,Y=np.meshgrid(grid,grid,indexing='ij')
        free=np.ones(X.shape,dtype=bool)
        for p,r,_ in obstacles:free &= (X-p[0])**2+(Y-p[1])**2>(r+.24)**2
        free &= (X-dock[0])**2+(Y-dock[1])**2>.34**2
        def idx(p):return tuple(np.argmin(abs(grid-c)) for c in p)
        a,b=idx(start),idx(approach)
        if not free[a] or not free[b]:continue
        seen={a};todo=deque([a])
        while todo:
            i,j=todo.popleft()
            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                q=(i+di,j+dj)
                if 0<=q[0]<len(grid) and 0<=q[1]<len(grid) and free[q] and q not in seen:
                    seen.add(q);todo.append(q)
        if b in seen:return dict(seed=int(seed),attempt=attempt,start=start,heading=heading,dock=dock,normal=normal,obstacles=obstacles)
    raise RuntimeError('No valid connected random room')

def load(seed):
    layout=sample_layout(seed)
    def configure(scene):
        world=scene.find('worldbody')
        for child in list(world):
            if child.tag in ('geom','site'):world.remove(child)
        # Add walls and floor
        ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 .1',rgba='.24 .27 .3 1')
        for name,pos,size in [('north','0 1.55 .2','1.6 .05 .2'),('south','0 -1.55 .2','1.6 .05 .2'),
                              ('east','1.55 0 .2','.05 1.5 .2'),('west','-1.55 0 .2','.05 1.5 .2')]:
            ET.SubElement(world,'geom',name=name,type='box',pos=pos,size=size,rgba='.7 .73 .76 1')
        for i,(p,r,cylinder) in enumerate(layout['obstacles']):
            # radius bounds the shape, including square corners.
            size=f'{r} .16' if cylinder else f'{r/np.sqrt(2)} {r/np.sqrt(2)} .16'
            ET.SubElement(world,'geom',name=f'obstacle_{i}',type='cylinder' if cylinder else 'box',
                          pos=f'{p[0]} {p[1]} .16',size=size,rgba='1 .65 .05 1' if cylinder else '1 .3 .08 1')
        body=world.find("body[@name='robot']");heading=layout['heading']
        body.set('pos',f"{layout['start'][0]} {layout['start'][1]} .002")
        body.set('quat',f'{np.cos(heading/2)} 0 0 {np.sin(heading/2)}')
        add_station(scene)
        station=world.find("body[@name='charging_station']")
        n=layout['normal'];rotation=np.column_stack(([-n[1],n[0],0],[0,0,1],[*n,0]))
        quat=np.empty(4);mujoco.mju_mat2Quat(quat,rotation.ravel())
        station.set('pos',f"{layout['dock'][0]} {layout['dock'][1]} .16")
        station.set('quat',' '.join(map(str,quat)))
        support=world.find("geom[@name='dock_support']")
        center=zone_center(layout["dock"],layout["normal"])
        support.set("type","cylinder")
        support.set("pos",f"{center[0]} {center[1]} .001")
        support.set('size',f'{ZONE_RADIUS} .001')
        support.set('contype','0');support.set('conaffinity','0')
        support.set('rgba','.75 .05 .8 1')
    model,data=robot.load_scene(configure)
    robot.advance(model,data,.5)
    return model,data,layout
