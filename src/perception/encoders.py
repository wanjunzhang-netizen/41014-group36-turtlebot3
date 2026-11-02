"""轮编码器仿真：只读轮关节传感器，不读取车体真实位姿。"""
import numpy as np
from src.estimation.ekf import EncoderIncrement


class WheelEncoders:
    def __init__(self,model,data,seed=7,scale_error=(.01,-.01)):
        self.addresses = [int(model.sensor_adr[model.sensor(name).id])
                          for name in ("left_encoder","right_encoder")]
        self.previous = data.sensordata[self.addresses].copy()
        self.time = float(data.time)
        self.rng = np.random.default_rng(seed)
        self.scale = 1.+np.asarray(scale_error,dtype=float)
        if self.scale.shape!=(2,) or not np.isfinite(self.scale).all() or np.any(self.scale<=0):
            raise ValueError('scale_error must contain two valid relative wheel scale errors')

    def read(self,data):
        angles = data.sensordata[self.addresses].copy()
        dt = float(data.time)-self.time
        if dt <= 0:
            raise ValueError("Encoder timestamps must increase")
        sigma = .0015*np.sqrt(dt/.05)
        delta = (angles-self.previous)*self.scale + self.rng.normal(0,sigma,2)
        self.previous,self.time = angles,float(data.time)
        return EncoderIncrement(self.time,dt,delta,np.full(2,sigma**2))
