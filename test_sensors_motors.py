from motors import Motors
from camera import Camera
from irproximity_sensor import IRProximitySensor
from reflectance_sensors import ReflectanceSensors
from ultrasonic import Ultrasonic
from robot import Sensob

m = Motors()
cam = Camera()
ir_proxim = IRProximitySensor()
reflect = ReflectanceSensors()
ultra = Ultrasonic()

sensob = Sensob()
sensob.sensors = [cam, ir_proxim, reflect, ultra]

sensob.update()
for i, value in enumerate(sensob.values):
    print(type(sensob.sensors), value)
