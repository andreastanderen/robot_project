import robot
from camera import Camera
from ultrasonic import Ultrasonic
from reflectance_sensors import ReflectanceSensors
from zumo_button import ZumoButton


class PiZumoRobot:

    def __init__(self):
        self.controller = robot.BBCON()
        self.controller.arbitrator = robot.Arbitrator(self.controller)

        self.controller.motobs = [robot.Motob()]

        reflect_sensob = robot.Sensob([ReflectanceSensors()])
        ultra_sensob = robot.Sensob([Ultrasonic()])
        cam_sensob = robot.Sensob([Camera()])
        self.controller.sensobs = [reflect_sensob, ultra_sensob, cam_sensob]



    def run(self):
        # button = ZumoButton()
        # button.wait_for_press()
        while True:
            self.controller.run_one_timestep()

if __name__ == '__main__':
    zumo = PiZumoRobot()
    zumo.run()
