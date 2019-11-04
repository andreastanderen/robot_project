import random
import sys
import time
import imager2 as IMR
import motors


class BBCON:

    def __init__(self):  # arbit
        self.behaviors = []
        self.active_behaviors = []
        self.sensobs = []
        self.motobs = []
        self.arbitrator = None  # arbit
        self.current_timestep = 0
        self.inactive_behavior = []
        self.currently_controlled = None

    def add_behavior(self, behavior):
        """ adds a new behavior """
        # mulig vi må lage en ny behavior-instans her...
        self.behaviors.append(behavior)

    def add_sensob(self, sensob):
        self.sensobs.append(sensob)

    def activate_behavior(self, behavior):
        """ if behavior is an existing behavior, add to
            active_behaviors """
        if behavior not in self.behaviors:
            return
        self.active_behaviors.append(behavior)
        behavior.active_flag = True

    def deactivate_behavior(self, behavior):
        """ if behavior is an existing behavior, remove
            from active_behaviors """
        if behavior not in self.active_behaviors:
            return
        self.active_behaviors.remove(behavior)
        behavior.active_flag = False

    def run_one_timestep(self):
        """ this is the core of the BBCON.
            update all sensobs and behaviors """
        # self.arbit..choose_act... chooses a winning behavior and returns
        # the winning behavors motor recommendations and halt_request
        print("*" * 50)
        print("Timestep", self.current_timestep)
        print("Before updates:")
        print("All behaviors:", self.behaviors)
        print("Active behaviors:", self.active_behaviors)
        [sensob.update() for sensob in self.sensobs]
        [behavior.update() for behavior in self.behaviors]
        print("\nAfter updates:")
        print("active after updates:", self.active_behaviors)
        motor_recs, halt_request = self.arbitrator.choose_action()
        if motor_recs is None:
            return
        if halt_request:
            self.motobs[0].update([0, 0])
            self.motobs[0].operationalize()
            sys.exit()

        self.motobs[0].update(motor_recs)
        print("motor recs", motor_recs)
        self.motobs[0].operationalize()
        # self.motobs.update(motor_recs)  # update motobs which then updates all motors
        time.sleep(0.5)
        [sensob.reset() for sensob in self.sensobs]
        self.current_timestep += 1
        print("*" * 50, "\n\n")


class Sensob:

    def __init__(self, sensors=None):
        if sensors is None:
            sensors = []
        self.sensors = sensors
        self.values = []

    def update(self):
        if len(self.values) == 0:
            self.values = [None] * len(self.sensors)
        for i, sensor in enumerate(self.sensors):
            sensor.update()
            self.values[i] = sensor.get_value()
            # print(type(sensor), self.values[i])

    def reset(self):
        for sensor in self.sensors:
            sensor.reset()


class Motob:

    def __init__(self):
        self.motors = [motors.Motors()]
        self.value = None

    def update(self, recommendation):
        self.value = recommendation

    def operationalize(self):
        if self.value is None:
            return
        self.motors[0].set_value(self.value)


class Behavior:
    def __init__(self, controller: BBCON, priority):
        self.controller = controller
        self.sensobs = []
        self.motor_recommendations = []
        self.active_flag = False
        self.halt_request = False
        self.priority = priority
        self.match_degree = 0
        self.weight = 0

    def consider_deactivation(self):
        raise NotImplementedError

    def consider_activation(self):
        raise NotImplementedError

    def update(self):
        [sensob.update() for sensob in self.sensobs]
        if self.active_flag:
            self.consider_deactivation()
        else:
            self.consider_activation()
        self.sense_and_act()
        self.weight = self.match_degree * self.priority

    def sense_and_act(self):
        raise NotImplementedError

    def __str__(self):
        return str(type(self))[8:-2]


class FollowLineBehavior(Behavior):

    def __init__(self, controller: BBCON, priority, ir_sensor):
        super().__init__(controller, priority)
        self.activate_value = 0.2
        self.sensobs = [ir_sensor]

    def consider_activation(self):
        if any(value <= self.activate_value for value in self.sensobs[0].values[0]):
            self.controller.activate_behavior(self)

    def consider_deactivation(self):
        if all(value > self.activate_value for value in self.sensobs[0].values[0]):
            self.controller.deactivate_behavior(self)

    def sense_and_act(self):
        values = self.sensobs[0].values[0]
        min_left_value = min(values[0:3])
        min_right_value = min(values[3:])
        left_motor_action = min_left_value
        right_motor_action = min_right_value
        if left_motor_action > right_motor_action:
            self.motor_recommendations = [0.4, 0]
        elif left_motor_action < right_motor_action:
            self.motor_recommendations = [0, 0.4]
        else:
            self.motor_recommendations = [0.3, 0.3]
        print(values)
        self.match_degree = max(1 - left_motor_action, 1 - right_motor_action)


class SearchBehavior(Behavior):
    def __init__(self, controller: BBCON, priority, ir_sensob, ultrasonic):
        super().__init__(controller, priority)
        self.sensobs = [ir_sensob, ultrasonic]
        self.deactivate_ir_value = 0.1
        self.deactivate_ultra = 15

    def consider_activation(self):
        ir_deactivated = all(value > self.deactivate_ir_value for value in self.sensobs[0].values[0])
        # ultra_deactivated = self.sensobs[1].values[0] > self.deactivate_ultra
        if ir_deactivated:
            self.controller.activate_behavior(self)

    def consider_deactivation(self):
        ir_deactivated = any(value < self.deactivate_ir_value for value in self.sensobs[0].values[0])
        ultra_deactivated = self.sensobs[1].values[0] < self.deactivate_ultra
        if ir_deactivated or ultra_deactivated:
            self.controller.deactivate_behavior(self)

    def sense_and_act(self):
        ir_values = self.sensobs[0].values[0]
        ultra_value = self.sensobs[1].values[0]

        ir_deactivated = min(ir_values) > self.deactivate_ir_value
        ultra_deactivated = ultra_value > self.deactivate_ultra
        ir_match = 1 - min(ir_values)
        ultra_match = 1
        if not ir_deactivated:
            self.motor_recommendations = [0, 0]
            self.match_degree = ir_match
        elif not ultra_deactivated:
            forward_speed = 0.3
            self.motor_recommendations = [forward_speed] * 2
            self.match_degree = ultra_match
        else:
            left_motor = random.random() * 0.6 - 0.3
            right_motor = random.random() * 0.6 - 0.3
            self.motor_recommendations = [left_motor, right_motor]
            self.match_degree = 1 - ir_match * ultra_match


class TakePictureBehavior(Behavior):
    def __init__(self, controller: BBCON, priority, camera, ultrasonic):
        super().__init__(controller, priority)
        self.sensobs = [camera, ultrasonic]
        self.activate_value = 15

    def consider_activation(self):
        value = self.sensobs[1].values[0]
        if value <= self.activate_value:
            self.controller.activate_behavior(self)

    def consider_deactivation(self):
        value = self.sensobs[1].values[0]
        if value > self.activate_value:
            self.controller.deactivate_behavior(self)

    def sense_and_act(self):
        value = self.sensobs[1].values[0]
        if value > 5:
            self.motor_recommendations = [0.3, 0.3]
            self.match_degree = 1
        elif value <= 5:
            print("Take picture")
            img = IMR.Imager(image=self.sensobs[0].values[0]).scale(1, 1)
            filename = "images/" + str(time.asctime()) + '.jpeg'
            pixel = img.get_pixel(img.image.size[0] // 2, img.image.size[1] // 2)
            print(pixel)
            self.match_degree = 1
            if sum(pixel) > 600:
                self.motor_recommendations = [0, 0]

                img.dump_image(filename)

                self.halt_request = True
            else:
                self.motor_recommendations = [-0.3, 0.3]


class Arbitrator:

    def __init__(self, controller: BBCON):
        self.controller = controller

    def choose_action(self):
        if len(self.controller.active_behaviors) == 0:
            return None, None
        action = max(self.controller.active_behaviors, key=lambda b: b.weight)
        print("\n", action, "was chosen", "\n")
        return action.motor_recommendations, action.halt_request


if __name__ == '__main__':
    CONTROLLER = BBCON()
    ARBITRATOR = Arbitrator(CONTROLLER)
    CONTROLLER.arbitrator = ARBITRATOR
