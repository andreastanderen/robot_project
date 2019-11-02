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
        [sensob.update() for sensob in self.sensobs]
        [behavior.update() for behavior in self.behaviors]
        motor_recs, halt_request = self.arbitrator.choose_action()
        if halt_request:
            sys.exit()
        for i, rec in enumerate(motor_recs):
            self.motobs[i].update(rec)

        # self.motobs.update(motor_recs)  # update motobs which then updates all motors
        for sensob in self.sensobs:
            sensob.reset()  # may need to reset associated sensors...
        time.sleep(0.5)
        [sensob.reset() for sensob in self.sensobs]
        self.current_timestep += 1


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


class Motob:

    def __init__(self):
        self.motors = [motors.Motors()]
        self.value = None

    def update(self, recommendation):
        self.value = recommendation

    def operationalize(self):
        pass


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


class FollowLineBehavior(Behavior):

    def __init__(self, controller: BBCON, priority, ir_sensor):
        super().__init__(controller, priority)
        self.activate_value = 0.5
        self.sensobs = [ir_sensor]

    def consider_activation(self):
        if any(value > self.activate_value for value in self.sensobs[0].values):
            self.controller.activate_behavior(self)

    def consider_deactivation(self):
        if all(value > 1 - self.activate_value for value in self.sensobs[0].values):
            self.controller.deactivate_behavior(self)

    def sense_and_act(self):
        values = self.sensobs[0].values
        min_left_value = min(values[0:3])
        min_right_value = min(values[0:3])
        left_motor_action = min_left_value
        right_motor_action = min_right_value
        if left_motor_action != right_motor_action:
            self.motor_recommendations = [left_motor_action, right_motor_action]
        else:
            max_action = max(left_motor_action, right_motor_action)
            self.motor_recommendations = [max_action, max_action]
        self.match_degree = max(1 - left_motor_action, 1 - right_motor_action)


class SearchBehavior(Behavior):
    def __init__(self, controller: BBCON, priority, ir_sensob, ultrasonic):
        super().__init__(controller, priority)
        self.sensobs = [ir_sensob, ultrasonic]
        self.deactivate_ir_value = 0.5
        self.deactivate_ultra = 15

    def consider_activation(self):
        ir_deactivated = all(value > self.deactivate_ir_value for value in self.sensobs[0].values)
        ultra_deactivated = self.sensobs[1].values[0] > self.deactivate_ultra
        if ir_deactivated and ultra_deactivated:
            self.controller.activate_behavior(self)

    def consider_deactivation(self):
        ir_deactivated = all(value > self.deactivate_ir_value for value in self.sensobs[0].get_value())
        ultra_deactivated = self.sensobs[1].values[0] > self.deactivate_ultra
        if not (ir_deactivated and ultra_deactivated):
            self.controller.deactivate_behavior(self)

    def sense_and_act(self):
        ir_values = self.sensobs[0].values
        ultra_value = self.sensobs[1].values[0]

        ir_deactivated = min(ir_values) > self.deactivate_ir_value
        ultra_deactivated = ultra_value > self.deactivate_ultra
        ir_match = 1 - min(ir_values)
        ultra_match = 1 - min(ultra_value, self.deactivate_ultra) / (self.deactivate_ultra + 1)
        if not ir_deactivated:
            self.motor_recommendations = [0, 0]
            self.match_degree = ir_match
        elif not ultra_deactivated:
            forward_speed = 0.3
            self.motor_recommendations = [forward_speed] * 2
            self.match_degree = ultra_match
        else:
            left_motor = random.random() * 2 - 1
            right_motor = random.random() * 2 - 1
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
            self.motor_recommendations = [0.5, 0.5]
            self.match_degree = 1
        elif value <= 5:
            self.motor_recommendations = [0, 0]
            self.match_degree = 1
            img = IMR.Imager(image=self.sensobs[0].values)
            img.dump_image(str(time.asctime()) + '.jpeg')


class Arbitrator:

    def __init__(self, controller: BBCON):
        self.controller = controller

    def choose_action(self):
        action = max(self.controller.active_behaviors, key=lambda b: b.weight)
        print("Chose", type(action))
        return action.motor_recommendations, action.halt_request


if __name__ == '__main__':
    CONTROLLER = BBCON()
    ARBITRATOR = Arbitrator(CONTROLLER)
    CONTROLLER.arbitrator = ARBITRATOR
