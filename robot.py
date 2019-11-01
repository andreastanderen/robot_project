import sys
import time

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

    def __init__(self, sensors=[]):
        self.sensors = sensors
        self.values = []

    def update(self):
        for i, sensor in enumerate(self.sensors):
            self.values[i] = sensor.get.value()


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
        if self.active_flag:
            self.consider_deactivation()
        else:
            self.consider_activation()
        self.sense_and_act()

    def sense_and_act(self):
        pass


class FollowLineBehavior(Behavior):

    def __init__(self, controller: BBCON, priority):
        super().__init__(controller, priority)
        self.activate_value = 0.5

    def consider_activation(self):
        if any(value > self.activate_value for value in self.sensobs[0].values):
            self.controller.activate_behavior(self)

    def consider_deactivation(self):
        if all(value > 1 - self.activate_value for value in self.sensobs[0].values):
            self.controller.deactivate_behavior(self)

    def sense_and_act(self):
        self.sensobs[0].update()
        values = self.sensobs[0].get_values()
        left_motor_action = 1 - min(values[0:3])
        right_motor_action = 1 - min(values[3:])
        self.motor_recommendations = [left_motor_action, right_motor_action]


class Arbitrator:

    def __init__(self, controller: BBCON):
        self.controller = controller

    def choose_action(self):
        action = max(self.controller.active_behaviors, key=lambda b: b.weight)
        return action.motor_recommendations, action.halt_request


if __name__ == '__main__':
    CONTROLLER = BBCON()
    ARBITRATOR = Arbitrator(CONTROLLER)
    CONTROLLER.arbitrator = ARBITRATOR
