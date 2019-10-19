class BBCON:


    def __init__(self): #arbit
        self.behaviors = []
        self.active_behaviors = []
        self.sensobs = []
        self.motobs = []
        self.arbitrator = None #arbit
        self.current_timestep = None
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

    def deactivate_behavior(self, behavior):
        """ if behavior is an existing behavior, remove
            from active_behaviors """
        if behavior not in self.behaviors:
            return
        self.active_behaviors.remove(behavior)

    def run_one_timestep(self):
        """ this is the core of the BBCON.
            update all sensobs and behaviors """
        # self.arbit..choose_act... chooses a winning behavior and returns
        # the winning behavors motor recommendations and halt_request
        motor_recs, halt_request = self.arbitrator.choose_action()
        self.motobs.update(motor_recs) # update motobs which then updates all motors
        for sensob in self.sensobs:
            sensob.reset() # may need to reset associated sensors...





