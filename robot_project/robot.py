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

