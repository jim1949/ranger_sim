from morse.builder.creator import ActuatorCreator

class Teleport2(ActuatorCreator):
    _classpath = "ranger_sim.actuators.teleport2.Teleport2"
    _blendname = "teleport2"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

