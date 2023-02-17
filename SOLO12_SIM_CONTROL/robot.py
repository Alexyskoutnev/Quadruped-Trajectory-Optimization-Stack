class SOLO12(object):
    def __init__(self, client, robot, config):
        self.client = client
        self.robot = robot
        self.config = config
        
    def get_link_states(self):
        pass
        # return self.client.getLinkStates(self.robot)
        
    def get_endeffector_states(self):
        pass