import time

from cyber_py3 import cyber
from cyber_py3 import cyber_time

from common.logger import Logger
from modules.planning.proto import planning_pb2
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles

import bark_ml.environments.gym

class BarkRlWrapper(object):
    def __init__(self, node):

        self.perception_obstacles_received_ = False
        self.perception_obstacle_msg_ = PerceptionObstacles()
        self.planning_pub_ = node.create_writer('/apollo/planning', planning_pb2.ADCTrajectory)

    def publish_planningmsg(self):

        if not self.perception_obstacles_received_:
            print("perception not received yet")
            return

        trajectory_msg = planning_pb2.ADCTrajectory()
        # TODO: fill this with data from BARK-ML
        # 
        # 
        self.planning_pub_.write(trajectory_msg)
        self.perception_obstacles_received_ = False

        print("Generated Trajectory")

    def perception_callback(self, data):
        """
        New Perception Received
        """
        self.perception_obstacle_msg_.CopyFrom(data)
        self.perception_obstacles_received_ = True

def main():
    """
    Main function
    """
    node = cyber.Node("bark_rl_node")
    bark_rl_wrapper = BarkRlWrapper(node)

    # TODO: read message with init point, trajectory, and reference line
    node.create_reader('/apollo/perception/obstacles', PerceptionObstacles, bark_rl_wrapper.perception_callback)

    while not cyber.is_shutdown():
        now = cyber_time.Time.now().to_sec()
        bark_rl_wrapper.publish_planningmsg()
        cycle_time = 0.2 # TODO: Change here
        sleep_time = cycle_time - (cyber_time.Time.now().to_sec() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)



if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
