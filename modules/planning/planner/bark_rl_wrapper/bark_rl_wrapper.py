import time
import numpy as np
import os.path
from cyber_py3 import cyber
from cyber_py3 import cyber_time

from common.logger import Logger
from modules.planning.proto import planning_pb2
from modules.planning.proto import bark_interface_pb2

import bark
from bark.core.world.map import MapInterface
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.commons.parameters import ParameterServer

from bark_ml.environments.external_runtime import ExternalRuntime
from bark_ml.library_wrappers.lib_tf_agents.agents.sac_agent import BehaviorSACAgent
from bark_ml.observers.nearest_state_observer import NearestAgentsObserver


CHANNEL_NAME_REQUEST = '/apollo/planning/apollo_to_bark'
CHANNEL_NAME_RESPONSE = '/apollo/planning/bark_response'

def convert_to_bark_state(traj_pt, time_offset):
    t_e = traj_pt.relative_time + time_offset
    x_e = traj_pt.path_point.x
    y_e = traj_pt.path_point.y
    theta_e = traj_pt.path_point.theta
    v_e = traj_pt.v
    state = np.array([t_e, x_e, y_e, theta_e, v_e])
    return state

class BarkRlWrapper(object):
    def __init__(self, node):

        self.apollo_to_bark_received_= False
        self.apollo_to_bark_msg_ = bark_interface_pb2.ApolloToBarkMsg()
        self.response_pub_ = node.create_writer(CHANNEL_NAME_RESPONSE, 
                                                bark_interface_pb2.BarkResponse)

        self.params_ = ParameterServer()
        observer = NearestAgentsObserver()
        csvfile = "/apollo/modules/planning/data/base_map_lanes_guerickestr_assymetric_48.csv"
        if not os.path.isfile(csvfile):
            print("map file does not exist, path might be wrong: {}".format(csvfile))
        map_interface = MapInterface()
        map_interface.SetCsvMap(csvfile)
        self.viewer_ = MPViewer(params=self.params_)
        self.env_ = ExternalRuntime(map_interface=map_interface, observer=observer, params=self.params_, viewer=self.viewer_, render=False)
        if not isinstance(self.env_._world, bark.core.world.World):
            print("BARK World could not be created")
        self.step_time_ = 0.2 # this should come from pb param file
        self.num_steps_ = 20 # this should come from pb param file
        self.cycle_time_ = 0.2
        self.sequence_num_ = 0
        self.use_idm_ = True


    def publish_planningmsg(self):

        if not self.apollo_to_bark_received_:
            print("apollo to bark msg not received yet")
            return
        
        # step 1: setup environment
        self.env_.setupWorld()

        # step 2: init ego vehicle with planning_init_point
        pl_init_pt = self.apollo_to_bark_msg_.planning_init_point
        print("planning init point received ", pl_init_pt)
        state = convert_to_bark_state(pl_init_pt, -pl_init_pt.relative_time)
        # TODO: how to handle time stamp of ego state?
        if self.use_idm_:
            self.params_["BehaviorIDMClassic"]["DesiredVelocity"] = float(self.apollo_to_bark_msg_.velocity_desired)
            self.env_._ml_behavior = bark.core.models.behavior.BehaviorIDMClassic(self.params_)
        else:
            sac_agent = BehaviorSACAgent(environment=self.env_, params=self.params_)
            self.env_._ml_behavior = sac_agent
        self.env_.addEgoAgent(state)
        
        # step 3: fill BARK world with perception_obstacle_msg_ (call self.env.addObstacle())
        for o in self.apollo_to_bark_msg_.obstacles:
            traj = np.array()
            for pred_state in o.prediction:
                state_i = convert_to_bark_state(pred_state, -pl_init_pt.relative_time)
                traj.append(state_i)
            self.env_.addObstacle(traj, o.box_length, o.box_width)

        # TODO step 3: set reference line

        # step 4: 
        state_action_traj = self.env_.generateTrajectory(self.step_time_, self.num_steps_)
        adc_trajectory = planning_pb2.ADCTrajectory()
        traj_point = adc_trajectory.trajectory_point.add()
        # append initial state to resulting trajectory
        traj_point.CopyFrom(pl_init_pt)
        for bark_state in state_action_traj[0]:
            traj_point = adc_trajectory.trajectory_point.add()
            traj_point.relative_time = bark_state[0] + pl_init_pt.relative_time
            traj_point.path_point.x = bark_state[1]
            traj_point.path_point.y = bark_state[2]
            traj_point.path_point.theta = bark_state[3]
            traj_point.v = bark_state[4]
            # TODO: do we need to fill traj_point.path_point.s?
        
        response_msg = bark_interface_pb2.BarkResponse()
        response_msg.planned_trajectory.CopyFrom(adc_trajectory)
        response_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        response_msg.header.module_name = 'bark_response'
        response_msg.header.sequence_num = self.sequence_num_
        self.sequence_num_ = self.sequence_num_ + 1

        self.response_pub_.write(response_msg)
        self.apollo_to_bark_received_ = False
        print("Generated Trajectory")

    def apollo_to_bark_callback(self, data):
        """
        New Apollo Data Received
        """
        print("received apollo_to_bark msg ", self.apollo_to_bark_msg_.header)
        self.apollo_to_bark_msg_.CopyFrom(data)
        self.apollo_to_bark_received_ = True

def main():
    """
    Main function
    """
    node = cyber.Node("bark_rl_node")
    bark_wrp = BarkRlWrapper(node)

    node.create_reader(CHANNEL_NAME_REQUEST, 
                       bark_interface_pb2.ApolloToBarkMsg, 
                       bark_wrp.apollo_to_bark_callback)

    while not cyber.is_shutdown():
        now = cyber_time.Time.now().to_sec()
        bark_wrp.publish_planningmsg()
        sleep_time = bark_wrp.cycle_time_ - (cyber_time.Time.now().to_sec() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)



if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
