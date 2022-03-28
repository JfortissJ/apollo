import shutil
import time
from datetime import datetime
import numpy as np
import os.path
from shutil import copy2
from pathlib import Path
from math import log, sin, cos, atan
import pickle

from numpy.lib.function_base import copy
from cyber_py3 import cyber
from cyber_py3 import cyber_time

from modules.planning.proto import planning_pb2
from modules.planning.proto import bark_interface_pb2
from modules.canbus.proto import chassis_detail_pb2
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_config_pb2

import common.proto_utils as proto_utils

import bark
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.commons.parameters import ParameterServer
from bark.core.geometry import Line2d

from bark_ml.environments.external_runtime import ExternalRuntime
from bark_ml.library_wrappers.lib_tf_agents.agents.sac_agent import BehaviorSACAgent
from bark_ml.experiment.experiment_runner import ExperimentRunner

CHANNEL_NAME_REQUEST = '/apollo/planning/apollo_to_bark'
CHANNEL_NAME_RESPONSE = '/apollo/planning/bark_response'
CHANNEL_NAME_CANBUS = '/apollo/canbus/chassis_detail'
CHANNEL_NAME_LOCALIZATION = '/apollo/localization/pose'

# this is also the parameter file of the apollo bark_rl_planner node
BARK_RL_PLANNER_CONF_FILE = "/apollo/modules/planning/conf/bark_rl_planning_config.pb.txt"

# TODO set reasonable values!
STEERING_WHEEL_TORQUE_LIMIT = 10.0
THROTTLE_PEDAL_LIMIT = 10.0
BRAKE_PEDAL_LIMIT = 10.0

LOG_DIRECTORY_PATH = None
LOG_FILE = None
EXCEPT_LOG_FILE = None
SERIALIZATION_FILE = None

def create_log_file():
    # Implementation follows modules/drivers/lidar/velodyne/parser/scripts/velodyne_check.py
    data_time = time.strftime(
        '%Y-%m-%d-%H-%M-%S', time.localtime(cyber_time.Time.now().to_sec()))    
    global LOG_DIRECTORY_PATH
    LOG_DIRECTORY_PATH = "/apollo/data/log/experiment_{}".format(data_time)
    Path(LOG_DIRECTORY_PATH).mkdir(parents=True, exist_ok=True)
    file_name = LOG_DIRECTORY_PATH + '/bark_rl_wrapper.' + data_time + '.log'
    except_file_name = LOG_DIRECTORY_PATH + '/bark_rl_wrapper.' + data_time + '.log.err'
    serialization_file_name = LOG_DIRECTORY_PATH + '/bark_serialization.' + data_time + '.pickle'
    global LOG_FILE
    global EXCEPT_LOG_FILE
    global SERIALIZATION_FILE
    LOG_FILE = open(file_name, 'a+')
    EXCEPT_LOG_FILE = open(except_file_name, 'a+')
    SERIALIZATION_FILE = open(serialization_file_name,'wb')


def load_bark_rl_planning_config():
    planning_conf_pb = planning_config_pb2.PlanningConfig()
    proto_utils.get_pb_from_text_file(BARK_RL_PLANNER_CONF_FILE, planning_conf_pb)
    copy2(BARK_RL_PLANNER_CONF_FILE, LOG_DIRECTORY_PATH)
    bark_rl_planner_config = planning_conf_pb.bark_rl_planner_config
    print("Reading config file: ", bark_rl_planner_config)
    return bark_rl_planner_config


def log_message(log_string: str):
    unix_time = cyber_time.Time.now()
    local_time = datetime.utcnow().isoformat(sep=' ', timespec='milliseconds')
    log_info = "{} {}:\t{}\n".format(unix_time, local_time, log_string)
    LOG_FILE.write(log_info)
        
class BarkRlWrapper(object):
    bark_rl_planning_config_: planning_config_pb2.PlanningConfig.bark_rl_planner_config
    step_time_: float
    num_steps_: float
    pts_offset_x_: float
    pts_offset_y_: float
    use_idm_: bool = False
    cycle_time_: float = 0.01
    sequence_num_: int = 0
    apollo_to_bark_received_: bool = False
    apollo_to_bark_msg_: bark_interface_pb2.ApolloToBarkMsg = bark_interface_pb2.ApolloToBarkMsg()
    chassis_detail_received_: bool = False
    chassis_detail_msg_: chassis_detail_pb2.ChassisDetail = chassis_detail_pb2.ChassisDetail()
    localization_msgs_: list = []
    response_pub_: cyber.Writer
    driver_interaction_timesteps_: list = []
    scenario_history_: dict = {}
    params_: ParameterServer
    env_: ExternalRuntime

    def __init__(self, node):
        self.bark_rl_planning_config_ = load_bark_rl_planning_config()
        self.step_time_ = self.bark_rl_planning_config_.ts
        self.num_steps_ = self.bark_rl_planning_config_.nr_steps
        self.pts_offset_x_ = self.bark_rl_planning_config_.pts_offset_x
        self.pts_offset_y_ = self.bark_rl_planning_config_.pts_offset_y

        self.response_pub_ = node.create_writer(CHANNEL_NAME_RESPONSE,
                                                bark_interface_pb2.BarkResponse)

        # TODO: make sure the same maps are being used (BARK-ML != BARK MAP)
        # folder stucture needs to be as follows:
        # /apollo/modules/planning/data/20211111_checkpoints/ HERE THE JSON NEEDS TO BE
        # /apollo/modules/planning/data/20211111_checkpoints/single_lane_large/0/ckps/ HERE THE CKPTS NEED TO BE
        # json_file_path = "/apollo/modules/planning/data/20211118_checkpoints/single_lane_large_max_vel.json"
        json_file_path = "/apollo/modules/planning/data/20220322_checkpoints/case_be_careful_new.json"
        # json_file_path = "/apollo/modules/planning/data/20211210_checkpoints/dense_small_limits_512.json"
        copy2(json_file_path, LOG_DIRECTORY_PATH)
        self.params_ = ParameterServer(filename=json_file_path)
        self.params_["SingleLaneBluePrint"]["MapOffstX"] = self.pts_offset_x_
        self.params_["SingleLaneBluePrint"]["MapOffsetY"] = self.pts_offset_y_
        # we call initialize_external_runtime here due to the long loading time of the network
        self.initialize_external_runtime(json_file_path)

    def initialize_external_runtime(self, json_file_path: str):
        exp_runner = ExperimentRunner(json_file=json_file_path, params=self.params_, mode="print", random_seed=0, use_best_ckpt_folder = True)
        observer = exp_runner._experiment._observer
        map_interface = exp_runner._experiment._blueprint._scenario_generation._map_interface
        viewer = MPViewer(params=self.params_)
        self.env_ = ExternalRuntime(
            map_interface=map_interface, observer=observer,
            params=self.params_, viewer=viewer, render=False)
        self.env_.setupWorld()
        # setting up ego agent initially as this takes some time...
        dummy_state = np.array([0, 0, 0, 0, 0])
        self.setup_ego_model()
        goal_line = Line2d(np.array([[0., 0.], [1., 1.]]))
        self.env_.addEgoAgent(dummy_state, goal_line)

    def convert_to_bark_state(self, traj_pt, time_offset):
        t_e = traj_pt.relative_time + time_offset
        x_e = traj_pt.path_point.x - self.pts_offset_x_
        y_e = traj_pt.path_point.y - self.pts_offset_y_
        theta_e = traj_pt.path_point.theta
        v_e = traj_pt.v
        state = np.array([t_e, x_e, y_e, theta_e, v_e])
        return state

    def convert_to_ego_bark_state(self, traj_pt, time_offset):
        # bark state of rl agent is normal state extended by steering angle
        t_e = traj_pt.relative_time + time_offset
        x_e = traj_pt.path_point.x - self.pts_offset_x_
        y_e = traj_pt.path_point.y - self.pts_offset_y_
        theta_e = traj_pt.path_point.theta
        v_e = traj_pt.v
        wheelbase = 2.786 # get from common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().wheel_base() ?
        # delta = atan(kappa * l) follows from kappa = tan(delta)/l
        delta = atan(traj_pt.path_point.kappa) * wheelbase
        state = np.array([t_e, x_e, y_e, theta_e, v_e, delta])
        state_string = np.array2string(state, formatter={'float_kind':lambda x: "%.2f" % x})
        log_message("Initializing bark with ego state: {}".format(state_string))
        return state

    def convert_reference_line_to_bark_line(self, reference_line):
        pts = []
        N = len(reference_line)
        for idx, ref_pt in enumerate(reference_line):
            if idx > N - 100: # only use last points
                pts.append([ref_pt.x - self.pts_offset_x_, ref_pt.y - self.pts_offset_y_])
        
        pts_np = np.array(pts)
        # print("goal line", pts_np)
        new_line = Line2d(pts_np)
        return new_line


    def setup_ego_model(self):
        if self.use_idm_:
            # IDM cannot deal with desired velocity equal to zero
            desired_velocity = max([0.01, float(self.apollo_to_bark_msg_.velocity_desired)])
            self.params_["BehaviorIDMClassic"]["DesiredVelocity"] = desired_velocity
            self.env_._ml_behavior = bark.core.models.behavior.BehaviorIDMClassic(self.params_)
        else:
            sac_agent = BehaviorSACAgent(environment=self.env_, params=self.params_)
            self.env_._ml_behavior = sac_agent

    def publish_planningmsg(self):

        if not self.apollo_to_bark_received_:
            print("apollo to bark msg not received yet")
            return

        time0 = time.time()

        # step 1: setup environment
        if self.use_idm_:
            # necessary to update desired velocity in IDM, but recreating SAC agent every time would take too long.
            self.setup_ego_model() 
        self.env_.setupWorld()
        time1 = time.time()
        log_message("Creating world took {}s, time since beginning {}".format(time1-time0, time1-time0))

        # step 2: init ego vehicle with planning_init_point
        pl_init_pt = self.apollo_to_bark_msg_.planning_init_point
        state = self.convert_to_ego_bark_state(pl_init_pt, -pl_init_pt.relative_time)
        goal_line = self.convert_reference_line_to_bark_line(self.apollo_to_bark_msg_.reference_line)
        self.env_.addEgoAgent(state, goal_line)
        time2 = time.time()
        log_message("Setup ego agents took {}s, time since beginning {}".format(time2-time1, time2-time0))

        # step 3: fill BARK world with perception_obstacle_msg_ (call self.env.addObstacle())
        for o in self.apollo_to_bark_msg_.obstacles:
            traj = []
            (crad, wb) = self.env_.ConvertShapeParameters(length=o.box_length, width=o.box_width)
            for pred_state in o.prediction:
                state_i = self.convert_to_bark_state(pred_state, -pl_init_pt.relative_time)
                # transform state from center to reference frame and
                # move position from center to rear axis
                theta = state_i[3]
                state_i[1] = state_i[1] - (wb/2 - o.s_distance_center_to_reference) * cos(theta)
                state_i[2] = state_i[2] - (wb/2 - o.s_distance_center_to_reference) * sin(theta)
                traj.append(state_i)
            traj_np = np.array(traj)
            self.env_.addObstacle(traj_np, o.box_length, o.box_width)

        time3 = time.time()
        log_message("Setup other agents took {}s, time since beginning {}".format(time3-time2, time3-time0))
        # TODO step 3: set reference line

        # step 4: saving scenario for serialization
        self.scenario_history_[str(time0)] = self.env_.getScenarioForSerialization()

        # step 5:
        state_action_traj = self.env_.generateTrajectory(self.step_time_, self.num_steps_)
        time4 = time.time()
        log_message("Generating trajectory took {}s, time since beginning. {}".format(time4-time3, time4-time0))
        adc_trajectory = planning_pb2.ADCTrajectory()
        traj_point = adc_trajectory.trajectory_point.add()
        # append initial state to resulting trajectory
        traj_point.CopyFrom(pl_init_pt)
        for bark_state in state_action_traj[0]:
            traj_point = adc_trajectory.trajectory_point.add()
            traj_point.relative_time = bark_state[0] + pl_init_pt.relative_time
            traj_point.path_point.x = bark_state[1] + self.pts_offset_x_
            traj_point.path_point.y = bark_state[2] + self.pts_offset_y_
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
        print("Generated Trajectory.")
        log_message("generated new trajectory: {}".format(response_msg))

    def apollo_to_bark_callback(self, data):
        """
        New Apollo Data Received
        """
        print("received apollo_to_bark msg ", self.apollo_to_bark_msg_.header)
        self.apollo_to_bark_msg_.CopyFrom(data)
        self.apollo_to_bark_received_ = True

    def chassis_detail_callback(self, data):
        """
        Received new Chassis Detail message
        """
        self.chassis_detail_msg_.CopyFrom(data)
        self.chassis_detail_received_ = True
        self.driver_interaction_triggered()

    def localization_callback(self, data):
        self.localization_msgs_.append({'Time': data.header.timestamp_sec, 'Localization_Pose': data.pose})

    def driver_interaction_triggered(self):
        # TODO idealy check these fields for existance
        steering_wheel_troque = self.chassis_detail_msg_.fortuna.steering.steering_wheel_torque
        throttle = self.chassis_detail_msg_.gas.throttle_input
        brake_input = self.chassis_detail_msg_.brake.brake_input

        # print("steering_wheel_troque {}, throttle {}, brake_input {}".format(steering_wheel_troque, throttle, brake_input))

        interaction = False
        if(steering_wheel_troque > STEERING_WHEEL_TORQUE_LIMIT):
            interaction = True
        if(throttle > THROTTLE_PEDAL_LIMIT):
            interaction = True
        if(brake_input > BRAKE_PEDAL_LIMIT):
            interaction = True

        if interaction:
            time = self.chassis_detail_msg_.timestamp
            self.driver_interaction_timesteps_.append(time)
            print("Found driver interaction at t = {}".format(time))

def main():
    """
    Main function
    """
    node = cyber.Node("bark_rl_node")
    bark_wrp = BarkRlWrapper(node)

    node.create_reader(CHANNEL_NAME_REQUEST,
                       bark_interface_pb2.ApolloToBarkMsg,
                       bark_wrp.apollo_to_bark_callback)

    node.create_reader(CHANNEL_NAME_CANBUS,
                       chassis_detail_pb2.ChassisDetail,
                       bark_wrp.chassis_detail_callback)

    node.create_reader(CHANNEL_NAME_LOCALIZATION,
                       localization_pb2.LocalizationEstimate,
                       bark_wrp.localization_callback)

    while not cyber.is_shutdown():
        now = cyber_time.Time.now().to_sec()
        bark_wrp.publish_planningmsg()
        sleep_time = bark_wrp.cycle_time_ - (cyber_time.Time.now().to_sec() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)

    pickle.dump([bark_wrp.scenario_history_, bark_wrp.driver_interaction_timesteps_, bark_wrp.localization_msgs_], SERIALIZATION_FILE)
    print("saved serialization")


if __name__ == '__main__':
    create_log_file()
    cyber.init()
    main()
    cyber.shutdown()
