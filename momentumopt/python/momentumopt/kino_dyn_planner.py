#! /usr/bin/python

'''
@file PyDemoMomentumopt.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

import time, sys
sys.path.append('/usr/local/lib/python3/dist-packages')
sys.path.append('/home/jhk/kino_dynamic_opt/momentumopt/build/')
from pysolver import *
from pysolverlqr import *
from pymomentum import *
from pinocchio.utils import *
from copy import copy
import os, getopt, numpy as np, pinocchio as pin
np.set_printoptions(threshold=sys.maxsize)

from momentumopt.kinoptpy.momentum_kinematics_optimizer import MomentumKinematicsOptimizer
from momentumopt.motion_execution import MotionExecutor
from momentumopt.kinoptpy.create_data_file import create_file, create_qp_files, create_lqr_files

from momentumopt.motion_planner import MotionPlanner
from .robots.blmc_robot_wrapper import QuadrupedWrapper, Quadruped12Wrapper, BipedWrapper, BipedTocabiWrapper

import matplotlib.pyplot as plt
import pickle


def parse_arguments(argv):
    cfg_file = ''
    try:
        opts, args = getopt.getopt(argv,"hi:m",["ifile=", "solo12", "bolt", "tocabi", "disable_lqr"])
    except getopt.GetoptError:
        print ('python kino_dyn_planner.py -i <path_to_datafile>')
        sys.exit(2)

    RobotWrapper = QuadrupedWrapper
    with_lqr = False

    for opt, arg in opts:
        if opt == '-h':
            print ('PyDemoMomentumopt.py -i <path_to_datafile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            cfg_file = arg
        elif opt in ("--solo12"):
            RobotWrapper = Quadruped12Wrapper
        elif opt in ("--bolt"):
            RobotWrapper = BipedWrapper
        elif opt in ("--tocabi"):
            RobotWrapper = BipedTocabiWrapper
        elif opt in ("--disable_lqr"):
            with_lqr = False

    if not os.path.exists(cfg_file):
        raise RuntimeError("The config file " + cfg_file + " does not exist.")

    return cfg_file, RobotWrapper, with_lqr

def build_optimization(cfg_file, RobotWrapper, with_lqr, i, j):
    """
    Build the optimization problem
    """
    # create the planner
    #if i == 0 and j == 0:
    motion_planner = MotionPlanner(cfg_file, MomentumKinematicsOptimizer, RobotWrapper, with_lqr)

    # load all the parameters of the planner
    motion_planner.init_from_settings(i, j)

    return motion_planner

def optimize_motion(motion_planner, plot_com_motion=False):
    """
    Optimize the motion using the kino-dyn optimizer.
    For the dynamics we use the centroidal dynamics solver from this package.
    Fro the Kinematics we use a python written kinematics solver.
    """

    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
      dynamics_feedback, planner_setting, time_vector = \
      motion_planner.optimize_motion(plot_com_motion)

    # Optimize the dynamic and kinematic motion.
    return optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
           dynamics_feedback, planner_setting, time_vector


def build_and_optimize_motion(cfg_file, RobotWrapper, with_lqr, i, j, plot_com_motion=False):
    """ Build the optimization problem and solve it in one go."""

    motion_planner = build_optimization(cfg_file, RobotWrapper, with_lqr,  i, j)
    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
      dynamics_feedback, planner_setting, time_vector = \
          optimize_motion(motion_planner, plot_com_motion)
    
    return motion_planner, optimized_kin_plan, optimized_motion_eff, \
           optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector


def main(argv):
    """
    Main function for optimization demo
    """
    # Get the arguments
    cfg_file, RobotWrapper, with_lqr = parse_arguments(argv)
    
    for i in range(0, 1):
        j=0
        # Compute the motion
        (motion_planner, optimized_kin_plan,
        optimized_motion_eff,
        optimized_dyn_plan,
        dynamics_feedback,
        planner_setting,
        time_vector) = build_and_optimize_motion(cfg_file, RobotWrapper, with_lqr, i, j)

        # The default visualizer is Meshcat, if you wanna use geppeto_viewer
        # pass viz="gepetto" as an argument.
        motion_planner.replay_kinematics(viz="gepetto")
        crocs_data = dict()
        crocs_data['Right'] = dict()
        crocs_data['Right']['x_inputs'] = []
        crocs_data['Right']['x_state'] = []  
        crocs_data['Right']['trajs'] = []  
        crocs_data['Right']['vel_trajs'] = [] 
        crocs_data['Right']['acc_trajs'] = []        
        crocs_data['Right']['u_trajs'] = []

        crocs_data1 = dict()
        crocs_data1['Right'] = dict()
        crocs_data1['Right']['x_inputs'] = []
        crocs_data1['Right']['x_state'] = []  
        crocs_data1['Right']['trajs'] = []  
        crocs_data1['Right']['vel_trajs'] = [] 
        crocs_data1['Right']['acc_trajs'] = []        
        crocs_data1['Right']['u_trajs'] = []
        
        state_q = []
        state_qd = []
        state_x = []
        state_xkin = []
        state_u = []
        state_ud = []
        state_udkin = []
        print(len(optimized_kin_plan.kinematics_states))
        print(len(optimized_dyn_plan.dynamics_states))
        for i in range(0, len(optimized_dyn_plan.dynamics_states)):
            state_q.append(optimized_kin_plan.kinematics_states[i].robot_posture.generalized_joint_positions)
            state_qd.append(optimized_kin_plan.kinematics_states[i].robot_velocity.generalized_joint_velocities)
            state_x.append([optimized_dyn_plan.dynamics_states[i].com[0], optimized_dyn_plan.dynamics_states[i].lmom[0]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[0], optimized_dyn_plan.dynamics_states[i].amom[1],
                            optimized_dyn_plan.dynamics_states[i].com[1], optimized_dyn_plan.dynamics_states[i].lmom[1]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[1], optimized_dyn_plan.dynamics_states[i].amom[0]])
            state_xkin.append([optimized_kin_plan.kinematics_states[i].com[0], optimized_kin_plan.kinematics_states[i].lmom[0]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[0], optimized_kin_plan.kinematics_states[i].amom[1],
                            optimized_kin_plan.kinematics_states[i].com[1], optimized_kin_plan.kinematics_states[i].lmom[1]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[1], optimized_kin_plan.kinematics_states[i].amom[0]])
        
        for i in range(0, len(optimized_dyn_plan.dynamics_states)-1):    
            state_ud.append([optimized_dyn_plan.dynamics_states[i].zmpd[0], optimized_dyn_plan.dynamics_states[i].amomd[1], optimized_dyn_plan.dynamics_states[i].zmpd[1], optimized_dyn_plan.dynamics_states[i].amomd[0]])
            state_udkin.append([optimized_dyn_plan.dynamics_states[i].zmpd[0], optimized_dyn_plan.dynamics_states[i].amomd[1], optimized_dyn_plan.dynamics_states[i].zmpd[1], optimized_dyn_plan.dynamics_states[i].amomd[0]])
            state_u.append(optimized_kin_plan.kinematics_states[i].robot_acceleration.generalized_joint_accelerations)
       
        crocs_data['Right']['trajs'].append(copy(state_q))
        crocs_data['Right']['vel_trajs'].append(copy(state_qd))
        crocs_data['Right']['u_trajs'].append(copy(state_u))
        crocs_data['Right']['acc_trajs'].append(copy(state_ud))
        crocs_data['Right']['x_state'].append(copy(state_x))

        crocs_data1['Right']['trajs'].append(copy(state_q))
        crocs_data1['Right']['vel_trajs'].append(copy(state_qd))
        crocs_data1['Right']['u_trajs'].append(copy(state_u))
        crocs_data1['Right']['x_state'].append(copy(state_xkin))
        crocs_data['Right']['acc_trajs'].append(copy(state_udkin))

        print(state_x[0])
        print(state_x[1])
        print(state_q[0])
        #print(state_ud[0])

        # Dump the computed trajectory in a files (should follow the dynamic graph format)
        motion_planner.save_files()
    
    with open('/home/jhk/kino_dynamic_opt/momentumopt/demos/Fdyn.txt','wb') as f:
        pickle.dump(crocs_data,f)
    with open('/home/jhk/kino_dynamic_opt/momentumopt/demos/kdyn.txt','wb') as f:
        pickle.dump(crocs_data1,f)

    # Display the motion
    display = True
    if(display): # plot trajectories
        motion_planner.plot_foot_traj()
        motion_planner.plot_joint_trajecory()
        motion_planner.plot_com_motion(optimized_dyn_plan.dynamics_states, optimized_kin_plan.kinematics_states)
        motion_planner.plot_base_trajecory()

    # Potentially simulate the motion
    simulation = False
    if simulation:
        motion_executor = MotionExecutor(optimized_kin_plan, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector)
        motion_executor.execute_motion(plotting=False, tune_online=False)

    print('Done...')

if __name__ == "__main__":
    main(sys.argv[1:])
