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
import os, getopt, numpy as np, pinocchio as pin

from momentumopt.kinoptpy.momentum_kinematics_optimizer import MomentumKinematicsOptimizer
from momentumopt.motion_execution import MotionExecutor
from momentumopt.kinoptpy.create_data_file import create_file, create_qp_files, create_lqr_files

from momentumopt.motion_planner import MotionPlanner
from .robots.blmc_robot_wrapper import QuadrupedWrapper, Quadruped12Wrapper, BipedWrapper, BipedTocabiWrapper

import matplotlib.pyplot as plt


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
    if i == 0 and j == 0:
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
    i = 0
    j = 0
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

    # Dump the computed trajectory in a files (should follow the dynamic graph format)
    motion_planner.save_files()

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
