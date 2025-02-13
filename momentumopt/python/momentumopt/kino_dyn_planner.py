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
sys.path.append('/usr/local/lib/python3.8/dist-packages')
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
    print("aa")
    RobotWrapper = BipedTocabiWrapper
    with_lqr = False
    print("Cc")
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

def build_optimization(cfg_file, RobotWrapper, with_lqr, i, j, k, l, l1, j5, H2):
    """
    Build the optimization problem
    """
    # create the planner
    #if i == 0 and j == 0:
    motion_planner = MotionPlanner(cfg_file, MomentumKinematicsOptimizer, RobotWrapper, with_lqr)

    # load all the parameters of the planner
    motion_planner.init_from_settings(i, j, k, l, l1, j5, H2)

    return motion_planner

def optimize_motion(motion_planner, plot_com_motion=False):
    """
    Optimize the motion using the kino-dyn optimizer.
    For the dynamics we use the centroidal dynamics solver from this package.
    Fro the Kinematics we use a python written kinematics solver.
    """

    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
      dynamics_feedback, planner_setting, time_vector, result, result1 = \
      motion_planner.optimize_motion(plot_com_motion)
    
    # Optimize the dynamic and kinematic motion.
    return optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
           dynamics_feedback, planner_setting, time_vector, result, result1


def build_and_optimize_motion(cfg_file, RobotWrapper, with_lqr, i, j, k, l, l1, j1, H2, plot_com_motion=False):
    """ Build the optimization problem and solve it in one go."""
    result12 = 0
    while(result12 == 0):
        print("Recompute")
        motion_planner = build_optimization(cfg_file, RobotWrapper, with_lqr,  i, j, k, l, l1, j1, H2)
        optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
        dynamics_feedback, planner_setting, time_vector, result, result12= \
            optimize_motion(motion_planner, plot_com_motion)
    
    return motion_planner, optimized_kin_plan, optimized_motion_eff, \
           optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector, result


def main(argv):
    """
    Main function for optimization demo
    """
    cfg_file, RobotWrapper, with_lqr = parse_arguments(argv)

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
    boole = False
    JJJJ = 5 #2
    for i1 in range(JJJJ,JJJJ+1):
        for j1 in range(0,5):#5):
            for j2 in range(0,5):#5):
                for j3 in range(0,5):#5):
                    for j4 in range(0,5):#5): #speed
                        for j5 in range(0,3):#3):
                            # 이번에는 4
                            if j1 == 1 and j2 == 0 and j3 == 0 and j4 == 0 and j5 == 2:
                                boole  = True
        
                            if j1 == 2 and j2 == 2 and j3 == 0 and j4 == 0 and j5 == 0:
                                
                                boole = False

                            H2 = -0.0002 # 0.0003 0.0006 -0.0002
                            
                            H3 = '_3'
                            
                            if boole == True:
                                # Compute the motion
                                (motion_planner, optimized_kin_plan,
                                optimized_motion_eff,
                                optimized_dyn_plan,
                                dynamics_feedback,
                                planner_setting,
                                time_vector, result) = build_and_optimize_motion(cfg_file, RobotWrapper, with_lqr, i1, j1, j2, j3, j4, j5,  H2)

                                # The default visualizer is Meshcat, if you wanna use geppeto_viewer
                                # pass viz="gepetto" as an argument.
                                print("re")
                                print(result)

                                if result != 2:
                                    #motion_planner.replay_kinematics(viz="gepetto")           
                                    state_q = []
                                    state_qd = []
                                    state_x = []
                                    state_xkin = []
                                    state_u = []
                                    state_ud = []
                                    state_udkin = []

                                    for i in range(0, len(optimized_dyn_plan.dynamics_states)):
                                        state_q.append(optimized_kin_plan.kinematics_states[i].robot_posture.generalized_joint_positions)
                                        state_qd.append(optimized_kin_plan.kinematics_states[i].robot_velocity.generalized_joint_velocities)
                                        state_x.append([optimized_dyn_plan.dynamics_states[i].com[0], optimized_dyn_plan.dynamics_states[i].lmom[0]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[0], optimized_dyn_plan.dynamics_states[i].amom[1],
                                                        optimized_dyn_plan.dynamics_states[i].com[1], optimized_dyn_plan.dynamics_states[i].lmom[1]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[1], optimized_dyn_plan.dynamics_states[i].amom[0],
                                                        optimized_dyn_plan.dynamics_states[i].com[2], optimized_dyn_plan.dynamics_states[i].lmom[2]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[2]])
                                        state_xkin.append([optimized_kin_plan.kinematics_states[i].com[0], optimized_kin_plan.kinematics_states[i].lmom[0]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[0], optimized_kin_plan.kinematics_states[i].amom[1],
                                                        optimized_kin_plan.kinematics_states[i].com[1], optimized_kin_plan.kinematics_states[i].lmom[1]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[1], optimized_kin_plan.kinematics_states[i].amom[0],
                                                        optimized_kin_plan.kinematics_states[i].com[2], optimized_kin_plan.kinematics_states[i].lmom[2]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[2]])
                                    
                                    
                                    '''
                                    for i in range(0, len(optimized_dyn_plan.dynamics_states)-1):    
                                        state_ud.append([optimized_dyn_plan.dynamics_states[i].zmpd[0], optimized_dyn_plan.dynamics_states[i].amomd[1], optimized_dyn_plan.dynamics_states[i].zmpd[1], optimized_dyn_plan.dynamics_states[i].amomd[0]]) #, optimized_dyn_plan.dynamics_states[i].lmomd[0]/95.941282,optimized_dyn_plan.dynamics_states[i].lmomd[1]/95.941282])
                                        state_udkin.append([optimized_dyn_plan.dynamics_states[i].zmpd[0], optimized_dyn_plan.dynamics_states[i].amomd[1], optimized_dyn_plan.dynamics_states[i].zmpd[1], optimized_dyn_plan.dynamics_states[i].amomd[0]])
                                        state_u.append(optimized_kin_plan.kinematics_states[i].robot_acceleration.generalized_joint_accelerations)
                                    '''
                                    crocs_data['Right']['trajs'].append(copy(state_q))
                                    crocs_data['Right']['vel_trajs'].append(copy(state_qd))
                                    crocs_data['Right']['x_state'].append(copy(state_x))
                                    crocs_data['Right']['x_inputs'].append(copy([i1, j1, j2, j3, j4]))

                                    H1 = '/home/jhk/walkingdata1/stairdown/25cm/ssp2/timestep=40/Fdyn_data5'
                                    G = H1 + str(i1) + '_' + str(H2) + H3 + '.txt'
                                    print(G)
                                    with open(G,'wb') as f:
                                        pickle.dump(crocs_data,f)
                                        print("dump")
                                    for ss in range(0, 10):
                                        print(ss, "ss")
                                        if ss >= 1:
                                            print(optimized_dyn_plan.dynamics_states[ss].lmomd[0]/95.941282 - (9.81+optimized_dyn_plan.dynamics_states[ss].lmomd[2]/95.941282)/(optimized_dyn_plan.dynamics_states[ss].com[2]-optimized_dyn_plan.dynamics_states[ss].zmp[2])*(optimized_dyn_plan.dynamics_states[ss].com[0]-optimized_dyn_plan.dynamics_states[ss].zmp[0]-optimized_dyn_plan.dynamics_states[ss].amomd[1]/(95.941282*(9.81+optimized_dyn_plan.dynamics_states[ss].lmomd[2]/95.941282))))
                                        print(state_q[ss][0:3])
                                        print(state_x[ss])
                                        print(state_xkin[ss])
                                    
    print("result")
    print(result)  
    #a = finish
    k = sfds
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
