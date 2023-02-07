#!/usr/bin/python

'''
@file motion_planner.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

import time
import numpy as np
import matplotlib.pyplot as plt

from .robots.blmc_robot_wrapper import QuadrupedWrapper
from pysolver import *
from pysolverlqr import *

from pymomentum import *
from momentumopt.kinoptpy.momentum_kinematics_optimizer import MomentumKinematicsOptimizer, EndeffectorTrajectoryGenerator
from momentumopt.kinoptpy.create_data_file import create_file, create_file1, create_qp_files, create_lqr_files, create_lqr_files1
from momentumopt.motion_execution import desired_state
import pickle
from copy import copy

np.set_printoptions(precision=3)

def create_time_vector(dynamics_sequence):
    num_time_steps = len(dynamics_sequence.dynamics_states)
    # Create time vector
    time = np.zeros((num_time_steps))
    for i in range(num_time_steps - 1):
        time[i + 1] = time[i] + dynamics_sequence.dynamics_states[i].dt

    return time


class MotionPlanner():

    def __init__(self, cfg_file, KinOpt=MomentumKinematicsOptimizer,
                 RobotWrapper=QuadrupedWrapper, with_lqr=False):
        'define problem configuration'

        self.planner_setting = PlannerSetting()
        self.planner_setting.initialize(cfg_file)

        self.dynlqr_setting = SolverLqrSetting()
        self.dynlqr_setting.initialize(cfg_file, "solverlqr_dynamics")

        'define robot initial state'
        self.ini_state = DynamicsState()
        self.ini_state.fillInitialRobotState(cfg_file)

        'define reference dynamic sequence'
        self.kin_sequence = KinematicsSequence()
        self.kin_sequence.resize(self.planner_setting.get(PlannerIntParam_NumTimesteps),
                                 self.planner_setting.get(PlannerIntParam_NumDofs))

        'define terrain description'
        self.terrain_description = TerrainDescription()
        self.terrain_description.loadFromFile(self.planner_setting.get(PlannerStringParam_ConfigFile))

        'define contact plan'
        self.contact_plan = ContactPlanFromFile()
        self.contact_plan.initialize(self.planner_setting)
        self.contact_plan.optimize(self.ini_state, self.terrain_description)

        'optimize motion'
        self.dyn_optimizer = DynamicsOptimizer()
        self.dyn_optimizer.initialize(self.planner_setting)

        'Kinematics Optimizer'
        self.kin_optimizer = KinOpt()
        #self.KinematicsInterface = KinematicsInterface() KinematicsOptimizer()#
        #self.KinematicsInterface.initialize(self.planner_setting)
        #self.KinematicsInterface.internalInitialization(self.planner_setting)
        self.kin_optimizer.initialize(self.planner_setting, RobotWrapper=RobotWrapper)
        self.dynamics_feedback = None
        self.with_lqr = with_lqr
        self.kd_iter1 = 0

        self.crocs_data = dict()
        self.crocs_data['Right'] = dict()
        #self.crocs_data['foot_poses'] = []
        #self.crocs_data['trajs'] = []
        #self.crocs_data['x'] = []
        #self.crocs_data['vel_trajs'] = [] 
        self.crocs_data['Right']['x_inputs'] = []
        self.crocs_data['Right']['x_state'] = []        
        self.crocs_data['Right']['u_trajs'] = []
        #self.crocs_data['data_phases_set'] = []
       # self.crocs_data['costs'] = []
        #self.crocs_data['iters'] = []

    def init_from_settings(self):
        kin_optimizer = self.kin_optimizer
        inv_kin = kin_optimizer.inv_kin
        snd_order_inv_kin = kin_optimizer.snd_order_inv_kin
        etg = kin_optimizer.endeff_traj_generator
        kin_optimizer.use_second_order_inv_kin = self.planner_setting.get(PlannerBoolParam_UseSecondOrderInverseKinematics)
        if self.kin_optimizer.use_second_order_inv_kin:
            print("\n Second order IK formulation is used, set use_second_order_inv_kin to False "
                  "in the config file if you want to use first order IK. \n")
            etg.z_offset = self.planner_setting.get(PlannerDoubleParam_SwingTrajViaZ_Second)
            snd_order_inv_kin.w_lin_mom_tracking = self.planner_setting.get(PlannerDoubleParam_WeightLinMomentumTracking_Second)
            snd_order_inv_kin.w_ang_mom_tracking = self.planner_setting.get(PlannerDoubleParam_WeightAngMomentumTracking_Second)
            snd_order_inv_kin.w_endeff_contact = self.planner_setting.get(PlannerDoubleParam_WeightEndEffContact_Second)
            snd_order_inv_kin.w_endeff_tracking = self.planner_setting.get(PlannerDoubleParam_WeightEndEffTracking_Second)
            snd_order_inv_kin.w_endori_contact = self.planner_setting.get(PlannerDoubleParam_WeightEndOriContact_Second)
            snd_order_inv_kin.w_endori_tracking = self.planner_setting.get(PlannerDoubleParam_WeightEndOriTracking_Second)
            snd_order_inv_kin.w_joint_regularization = self.planner_setting.get(PlannerDoubleParam_WeightJointReg_Second)
            snd_order_inv_kin.p_endeff_tracking = self.planner_setting.get(PlannerDoubleParam_PGainEndEffTracking_Second)
            snd_order_inv_kin.p_com_tracking = self.planner_setting.get(PlannerDoubleParam_PGainComTracking_Second)
            kin_optimizer.n_via_joint = self.planner_setting.get(PlannerIntParam_NumJointViapoints_Second)
            kin_optimizer.via_joint = self.planner_setting.get(PlannerCVectorParam_JointViapoints_Second)
            kin_optimizer.n_via_base = self.planner_setting.get(PlannerIntParam_NumBaseViapoints_Second)
            kin_optimizer.via_base = self.planner_setting.get(PlannerCVectorParam_BaseViapoints_Second)
            # parameters specific to second order IK
            snd_order_inv_kin.d_endeff_tracking = self.planner_setting.get(PlannerDoubleParam_DGainEndEffTracking_Second)
            snd_order_inv_kin.p_orient_tracking = self.planner_setting.get(PlannerDoubleParam_PGainBaseOrientationTracking_Second)
            snd_order_inv_kin.d_orient_tracking = self.planner_setting.get(PlannerDoubleParam_DGainBaseOrientationTracking_Second)
            snd_order_inv_kin.p_orientf_tracking = self.planner_setting.get(PlannerDoubleParam_PGainFOrientationTracking_Second)
            snd_order_inv_kin.d_orientf_tracking = self.planner_setting.get(PlannerDoubleParam_DGainFOrientationTracking_Second)
            snd_order_inv_kin.p_joint_regularization = self.planner_setting.get(PlannerDoubleParam_PGainJointRegularization_Second)
            snd_order_inv_kin.d_joint_regularization =self.planner_setting.get(PlannerDoubleParam_DGainJointRegularization_Second)
            snd_order_inv_kin.p_mom_tracking = self.planner_setting.get(PlannerVectorParam_PGainMomentumTracking_Second)
        else:
            print("\n First order IK formulation is used, set use_second_order_inv_kin to True "
                  "in the config file if you want to use second order IK. \n")
            etg.z_offset = self.planner_setting.get(PlannerDoubleParam_SwingTrajViaZ)
            inv_kin.w_lin_mom_tracking = self.planner_setting.get(PlannerDoubleParam_WeightLinMomentumTracking)
            inv_kin.w_ang_mom_tracking = self.planner_setting.get(PlannerDoubleParam_WeightAngMomentumTracking)
            inv_kin.w_endeff_contact = self.planner_setting.get(PlannerDoubleParam_WeightEndEffContact)
            inv_kin.w_endeff_tracking = self.planner_setting.get(PlannerDoubleParam_WeightEndEffTracking)
            inv_kin.w_joint_regularization = self.planner_setting.get(PlannerDoubleParam_WeightJointReg)
            inv_kin.p_endeff_tracking = self.planner_setting.get(PlannerDoubleParam_PGainEndEffTracking)
            inv_kin.p_com_tracking = self.planner_setting.get(PlannerDoubleParam_PGainComTracking)
            kin_optimizer.reg_orientation = self.planner_setting.get(PlannerDoubleParam_PGainOrientationTracking)
            kin_optimizer.reg_joint_position = self.planner_setting.get(PlannerDoubleParam_PGainPositionTracking)
            kin_optimizer.n_via_joint = self.planner_setting.get(PlannerIntParam_NumJointViapoints)
            kin_optimizer.via_joint = self.planner_setting.get(PlannerCVectorParam_JointViapoints)
            kin_optimizer.n_via_base = self.planner_setting.get(PlannerIntParam_NumBaseViapoints)
            kin_optimizer.via_base = self.planner_setting.get(PlannerCVectorParam_BaseViapoints)


    def optimize_dynamics(self, kd_iter):
        print("DynOpt", kd_iter)
        start = time.time()
        self.dyn_optimizer.optimize(self.ini_state, self.contact_plan,
                                    self.kin_optimizer.kinematics_sequence, kd_iter > 0)
        print("Dynopt - " , time.time() -start)

    def optimize_kinematics(self, kd_iter, plotting=False):
        print("KinOpt", kd_iter)
        start = time.time()
        self.kin_optimizer.optimize(self.ini_state, self.contact_plan.contactSequence(),
                                    self.dyn_optimizer.dynamicsSequence(), plotting=plotting)
        print("kinopt - ", time.time() - start)

    def optimize_dynamics_feedback(self):
        # 'define dynamics feedback controller'
        # '''
        # Access feedback gains using: dynamics_feedback.forceGain(time_id)
        #                             [currentCOM  - desiredCoM ]
        #   deltaForce = forceGain *  [currentLMOM - desiredLMOM]
        #                             [currentAMOM - desiredAMOM]
        #
        #  Torque = PD(q,qdot) + J^T * (plannedForce + deltaForce)
        #  Remember that plannedForce of dyn_optimizer is normalized by robot weight
        #  (self.planner_setting.get(PlannerDoubleParam_RobotWeight)),
        #  so you need to multiply it by that amount for it to work!
        #  deltaForce comes already in the right units.
        # '''
        self.dynamics_feedback = DynamicsFeedback()
        self.dynamics_feedback.initialize(self.dynlqr_setting, self.planner_setting)
        self.dynamics_feedback.optimize(self.ini_state, self.dyn_optimizer.dynamicsSequence())

    def _plot_show(self, plot_show):
        if plot_show:
            plt.show()
        else:
            plt.draw()
            plt.pause(0.001)

    def plot_centroidal(self):
        fig, axes = plt.subplots(3, 1, figsize=(6, 8), sharex=True)

        dynseq = self.dyn_optimizer.dynamicsSequence()
        kinseq = self.kin_optimizer.kinematics_sequence

        for i, (ax, prop) in enumerate(zip(axes, ['com', 'lmom', 'amom'])):
            data_dyn = np.array([getattr(ds, prop) for ds in dynseq.dynamics_states])
            data_kin = np.array([getattr(ds, prop) for ds in kinseq.kinematics_states])

            for dyn, kin, label in zip(data_dyn.T, data_kin.T, ['{}_{}'.format(prop, d) for d in ['x', 'y', 'z']]):
                line = ax.plot(dyn, label=label, alpha=0.75)[0]
                ax.plot(kin, '--', color=line.get_color())[0]

            ax.legend()
            ax.grid(True)

        fig.suptitle('Centroidal info for dyn (-) and kin (--)')
        fig.tight_layout(rect=[0, 0, 1., 0.95])
        plt.show()

        return fig, axes


    def replay_kinematics(self, start=0, end=None, viz="meshcat"):
        if (viz=="meshcat"):
            viz = self.kin_optimizer.robot.initMeshcat()
            viz.loadViewerModel()
            print("Replay the kinematics using Meshcat!")
        elif (viz=="gepetto"):
            try:
                self.kin_optimizer.robot.ensureDisplay()
                viz = self.kin_optimizer.robot
                print("Replay the kinematics using geppeto_viewer!")
            except:
                "Check whether gepetto-viewer is properly started"
        else:
            print ("You need to specify either meshcat or gepetto as visualizer...")

        try:
            for ks in self.kin_optimizer.kinematics_sequence.kinematics_states[start:end]:
                q = ks.robot_posture.generalized_joint_positions
                viz.display(np.matrix(q).T)
                time.sleep(self.kin_optimizer.dt)
        except:
            "Check whether gepetto-viewer is properly started"


    def plot_base_trajecory(self, start=0, end=None, plot_show=True):
        fig, axes = plt.subplots(3, 1, figsize=(16, 8), sharex=True)
        q_app = np.zeros([1,self.kin_optimizer.robot.model.nq])
        for ks in self.kin_optimizer.kinematics_sequence.kinematics_states[start:end]:
            q = ks.robot_posture.generalized_joint_positions
            q_app = np.append(q_app,q.reshape(1,len(q)),axis=0)

        for i, ylabel in enumerate(["x", "y", "z"]):
            axes[i].plot(q_app[1:end,i])
            axes[i].set_ylabel(ylabel + ' [m]')
            axes[i].grid(True)

        axes[2].set_xlabel('time steps')
        axes[0].set_title('Base reference point trajectory')
        self._plot_show(plot_show)


    def plot_joint_trajecory(self, start=0, end=None,
                             plot_show=True, fig_suptitle=''):

        n_eff = np.size(self.kin_optimizer.robot.effs)
        n_joints = np.size(self.kin_optimizer.robot.joints_list[:-1])
        fig, axes = plt.subplots(n_eff,
                                 n_joints,
                                 figsize=(16, 10), sharex=True)

        q_app = np.zeros([1,self.kin_optimizer.robot.model.nq])
        for ks in self.kin_optimizer.kinematics_sequence.kinematics_states[start:end]:
            q = ks.robot_posture.generalized_joint_positions
            q_app = np.append(q_app,q.reshape(1,len(q)),axis=0)

        for i in range(1):#n_eff):
            for j in range(n_joints):
                axes[i,j].plot(q_app[1:end,i*n_joints+j+7], label = "act")
                #kin_optimizer.joint_des[i*n_joints+j,:]
                axes[i,j].plot(self.kin_optimizer.q_kin[:,i*n_joints+j+7], label = "des")
                axes[i,j].grid(True)
                

        for i, label in enumerate(self.kin_optimizer.robot.effs):
            axes[i, 0].set_ylabel(label+ ' [rad]')
        for j, title in enumerate(self.kin_optimizer.robot.joints_list[:-1]):
            axes[n_eff-1, j].set_xlabel('time steps')
            axes[0, j].set_title(title)

        axes[0,n_joints-1].legend()
        fig.suptitle('Desired and actual joint trajectories')
        self._plot_show(plot_show)


    def plot_foot_traj(self, plot_show=True):
        fig, axes = plt.subplots(np.size(self.kin_optimizer.robot.effs), 3, figsize=(16, 8), sharex=True)
        des_ee_traj = EndeffectorTrajectoryGenerator()
        des_ee_traj.z_offset = self.planner_setting.get(PlannerDoubleParam_SwingTrajViaZ)
        des_ee_pos = des_ee_traj(self.kin_optimizer)[0]
        foot_traj = self.kin_optimizer.motion_eff['trajectory']

        for i in range(np.size(self.kin_optimizer.robot.effs)):
            for j in range(3):
                axes[i,j].plot(foot_traj[:,3*i+j], label = "act")
                axes[i,j].plot(des_ee_pos[:,i,j], label = "des")
                axes[i,j].grid(True)

        for i, label in enumerate(self.kin_optimizer.robot.effs):
            axes[i, 0].set_ylabel(label+ ' [m]')
        for j, title in enumerate(['x', 'y', 'z']):
            axes[np.size(self.kin_optimizer.robot.effs)-1, j].set_xlabel('time steps')
            axes[0, j].set_title(title)

        axes[0, 2].legend()
        fig.suptitle('Desired and actual feet trajectories')
        self._plot_show(plot_show)


    def plot_com_motion(self, dynamics_states, kinematics_states,
            plot_show=True, fig_suptitle=''):
        fig, axes = plt.subplots(3, 3, figsize=(12, 8), sharex=True)
        axes = np.array(axes)

        def states_to_vec(states):
            com = np.vstack([s.com for s in states])
            lmom = np.vstack([s.lmom/95.941282 for s in states])
            amom = np.vstack([s.amom for s in states])
            return com, lmom, amom

        for i, (title, dyn, kin) in enumerate(zip(
            ['com', 'lmom', 'amom'],
            states_to_vec(dynamics_states),
            states_to_vec(kinematics_states))):

            axes[0, i].set_title(title)

            for j in range(3):
                axes[j, i].plot(dyn[:, j], label='dynamic')
                axes[j, i].plot(kin[:, j], label='kinematic')

        [ax.grid(True) for ax in axes.reshape(-1)]

        for i, label in enumerate(['x', 'y', 'z']):
            axes[i, 0].set_ylabel(label + ' [m]')
            axes[2, i].set_xlabel('time steps [5ms]')

        axes[0, 2].legend()

        if fig_suptitle:
            fig.suptitle(fig_suptitle)

        self._plot_show(plot_show)


    def save_files(self):
        time_vector = create_time_vector(self.dyn_optimizer.dynamicsSequence())
        create_file(time_vector,
                self.kin_optimizer.kinematics_sequence,
                self.dyn_optimizer.dynamicsSequence(),
                self.dynamics_feedback,
                self.planner_setting.get(PlannerDoubleParam_RobotWeight),
                )

        self.with_lqr = False
        if self.with_lqr:
            create_lqr_files(time_vector,
                             self.kin_optimizer.motion_eff,
                             self.kin_optimizer.kinematics_sequence,
                             self.dyn_optimizer.dynamicsSequence(),
                             self.dynamics_feedback,
                             self.planner_setting.get(PlannerDoubleParam_RobotWeight))


    def save_qp_files(self):
        time_vector = create_time_vector(self.dyn_optimizer.dynamicsSequence())
        create_qp_files(time_vector,
                    self.kin_optimizer.motion_eff,
                    self.kin_optimizer.kinematics_sequence,
                    self.dyn_optimizer.dynamicsSequence(),
                    self.dynamics_feedback,
                    self.planner_setting.get(PlannerDoubleParam_RobotWeight))


    def time_vector(self):
        return create_time_vector(self.dyn_optimizer.dynamicsSequence())


    def optimize_motion(self, plot_com_motion=True):
        dyn_optimizer = self.dyn_optimizer
        kin_optimizer = self.kin_optimizer

        self.optimize_dynamics(0)
        bool_iter = True
        kd_iter = 0
        for kd_iter in range(0, self.planner_setting.get(PlannerIntParam_KinDynIterations)):
            self.optimize_kinematics(kd_iter + 1, plotting=False)
            self.optimize_dynamics(kd_iter + 1)
            optimized_kin_plan = self.kin_optimizer.kinematics_sequence
            optimized_dyn_plan = self.dyn_optimizer.dynamicsSequence()
            
            kd_iter = kd_iter + 1
        
        optimized_kin_plan = kin_optimizer.kinematics_sequence
        optimized_dyn_plan = dyn_optimizer.dynamicsSequence()

        init_pos = np.hstack([optimized_kin_plan.kinematics_states[0].robot_posture.generalized_joint_positions, optimized_kin_plan.kinematics_states[0].robot_velocity.generalized_joint_velocities, self.ini_state.com[0], self.ini_state.lmom[0]/95.941282, self.ini_state.zmp[0], self.ini_state.amom[1], self.ini_state.com[1],  self.ini_state.lmom[1]/95.941282, self.ini_state.zmp[1],self.ini_state.amom[0]])
        init_pos = init_pos.reshape(1,45)

        x_pos = np.zeros((self.planner_setting.get(PlannerIntParam_NumTimesteps), 45))
        u_pos = np.zeros((self.planner_setting.get(PlannerIntParam_NumTimesteps), 22))
        for i in range(0, self.planner_setting.get(PlannerIntParam_NumTimesteps)):
            x_stack = np.hstack([optimized_kin_plan.kinematics_states[i].robot_posture.generalized_joint_positions, optimized_kin_plan.kinematics_states[i].robot_velocity.generalized_joint_velocities, optimized_dyn_plan.dynamics_states[i].com[0], optimized_dyn_plan.dynamics_states[i].lmom[0]/95.941282,optimized_dyn_plan.dynamics_states[i].zmp[0],  optimized_dyn_plan.dynamics_states[i].amom[1],optimized_dyn_plan.dynamics_states[i].com[1], optimized_dyn_plan.dynamics_states[i].lmom[1]/95.941282, optimized_dyn_plan.dynamics_states[i].zmp[1], optimized_dyn_plan.dynamics_states[i].amom[0]])
            x_stack = x_stack.reshape(1,45)
            x_pos[i] = x_stack

        for i in range(0, self.planner_setting.get(PlannerIntParam_NumTimesteps)):
            if i == 0:
                u_stack = np.hstack([optimized_kin_plan.kinematics_states[i].robot_velocity.generalized_joint_velocities / self.planner_setting.get(PlannerDoubleParam_TimeStep), (optimized_dyn_plan.dynamics_states[i].zmp[0] - self.ini_state.zmp[0]) / self.planner_setting.get(PlannerDoubleParam_TimeStep),  optimized_dyn_plan.dynamics_states[i].amomd[1],  (optimized_dyn_plan.dynamics_states[i].zmp[1] - self.ini_state.zmp[1]) / self.planner_setting.get(PlannerDoubleParam_TimeStep), optimized_dyn_plan.dynamics_states[i].amomd[0]])
            else:
                u_stack = np.hstack([(optimized_kin_plan.kinematics_states[i].robot_velocity.generalized_joint_velocities - optimized_kin_plan.kinematics_states[i-1].robot_velocity.generalized_joint_velocities) / self.planner_setting.get(PlannerDoubleParam_TimeStep), (optimized_dyn_plan.dynamics_states[i].zmp[0] - optimized_dyn_plan.dynamics_states[i-1].zmp[0]) / self.planner_setting.get(PlannerDoubleParam_TimeStep),  optimized_dyn_plan.dynamics_states[i].amomd[1],  (optimized_dyn_plan.dynamics_states[i].zmp[1] - optimized_dyn_plan.dynamics_states[i-1].zmp[1]) / self.planner_setting.get(PlannerDoubleParam_TimeStep), optimized_dyn_plan.dynamics_states[i].amomd[0]])
            u_stack = u_stack.reshape(1,22)
            u_pos[i] = u_stack

        self.crocs_data['Right']['x_inputs'].append(copy(init_pos))
        self.crocs_data['Right']['x_state'].append(copy(x_pos))
        self.crocs_data['Right']['u_trajs'].append(copy(u_pos))
        with open('data.pickle', 'wb') as f:
            pickle.dump(self.crocs_data, f, pickle.HIGHEST_PROTOCOL)
        np.savetxt("hat.dat", x_pos)
        
        time_vector = create_time_vector(dyn_optimizer.dynamicsSequence())
        self.with_lqr = False
        if self.with_lqr:
            self.optimize_dynamics_feedback()
        return optimized_kin_plan, kin_optimizer.motion_eff, \
                optimized_dyn_plan, self.dynamics_feedback, \
                self.planner_setting, time_vector

    def save_files1(self, kd_iter):
        time_vector = create_time_vector(self.dyn_optimizer.dynamicsSequence())
        create_file1(time_vector, self.ini_state, self.kin_optimizer.kinematics_sequence, self.dyn_optimizer.dynamicsSequence(), self.dynamics_feedback, self.planner_setting.get(PlannerDoubleParam_RobotWeight), kd_iter)
        self.with_lqr = False
        if self.with_lqr:
            create_lqr_files1(time_vector,
                             self.kin_optimizer.motion_eff,
                             self.kin_optimizer.kinematics_sequence,
                             self.dyn_optimizer.dynamicsSequence(),
                             self.dynamics_feedback,
                             self.planner_setting.get(PlannerDoubleParam_RobotWeight), kd_iter)

    