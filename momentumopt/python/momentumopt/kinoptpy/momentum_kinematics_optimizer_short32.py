'''
@file momentum_kinematics_optimizer.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

import os
import numpy as np

from momentumopt.kinoptpy.qp import QpSolver
from momentumopt.kinoptpy.inverse_kinematics import PointContactInverseKinematics
from momentumopt.kinoptpy.second_order_ik import SecondOrderInverseKinematics
from pinocchio import RobotWrapper
import pinocchio as se3
from pinocchio.utils import zero
from pymomentum import *

from momentumopt.robots.blmc_robot_wrapper import QuadrupedWrapper
from momentumopt.kinoptpy.min_jerk_traj import *
import math
from pymomentum import \
    PlannerVectorParam_KinematicDefaultJointPositions, \
    PlannerIntParam_NumTimesteps, \
    PlannerDoubleParam_TimeStep

class EndeffectorTrajectoryGenerator(object):
    def __init__(self):
        self.z_offset = 0.1

    def get_z_bound(self, mom_kin_optimizer):
        z_max = min(max(mom_kin_optimizer.com_dyn[:, 2]), self.max_bound)
        z_min = max(min(mom_kin_optimizer.com_dyn[:, 2]), self.min_bound)
        return z_max, z_min

    def is_end_eff_in_contact(self, it, eff, mom_kin_optimizer):
        if mom_kin_optimizer.dynamic_sequence.dynamics_states[it].effActivation(eff):
            endeff_contact = 1.
        else:
            endeff_contact = 0.
        return endeff_contact

    def get_contact_plan_from_dyn_optimizer(self, mom_kin_optimizer):
        contact_plan = {}
        print("contact")
        for i, eff in enumerate(mom_kin_optimizer.eff_names):
            contact_plan[eff] = []
            start_time = 0.
            count = 1
            First_contact = 0
            for it in range(mom_kin_optimizer.num_time_steps-1):
                current_contact_activation = mom_kin_optimizer.dynamic_sequence.dynamics_states[it].effActivation(i)
                last_contact_activation = mom_kin_optimizer.dynamic_sequence.dynamics_states[it+1].effActivation(i)
                
                if eff == 'LF_contact':
                    if current_contact_activation == True and First_contact == 0:
                        First_contact = 1
                        start_time = it
                    if First_contact == 1:
                        First_contact = 2  
                    if current_contact_activation == False and First_contact == 2:
                        end_time = it - 1
                        First_contact = 3
                        plan = [start_time, end_time, mom_kin_optimizer.dynamic_sequence.dynamics_states[it-1].eff(i)]
                        First_contact = 3
                        contact_plan[eff].append(plan)  
                    if First_contact == 3 and current_contact_activation == True:
                        First_contact = 4
                        start_time = it
                    if First_contact == 4 and it == mom_kin_optimizer.num_time_steps-2:
                        end_time = it
                        plan = [start_time, end_time, mom_kin_optimizer.dynamic_sequence.dynamics_states[it-1].eff(i)]
                        First_contact = 5
                        contact_plan[eff].append(plan)  

                if eff == 'RF_contact' :
                    if not current_contact_activation == last_contact_activation:
                        if (count%2 == 0):
                            start_time = it+1
                        elif (count%2 == 1 or it == mom_kin_optimizer.num_time_steps-1):
                            end_time = it
                            plan = [start_time, end_time, mom_kin_optimizer.dynamic_sequence.dynamics_states[it].eff(i)]
                            print("plan")
                            print(plan)
                            contact_plan[eff].append(plan)
                        count += 1
            print("conta")
            print(contact_plan)    
        return contact_plan

    def quinticSpline(self, time, time_0, time_f, x_0, x_dot_0, x_ddot_0, x_f, x_dot_f, x_ddot_f):
        time_s = time_f - time_0
        a1 = x_0
        a2 = x_dot_0
        a3 = x_ddot_0 / 2.0

        Temp = np.zeros((3,3))
        R_temp = np.zeros(3)

        Temp[0,0] = math.pow(time_s, 3)
        Temp[0,1] = math.pow(time_s, 4)
        Temp[0,2] = math.pow(time_s, 5)
        Temp[1,0] = 3.0 * math.pow(time_s, 2)
        Temp[1,1] = 4.0 * math.pow(time_s, 3)
        Temp[1,2] = 5.0 * math.pow(time_s, 4)
        Temp[2,0] = 6.0 * time_s
        Temp[2,1] = 12.0 * math.pow(time_s, 2)
        Temp[2,2] = 20.0 * math.pow(time_s, 3)

        R_temp[0] = x_f - x_0 - x_dot_0 * time_s - x_ddot_0 * math.pow(time_s, 2) / 2.0
        R_temp[1] = x_dot_f - x_dot_0 - x_ddot_0 * time_s
        R_temp[2] = x_ddot_f - x_ddot_0
        
        RES = np.matmul(np.linalg.inv(Temp), R_temp)

        a4 = RES[0]
        a5 = RES[1]
        a6 = RES[2]

        time_fs = time - time_0

        position = a1 + a2 * math.pow(time_fs, 1) + a3 * math.pow(time_fs, 2) + a4 * math.pow(time_fs, 3) + a5 * math.pow(time_fs, 4) + a6 * math.pow(time_fs, 5)
        
        result = position

        if time < time_0:
            result = x_0
        elif time > time_f:
            result = x_f

        return result

    def quinticSplineDot(self, time, time_0, time_f, x_0, x_dot_0, x_ddot_0, x_f, x_dot_f, x_ddot_f):
        time_s = time_f - time_0
        a1 = x_0
        a2 = x_dot_0
        a3 = x_ddot_0 / 2.0

        Temp = np.zeros((3,3))
        R_temp = np.zeros(3)

        Temp[0,0] = math.pow(time_s, 3)
        Temp[0,1] = math.pow(time_s, 4)
        Temp[0,2] = math.pow(time_s, 5)
        Temp[1,0] = 3.0 * math.pow(time_s, 2)
        Temp[1,1] = 4.0 * math.pow(time_s, 3)
        Temp[1,2] = 5.0 * math.pow(time_s, 4)
        Temp[2,0] = 6.0 * time_s
        Temp[2,1] = 12.0 * math.pow(time_s, 2)
        Temp[2,2] = 20.0 * math.pow(time_s, 3)

        R_temp[0] = x_f - x_0 - x_dot_0 * time_s - x_ddot_0 * math.pow(time_s, 2) / 2.0
        R_temp[1] = x_dot_f - x_dot_0 - x_ddot_0 * time_s
        R_temp[2] = x_ddot_f - x_ddot_0

        RES = np.matmul(np.linalg.inv(Temp), R_temp)

        a4 = RES[0]
        a5 = RES[1]
        a6 = RES[2]

        time_fs = time - time_0

        position = a1 + a2 * math.pow(time_fs, 1) + a3 * math.pow(time_fs, 2) + a4 * math.pow(time_fs, 3) + a5 * math.pow(time_fs, 4) + a6 * math.pow(time_fs, 5)
        velocity = a2 + 2.0 * a3 * math.pow(time_fs, 1) + 3.0 * a4 * math.pow(time_fs, 2) + 4.0 * a5 * math.pow(time_fs, 3) + 5.0 * a6 * math.pow(time_fs, 4);
        
        result = velocity

        if time < time_0:
            result = x_dot_0
        elif time > time_f:
            result = x_dot_f

        return result


    def generate_eff_traj(self, mom_kin_optimizer, contact_plan):
        eff_traj_poly = {}
        for eff in mom_kin_optimizer.eff_names:
            num_contacts = len(contact_plan[eff])
            poly_traj = [PolynominalList(), PolynominalList(), PolynominalList()]
            for i in range(num_contacts):
                # Create a constant polynominal for endeffector on the ground.
                t = [contact_plan[eff][i][0], contact_plan[eff][i][1]]
                for idx in range(3):
                    poly_traj[idx].append(t, constant_poly(contact_plan[eff][i][2][idx]))

                # If there is a contact following, add the transition between
                # the two contact points.
                if i < num_contacts - 1:
                    t = [contact_plan[eff][i][1], contact_plan[eff][i+1][0]]
                    for idx in range(3):
                        via = None
                        if idx == 2:
                            via = self.z_offset + contact_plan[eff][i][2][idx]
                        poly = poly_points(t, contact_plan[eff][i][2][idx], contact_plan[eff][i+1][2][idx], via)
                        poly_traj[idx].append(t,poly)

            eff_traj_poly[eff] = poly_traj
        # returns end eff trajectories
        
        return eff_traj_poly

    def generate_eff_traj1(self, mom_kin_optimizer, contact_plan):
        eff_traj_poly = {}
        for eff in mom_kin_optimizer.eff_names:
            num_contacts = len(contact_plan[eff])
            poly_traj = [PolynominalList(), PolynominalList(), PolynominalList()]
            for i in range(num_contacts):
                # Create a constant polynominal for endeffector on the ground.
                t = [contact_plan[eff][i][0], contact_plan[eff][i][1]]
                for idx in range(3):
                    poly_traj[idx].append(t, constant_poly(contact_plan[eff][i][2][idx]))
                
                # If there is a contact following, add the transition between
                # the two contact points.
                if i < num_contacts - 1:
                    t = [contact_plan[eff][i][1], contact_plan[eff][i+1][0]]
                    t1 = [contact_plan[eff][i][0], 0]
                    
                    for idx in range(3):
                        via = None
                        if idx == 2:
                            via = self.z_offset + contact_plan[eff][i][2][idx]
                    if i == 0:
                        if eff == 'RF_contact':
                            poly = poly_points1(t, contact_plan[eff][i][2][0], contact_plan[eff][i+1][2][0], contact_plan[eff][i][2][1], contact_plan[eff][i+1][2][1], contact_plan[eff][i][2][2], contact_plan[eff][i+1][2][2], t1, via)
                        else:
                            poly1 = poly_points1(t, contact_plan[eff][i][2][0], contact_plan[eff][i+1][2][0], contact_plan[eff][i][2][1], contact_plan[eff][i+1][2][1], contact_plan[eff][i][2][2], contact_plan[eff][i+1][2][2], t1, via)
                    else:
                        if eff == 'RF_contact':
                            temp = poly_points1(t, contact_plan[eff][i][2][0], contact_plan[eff][i+1][2][0], contact_plan[eff][i][2][1], contact_plan[eff][i+1][2][1], contact_plan[eff][i][2][2], contact_plan[eff][i+1][2][2], t1, via)
                        else:
                            temp1 = poly_points1(t, contact_plan[eff][i][2][0], contact_plan[eff][i+1][2][0], contact_plan[eff][i][2][1], contact_plan[eff][i+1][2][1], contact_plan[eff][i][2][2], contact_plan[eff][i+1][2][2], t1, via)
                    if i != 0:
                        if eff == 'RF_contact':
                            poly = np.append(poly, temp[0:6])
                        else:
                            poly1 = np.append(poly1, temp1[0:6])
                    '''   
                    if i == num_contacts - 2:
                        if eff == 'RF_contact':
                            poly = np.reshape(poly,(12,5))
                        else:
                            poly1 = np.reshape(poly1,(12,5))
                    
                    '''
                if(num_contacts == 1):
                    t1 = [contact_plan[eff][i][0], contact_plan[eff][i][1]]
                    if eff == 'RF_contact':
                        poly = np.array(poly_points1(t, contact_plan[eff][i][2][0], contact_plan[eff][i][2][0], contact_plan[eff][i][2][1], contact_plan[eff][i][2][1], contact_plan[eff][i][2][2], contact_plan[eff][i][2][2], t1, 0.0))
                    
                    else:
                        poly1 = np.array(poly_points1(t, contact_plan[eff][i][2][0], contact_plan[eff][i][2][0], contact_plan[eff][i][2][1], contact_plan[eff][i][2][1], contact_plan[eff][i][2][2], contact_plan[eff][i][2][2], t1, 0.0))
     
        return poly, poly1


    def __call__(self, mom_kin_optimizer):
        '''
        Computes the endeffector positions and velocities.

        Returns endeff_pos_ref, endeff_vel_ref
            [0]: endeff_pos_ref: np.array, shape=[num_time_steps, num_eff, 3={x, y, z}]
            [1]: endeff_vel_ref: np.array, shape=[num_time_steps, num_eff, 3={x, y, z}]
        '''
        dt = mom_kin_optimizer.dt
        num_eff = len(mom_kin_optimizer.eff_names)
        
        num_time_steps = mom_kin_optimizer.num_time_steps
        
        contact_plan = self.get_contact_plan_from_dyn_optimizer(mom_kin_optimizer)

        # Generate minimum jerk trajectories
        print("contact")
        print(contact_plan)
        eff_traj_poly, eff_traj_poly1 = self.generate_eff_traj1(mom_kin_optimizer, contact_plan)

        # Compute the endeffector position and velocity trajectories.
        endeff_pos_ref = np.zeros((num_time_steps, num_eff, 3))
        endeff_pos_ref1 = np.zeros((num_eff, num_time_steps, 3))
        endeff_vel_ref = np.zeros((num_time_steps, num_eff, 3))
        endeff_contact = np.zeros((num_time_steps, num_eff))
        
        num_step = 0
        num_step1 = 0
        print(eff_traj_poly)
        print(eff_traj_poly1)#0.1025
        
        for it in range(num_time_steps):
            for eff, name in enumerate(mom_kin_optimizer.eff_names):
                if eff == 1:
                    if(it < eff_traj_poly1[0,0]):
                        x = eff_traj_poly1[0,1]
                        y = eff_traj_poly1[0,2]
                        z = eff_traj_poly1[0,3]
                        xv = 0.0
                        yv = 0.0
                        zv = 0.0
              
                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])
                        
                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])
                else:
                    if(it < eff_traj_poly[0,5]):
                        x = self.quinticSpline(it, int(eff_traj_poly[6* num_step, 5]), int(eff_traj_poly[6*num_step , 5]), -0.0052012, 0.0, 0.0, eff_traj_poly[6*num_step , 1] + 0.4, eff_traj_poly[6* num_step  + 1, 1], eff_traj_poly[6*num_step + 2, 1])
                        xv = self.quinticSplineDot(it, int(eff_traj_poly[6* num_step, 5]), int(eff_traj_poly[6*num_step , 5]), -0.0052012, 0.0, 0.0, eff_traj_poly[6*num_step , 1] + 0.4, eff_traj_poly[6* num_step + 1 , 1], eff_traj_poly[6*num_step , 1])
                        y = self.quinticSpline(it, int(eff_traj_poly[6* num_step, 5]), int(eff_traj_poly[6*num_step , 5]), 0.1025, 0.0, 0.0, eff_traj_poly[6*num_step , 2], eff_traj_poly[6* num_step + 1, 2], eff_traj_poly[6*num_step + 2, 2])
                        yv = self.quinticSplineDot(it, int(eff_traj_poly[6* num_step, 5]), int(eff_traj_poly[6*num_step, 5]), 0.1025, 0.0, 0.0, eff_traj_poly[6*num_step , 2], eff_traj_poly[6* num_step + 1, 2], eff_traj_poly[6*num_step + 2, 2])

                        if (it <= (int(eff_traj_poly[6* num_step, 6]) + int(eff_traj_poly[6*num_step, 0]))/2):
                            z = self.quinticSpline(it, int(eff_traj_poly[6* num_step, 0]), (int(eff_traj_poly[6* num_step, 6]) + int(eff_traj_poly[6*num_step , 0]))/2, 0.0, 0.0, 0.0, eff_traj_poly[6*num_step , 3] + 0.03, eff_traj_poly[6* num_step + 1 , 3], eff_traj_poly[6*num_step + 2, 3])
                            zv = self.quinticSplineDot(it, int(eff_traj_poly[6* num_step, 0]), (int(eff_traj_poly[6* num_step, 6]) + int(eff_traj_poly[6*num_step , 0]))/2, 0.0, 0.0, 0.0, eff_traj_poly[6*num_step , 3] + 0.03, eff_traj_poly[6* num_step + 1 , 3], eff_traj_poly[6*num_step + 2, 3])
                        else:
                            z = self.quinticSpline(it, (int(eff_traj_poly[6* num_step, 0]) + int(eff_traj_poly[6*num_step , 6]))/2, int(eff_traj_poly[6*num_step , 5]), 0.0, 0.0, 0.0, eff_traj_poly[6*num_step , 3], eff_traj_poly[6* num_step  + 1, 3], eff_traj_poly[6*num_step + 2, 3])
                            zv = self.quinticSplineDot(it, (int(eff_traj_poly[6* num_step, 0]) + int(eff_traj_poly[6*num_step , 6]))/2, int(eff_traj_poly[6*num_step , 0]), 0.0, 0.0, 0.0, eff_traj_poly[6*num_step , 3], eff_traj_poly[6* num_step  + 1, 3], eff_traj_poly[6*num_step + 2, 3])

                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])
                        
                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])
                    elif (it < eff_traj_poly[0,0]):
                        x = eff_traj_poly[0,1]
                        y = eff_traj_poly[0,2]
                        z = eff_traj_poly[0,3]
                        xv = 0.0
                        yv = 0.0
                        zv = 0.0
              
                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])
                        
                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])

                
                if eff == 1:
                    if it < eff_traj_poly1[0,0]:
                        if(it <= eff_traj_poly1[6*num_step + 3, 0]):
                            num_step = num_step
                        else:
                            if (num_step+1) * 6 < eff_traj_poly1.shape[0]:
                                num_step = num_step + 1
                        x = self.quinticSpline(it, int(eff_traj_poly1[6* num_step, 0]), int(eff_traj_poly1[6*0 + 3, 0]), eff_traj_poly1[6* 0, 1], eff_traj_poly1[6*0 + 1, 1], eff_traj_poly1[6* 0 + 2, 1], eff_traj_poly1[6*0 + 3, 1], eff_traj_poly1[6* 0 + 4, 1], eff_traj_poly1[6*0 + 5, 1])
                        xv = self.quinticSplineDot(it, int(eff_traj_poly1[6* num_step, 0]), int(eff_traj_poly1[6*0 + 3, 0]), eff_traj_poly1[6* 0, 1], eff_traj_poly1[6*0 + 1, 1], eff_traj_poly1[6* 0 + 2, 1], eff_traj_poly1[6*0 + 3, 1], eff_traj_poly1[6* 0 + 4, 1], eff_traj_poly1[6*0 + 5, 1])
                        y = self.quinticSpline(it, int(eff_traj_poly1[6* num_step, 0]), int(eff_traj_poly1[6*0 + 3, 0]), eff_traj_poly1[6* 0, 2], eff_traj_poly1[6*0 + 1, 2], eff_traj_poly1[6* 0 + 2, 2], eff_traj_poly1[6*0 + 3, 2], eff_traj_poly1[6* 0 + 4, 2], eff_traj_poly1[6*0 + 5, 2])
                        yv = self.quinticSplineDot(it, int(eff_traj_poly1[6* num_step, 0]), int(eff_traj_poly1[6*0 + 3, 0]), eff_traj_poly1[6* 0, 2], eff_traj_poly1[6*0 + 1, 2], eff_traj_poly1[6* 0 + 2, 2], eff_traj_poly1[6*0 + 3, 2], eff_traj_poly1[6* 0 + 4, 2], eff_traj_poly1[6*0 + 5, 2])
                    
                        if (it <= (int(eff_traj_poly1[6* num_step, 0]) + int(eff_traj_poly1[6*num_step + 3, 0]))/2):
                            z = 0.0#self.quinticSpline(it, int(eff_traj_poly1[6* num_step, 0]), (int(eff_traj_poly1[6* num_step, 0]) + int(eff_traj_poly1[6*num_step + 3, 0]))/2, eff_traj_poly1[6* num_step, 3], eff_traj_poly1[6*num_step + 1, 3], eff_traj_poly1[6* num_step + 2, 3], eff_traj_poly1[6*num_step + 3, 3] + 0.03, eff_traj_poly1[6* num_step + 4, 3], eff_traj_poly1[6*num_step + 5, 3])
                            zv = 0.0#self.quinticSplineDot(it, int(eff_traj_poly1[6* num_step, 0]), (int(eff_traj_poly1[6* num_step, 0]) + int(eff_traj_poly1[6*num_step + 3, 0]))/2, eff_traj_poly1[6* num_step, 3], eff_traj_poly1[6*num_step + 1, 3], eff_traj_poly1[6* num_step + 2, 3], eff_traj_poly1[6*num_step + 3, 3] + 0.03, eff_traj_poly1[6* num_step + 4, 3], eff_traj_poly1[6*num_step + 5, 3])
                        else:
                            z = 0.0#self.quinticSpline(it, (int(eff_traj_poly1[6* num_step, 0]) + int(eff_traj_poly1[6*num_step + 3, 0]))/2+ 1, int(eff_traj_poly1[6*num_step + 3, 0]), eff_traj_poly1[6* num_step, 3] + 0.03, eff_traj_poly1[6*num_step + 1, 3], eff_traj_poly[6* num_step + 2, 3], eff_traj_poly[6*num_step + 3, 3], eff_traj_poly[6* num_step + 4, 3], eff_traj_poly[6*num_step + 5, 3])
                            zv = 0.0#self.quinticSplineDot(it, (int(eff_traj_poly[6* num_step, 0]) + int(eff_traj_poly[6*num_step + 3, 0]))/2+ 1, int(eff_traj_poly[6*num_step + 3, 0]), eff_traj_poly[6* num_step, 3] + 0.03, eff_traj_poly[6*num_step + 1, 3], eff_traj_poly[6* num_step + 2, 3] , eff_traj_poly[6*num_step + 3, 3], eff_traj_poly[6* num_step + 4, 3], eff_traj_poly[6*num_step + 5, 3])
                    
                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])
                    elif it >= eff_traj_poly1[0,0]:
                        num_step = 0
                        x = self.quinticSpline(it, int(eff_traj_poly1[0, 0]), int(eff_traj_poly1[3, 0]), eff_traj_poly1[0, 1], eff_traj_poly1[6*0 + 1, 1], eff_traj_poly1[6* 0 + 2, 1], eff_traj_poly1[ 3, 1], eff_traj_poly1[6* 0 + 4, 1], eff_traj_poly1[6*0 + 5, 1])
                        xv = self.quinticSplineDot(it, int(eff_traj_poly1[0, 0]), int(eff_traj_poly1[3, 0]), eff_traj_poly1[0, 1], eff_traj_poly1[6*0 + 1, 1], eff_traj_poly1[6* 0 + 2, 1], eff_traj_poly1[3, 1] , eff_traj_poly1[6* 0 + 4, 1], eff_traj_poly1[6*0 + 3, 1])
                        y = self.quinticSpline(it, int(eff_traj_poly1[0, 0]), int(eff_traj_poly1[3, 0]), eff_traj_poly1[0, 2], eff_traj_poly1[6*0 + 1, 2], eff_traj_poly1[6* 0 + 2, 2], eff_traj_poly1[3, 2], eff_traj_poly1[6* 0 + 4, 2], eff_traj_poly1[6*0 + 5, 2])
                        yv = self.quinticSplineDot(it, int(eff_traj_poly1[0, 0]), int(eff_traj_poly1[3, 0]), eff_traj_poly1[0, 2], eff_traj_poly1[6*0 + 1, 2], eff_traj_poly1[6* 0 + 2, 2], eff_traj_poly1[3, 2], eff_traj_poly1[6* 0 + 4, 2], eff_traj_poly1[6*0 + 5, 2])
                        if (it <= (int(eff_traj_poly1[3, 0]) + int(eff_traj_poly1[0, 0]))/2):
                            z = self.quinticSpline(it, int(eff_traj_poly1[0, 0]), (int(eff_traj_poly1[0, 0])  + int(eff_traj_poly1[3, 0]))/2, eff_traj_poly1[6* num_step, 3], eff_traj_poly1[6*num_step + 1, 3], eff_traj_poly1[6* num_step + 2, 3], eff_traj_poly1[6*num_step + 0, 3] + 0.03, eff_traj_poly1[6* num_step + 4, 3], eff_traj_poly1[6*num_step + 5, 3])
                            zv = self.quinticSplineDot(it, int(eff_traj_poly1[0, 0]) , (int(eff_traj_poly1[0, 0]) + int(eff_traj_poly1[3, 0]))/2, eff_traj_poly1[6* num_step, 3], eff_traj_poly1[6*num_step + 1, 3], eff_traj_poly1[6* num_step + 2, 3], eff_traj_poly1[6*num_step + 0, 3] + 0.03, eff_traj_poly1[6* num_step + 4, 3], eff_traj_poly1[6*num_step + 5, 3])
                        else:
                            z = self.quinticSpline(it, (int(eff_traj_poly1[0, 0])  + int(eff_traj_poly1[3, 0]))/2, int(eff_traj_poly1[6*num_step + 3, 0]), eff_traj_poly1[6* num_step, 3] + 0.03, eff_traj_poly1[6*num_step + 1, 3], eff_traj_poly1[6* num_step + 2, 3], eff_traj_poly1[6*num_step + 3, 3], eff_traj_poly1[6* num_step + 4, 3], eff_traj_poly1[6*num_step + 5, 3])
                            zv = self.quinticSplineDot(it, (int(eff_traj_poly1[0, 0])  + int(eff_traj_poly1[3, 0]))/2, int(eff_traj_poly1[6*num_step + 3, 0]), eff_traj_poly1[6* num_step, 3] + 0.03, eff_traj_poly1[6*num_step + 1, 3], eff_traj_poly1[6* num_step + 2, 3] , eff_traj_poly1[6*num_step + 3, 3], eff_traj_poly1[6* num_step + 4, 3], eff_traj_poly1[6*num_step + 5, 3])
                    
                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])
                else:
                    if it >= eff_traj_poly[0,0]:
                        if(it <= eff_traj_poly[6*num_step1 + 3, 0]) or num_step * 6 < eff_traj_poly.shape[0]:
                            num_step1 = num_step1
                        else:
                            if (num_step1+1) * 6 < eff_traj_poly.shape[0]:
                                num_step1 = num_step1 + 1

                        x = self.quinticSpline(it, int( eff_traj_poly[6* num_step, 0]), int( eff_traj_poly[6*num_step + 3, 0]),  eff_traj_poly[6* num_step, 1],  eff_traj_poly[6*num_step + 1, 1],  eff_traj_poly[6* num_step + 2, 1],  eff_traj_poly[6*num_step + 3, 1],  eff_traj_poly[6* num_step + 4, 1],  eff_traj_poly[6*num_step + 5, 1])
                        xv = self.quinticSplineDot(it, int( eff_traj_poly[6* num_step, 0]), int( eff_traj_poly[6*num_step + 3, 0]),  eff_traj_poly[6* num_step, 1],  eff_traj_poly[6*num_step + 1, 1],  eff_traj_poly[6* num_step + 2, 1],  eff_traj_poly[6*num_step + 3, 1],  eff_traj_poly[6* num_step + 4, 1],  eff_traj_poly[6*num_step + 3, 1])
                        y = self.quinticSpline(it, int( eff_traj_poly[6* num_step, 0]), int( eff_traj_poly[6*num_step + 3, 0]),  eff_traj_poly[6* num_step, 2],  eff_traj_poly[6*num_step + 1, 2],  eff_traj_poly[6* num_step + 2, 2],  eff_traj_poly[6*num_step + 3, 2],  eff_traj_poly[6* num_step + 4, 2],  eff_traj_poly[6*num_step + 5, 2])
                        yv = self.quinticSplineDot(it, int( eff_traj_poly[6* num_step, 0]), int( eff_traj_poly[6*num_step + 3, 0]),  eff_traj_poly[6* num_step, 2],  eff_traj_poly[6*num_step + 1, 2],  eff_traj_poly[6* num_step + 2, 2],  eff_traj_poly[6*num_step + 3, 2],  eff_traj_poly[6* num_step + 4, 2],  eff_traj_poly[6*num_step + 5, 2])
                        if (it <= (int( eff_traj_poly[6* num_step, 0]) + int( eff_traj_poly[6*num_step + 3, 0]))/2):
                            z = self.quinticSpline(it, int( eff_traj_poly[6* num_step, 0]), (int( eff_traj_poly[6* num_step, 0]) + int( eff_traj_poly[6*num_step + 3, 0]))/2,  eff_traj_poly[6* num_step, 3],  eff_traj_poly[6*num_step + 1, 3],  eff_traj_poly[6* num_step + 2, 3],  eff_traj_poly[6*num_step + 0, 3] + 0.03,  eff_traj_poly[6* num_step + 4, 3],  eff_traj_poly[6*num_step + 5, 3])
                            zv = self.quinticSplineDot(it, int( eff_traj_poly[6* num_step, 0]), (int( eff_traj_poly[6* num_step, 0]) + int( eff_traj_poly[6*num_step + 3, 0]))/2,  eff_traj_poly[6* num_step, 3],  eff_traj_poly[6*num_step + 1, 3],  eff_traj_poly[6* num_step + 2, 3],  eff_traj_poly[6*num_step + 0, 3] + 0.03,  eff_traj_poly[6* num_step + 4, 3],  eff_traj_poly[6*num_step + 5, 3])
                        else:
                            z = self.quinticSpline(it, (int( eff_traj_poly[6* num_step, 0]) + int( eff_traj_poly[6*num_step + 3, 0]))/2+ 1, int( eff_traj_poly[6*num_step + 3, 0]),  eff_traj_poly[6* num_step, 3] + 0.03,  eff_traj_poly[6*num_step + 1, 3],  eff_traj_poly[6* num_step + 2, 3],  eff_traj_poly[6*num_step + 3, 3],  eff_traj_poly[6* num_step + 4, 3],  eff_traj_poly[6*num_step + 5, 3])
                            zv = self.quinticSplineDot(it, (int( eff_traj_poly[6* num_step, 0]) + int( eff_traj_poly[6*num_step + 3, 0]))/2+ 1, int( eff_traj_poly[6*num_step + 3, 0]),  eff_traj_poly[6* num_step, 3] + 0.03,  eff_traj_poly[6*num_step + 1, 3],  eff_traj_poly[6* num_step + 2, 3] ,  eff_traj_poly[6*num_step + 3, 3],  eff_traj_poly[6* num_step + 4, 3],  eff_traj_poly[6*num_step + 5, 3])
                        endeff_pos_ref[it][eff] = np.array([x, y, z])
                        endeff_vel_ref[it][eff] = np.array([xv, yv, zv])
                        endeff_pos_ref1[eff][it] = np.array([x, y, z])
                    
                    
                    '''
                    print("it")
                    print(it)
                    print("num")
                    print(num_step)
                    '''
                    endeff_contact[it][eff] = self.is_end_eff_in_contact(it, eff, mom_kin_optimizer)                     
        
        for eff, name in enumerate(mom_kin_optimizer.eff_names):
            if(endeff_pos_ref[0][eff][0] == endeff_pos_ref[len(endeff_pos_ref)-1][eff][0]):
                for i in range(0, len(endeff_pos_ref)):
                    endeff_pos_ref[i][eff][2] = 0.0
                    endeff_vel_ref[i][eff][2] = 0.0
                    endeff_pos_ref1[eff][i][2] = 0.0

        
        np.savetxt("/home/jhk/walkingdata1/stairdown/25cm/ssp2/timestep=90/lfoot1.txt", endeff_pos_ref1[0])
        np.savetxt("/home/jhk/walkingdata1/stairdown/25cm/ssp2/timestep=90/rfoot1.txt", endeff_pos_ref1[1])
        
        return endeff_pos_ref, endeff_vel_ref, endeff_contact


class TrajectoryInterpolator(object):
    def __init__(self):
        self.num_time_steps = None
        self.q_init = None
        self.init = None
        self.end = None
        self.poly_traj = None

    def generate_trajectory(self, n_via, q_via, dt):
        self.poly_traj = []
        for i in range(len(self.init)):
            self.poly_traj = np.append(self.poly_traj, [PolynominalList()])
        for j in range(len(self.init)):
            for i in range (n_via+1):
                if i==0:
                    t = [0, q_via[0][0]/dt]
                    poly = poly_points(t, self.init[j], q_via[i][j+1])
                    self.poly_traj[j].append(t, poly)
                elif(i==n_via):
                    t = [q_via[i-1][0]/dt, self.num_time_steps]
                    if t[0] != t[1]: # Avoid singular results at the end.
                        poly = poly_points(t, q_via[i-1][j+1], self.end[j])
                        self.poly_traj[j].append(t, poly)
                else:
                    t = [q_via[i-1][0]/dt, q_via[i][0]/dt]
                    poly = poly_points(t, q_via[i-1][j+1], q_via[i][j+1])
                    self.poly_traj[j].append(t, poly)

    def evaluate_trajecory(self,t):
        q = np.zeros((1,len(self.init)),float)
        for j in range(len(self.init)):
            q[0,j] = self.poly_traj[j].eval(t)
        return q


class MomentumKinematicsOptimizer(object):
    def __init__(self):
        self.q_init = None
        self.dq_init = None
        self.reg_orientation = 1e-2
        self.reg_joint_position = 2.
        self.joint_des = None
        self.n_via_joint = 0
        self.n_via_base = 0
        self.via_joint = None
        self.via_base = None

    def reset(self):
        self.kinematics_sequence = KinematicsSequence()
        self.kinematics_sequence.resize(self.planner_setting.get(PlannerIntParam_NumTimesteps)+1,
                                        self.planner_setting.get(PlannerIntParam_NumDofs))

    def initialize(self, planner_setting, max_iterations=50, eps=0.001, endeff_traj_generator=None,
                   RobotWrapper=QuadrupedWrapper):
        self.planner_setting = planner_setting

        if endeff_traj_generator is None:
            endeff_traj_generator = EndeffectorTrajectoryGenerator()
        self.endeff_traj_generator = endeff_traj_generator

        self.dt = planner_setting.get(PlannerDoubleParam_TimeStep)
        self.num_time_steps = planner_setting.get(PlannerIntParam_NumTimesteps) + 1

        self.max_iterations = max_iterations
        self.eps = eps
        self.robot = RobotWrapper()

        self.reset()

        # Holds dynamics and kinematics results
        self.com_dyn = np.zeros((self.num_time_steps, 3))
        self.lmom_dyn = np.zeros((self.num_time_steps, 3))
        self.amom_dyn = np.zeros((self.num_time_steps, 3))

        self.com_kin = np.zeros((self.num_time_steps, 3))
        self.lmom_kin = np.zeros((self.num_time_steps, 3))
        self.amom_kin = np.zeros((self.num_time_steps, 3))
        self.q_kin = np.zeros((self.num_time_steps, self.robot.model.nq))
        self.dq_kin = np.zeros((self.num_time_steps, self.robot.model.nv))
        self.rf_ori_kin = np.zeros((self.num_time_steps, 3))
        self.lf_ori_kin = np.zeros((self.num_time_steps, 3))

        self.hip_names = ['base_link'.format(eff) for eff in self.robot.effs]
        self.hip_ids = [self.robot.model.getFrameId(name) for name in self.hip_names]
        #otherrobot = self.robot.effs#
        self.eff_names = self.robot.effs#= ['{}_{}'.format(eff, self.robot.joints_list[-1]) for eff in self.robot.effs]
        self.q_init = self.robot.q
        print("q")
        print(self.q_init)
        self.dq_init = self.robot.dq
        self.inv_kin = PointContactInverseKinematics(self.robot.model, self.eff_names)
        self.snd_order_inv_kin = SecondOrderInverseKinematics(self.robot.model, self.eff_names)
        self.use_second_order_inv_kin = False

        self.motion_eff = {
            'trajectory': np.zeros((self.num_time_steps, 3 * self.inv_kin.ne)),
            'velocity': np.zeros((self.num_time_steps, 3 * self.inv_kin.ne)),
            'trajectory_wrt_base': np.zeros((self.num_time_steps, 3 * self.inv_kin.ne)),
            'velocity_wrt_base': np.zeros((self.num_time_steps, 3 * self.inv_kin.ne))
        }

    def fill_data_from_dynamics(self):
        # The centroidal information
        for it in range(self.num_time_steps):
          self.com_dyn[it] = self.dynamic_sequence.dynamics_states[it].com
          self.lmom_dyn[it] = self.dynamic_sequence.dynamics_states[it].lmom
          self.amom_dyn[it] = self.dynamic_sequence.dynamics_states[it].amom

    def fill_endeffector_trajectory(self):
        self.endeff_pos_ref, self.endeff_vel_ref, self.endeff_contact = \
                self.endeff_traj_generator(self)

    def fill_kinematic_result(self, it, q, dq, ddq):
        def framesPos(frames):
            
            return np.vstack([data.oMf[idx].translation for idx in frames]).reshape(-1)

        def framesVel(frames):
            return np.vstack([
                    self.inv_kin.get_world_oriented_frame_jacobian(q, idx).dot(dq)[:3] for idx in frames
                ]).reshape(-1)

        data = self.inv_kin.robot.data
        hg = self.inv_kin.robot.centroidalMomentum(q, dq)

        # Storing on the internal array.
        self.com_kin[it] = self.inv_kin.robot.com(q).T
        self.lmom_kin[it] = hg.linear.T
        self.amom_kin[it] = hg.angular.T
        self.q_kin[it] = q.T
        self.dq_kin[it] = dq.T

        # The endeffector informations as well.
        self.motion_eff['trajectory'][it] = framesPos(self.inv_kin.endeff_ids)
        self.motion_eff['velocity'][it] = self.inv_kin.J[6:(self.inv_kin.ne + 2) * 3].dot(dq).T

        self.motion_eff['trajectory_wrt_base'][it] = \
            self.motion_eff['trajectory'][it] - framesPos(self.hip_ids)
        self.motion_eff['velocity_wrt_base'][it] = \
            self.motion_eff['velocity'][it] - framesVel(self.hip_ids)

        # Storing on the kinematic sequence.
        kinematic_state = self.kinematics_sequence.kinematics_states[it]
        kinematic_state.com = self.com_kin[it]
        kinematic_state.lmom = self.lmom_kin[it]
        kinematic_state.amom = self.amom_kin[it]
        kinematic_state.rf_ori = self.rf_ori_kin[it]
        kinematic_state.lf_ori = self.lf_ori_kin[it]

        kinematic_state.robot_posture.base_position = q[:3]
        kinematic_state.robot_posture.base_orientation = q[3:7]
        kinematic_state.robot_posture.joint_positions = q[7:]

        kinematic_state.robot_velocity.base_linear_velocity = dq[:3]
        kinematic_state.robot_velocity.base_angular_velocity = dq[3:6]
        kinematic_state.robot_velocity.joint_velocities = dq[6:]
        kinematic_state.robot_acceleration.generalized_joint_accelerations = ddq


    def optimize_initial_position(self, init_state):
        # Optimize the initial configuration
        q = se3.neutral(self.robot.model)
        
        plan_joint_init_pos = self.planner_setting.get(
            PlannerVectorParam_KinematicDefaultJointPositions)
        if len(plan_joint_init_pos) != self.robot.num_ctrl_joints:
            raise ValueError(
                'Number of joints in config file not same as required for robot\n' +
                'Got %d joints but robot expects %d joints.' % (
                    len(plan_joint_init_pos), self.robot.num_ctrl_joints))

        q[7:] = plan_joint_init_pos
        
        dq = np.zeros(self.robot.robot.nv)

        com_ref = init_state.com
        lmom_ref = np.zeros(3)
        amom_ref = np.zeros(3)
        num_eff = len(self.eff_names)
        endeff_pos_ref = np.array([init_state.effPosition(i) for i in range(num_eff)])
        endeff_vel_ref = np.matrix(np.zeros((num_eff, 3)))
        endeff_contact = np.ones(num_eff)

        quad_goal = se3.Quaternion(se3.rpy.rpyToMatrix(np.zeros([3,])))
 
        q[3:7] = quad_goal.coeffs()
        self.max_iterations = 100

        for iters in range(self.max_iterations):
            # Adding small P controller for the base orientation to always start with flat
            # oriented base.
            quad_q = se3.Quaternion(float(q[6]), float(q[3]), float(q[4]), float(q[5]))
            amom_ref = 1e-1 * se3.log((quad_goal * quad_q.inverse()).matrix())
            res = self.inv_kin.compute(q, dq, com_ref, lmom_ref, amom_ref,
                                      endeff_pos_ref, endeff_vel_ref, endeff_contact, None)
            q = se3.integrate(self.robot.model, q, res)

            if np.linalg.norm(res) < 1e-3:
                print('Found initial configuration after {} iterations'.format(iters + 1))
                break

        if iters == self.max_iterations - 1:
            print('Failed to converge for initial setup.')

        self.q_init = q.copy()
        self.dq_init = dq.copy()


    def optimize(self, init_state, contact_sequence, dynamic_sequence, plotting=False):
        self.init_state = init_state
        self.contact_sequence = contact_sequence
        self.dynamic_sequence = dynamic_sequence

        # Create array with centroidal and endeffector informations.
        self.fill_data_from_dynamics()
        
        self.fill_endeffector_trajectory()

        # Run the optimization for the initial configuration only once.
        if self.q_init is None:
            self.optimize_initial_position(init_state)
        
        # Generate smooth joint trajectory for regularization
        self.joint_des = np.zeros((len(self.q_init[7:]),self.num_time_steps), float)
        if self.n_via_joint == 0:
            for i in range (self.num_time_steps):
                self.joint_des[:,i] = self.q_init[7 : ].T
        else:
            joint_traj_gen = TrajectoryInterpolator()
            joint_traj_gen.num_time_steps = self.num_time_steps
            joint_traj_gen.init = self.q_init[7:]
            joint_traj_gen.end = self.q_init[7:]

            joint_traj_gen.generate_trajectory(self.n_via_joint, self.via_joint, self.dt)
            for it in range(self.num_time_steps):
                self.joint_des[:,it] = joint_traj_gen.evaluate_trajecory(it)
        
        # Generate smooth base trajectory for regularization
        self.base_des = np.zeros((3,self.num_time_steps), float)
        if self.n_via_base == 0:#other robot
            for it in range(self.num_time_steps):
                self.base_des[:,it] = np.array([0., 0., 0]).reshape(-1)
        else:
            base_traj_gen = TrajectoryInterpolator()
            base_traj_gen.num_time_steps = self.num_time_steps
            base_traj_gen.init = np.array([0.0, 0.0, 0])
            base_traj_gen.end = np.array([0.0, 0.0, 0])
            base_traj_gen.generate_trajectory(self.n_via_base, self.via_base, self.dt)
            for it in range(self.num_time_steps):
                self.base_des[:,it] = base_traj_gen.evaluate_trajecory(it)

        # Compute inverse kinematics over the full trajectory.
        self.inv_kin.is_init_time = 0
        q, dq = self.q_init.copy(), self.dq_init.copy()
        
        if self.use_second_order_inv_kin:
            
            q_kin, dq_kin, ddq_kin, com_kin, lmom_kin, amom_kin, endeff_pos_kin, endeff_vel_kin, self.rf_ori_kin, self.lf_ori_kin, result= \
                self.snd_order_inv_kin.solve(self.dt, q, dq, self.com_dyn, self.lmom_dyn,
                    self.amom_dyn, self.endeff_pos_ref, self.endeff_vel_ref,
                    self.endeff_contact, self.joint_des.T, self.base_des.T)
            if result != 2:
                for it, (q, dq, ddq) in enumerate(zip(q_kin, dq_kin, ddq_kin)):
                    self.inv_kin.forward_robot(q, dq)
                    self.fill_kinematic_result(it, q, dq, ddq) 
                    
        
                    #print(self.q_kin[it])
                    #print("dq")
                    #print(self.dq_kin[it])
                    #print("com_kin")#print
                    #print("base_kin")
                    #print(self.q_kin[it][0:7]) 
                    #print("eff_kin")
                    #print(self.q_kin[it][0:7]) 
                    #print("Eff_con1")
                    #print(self.endeff_pos_ref[it])
                    #print("Eff_con")
                    #print(endeff_pos_kin[it])
                    #print("rf_ori_kin")
                    #print(self.rf_ori_kin[it])
                    #print("lf_ori_kin")
                    #print(self.lf_ori_kin[it])
                return result
                
        else:
            for it in range(self.num_time_steps):
                quad_goal = se3.Quaternion(se3.rpy.rpyToMatrix(np.matrix(self.base_des[:,it]).T))
                quad_q = se3.Quaternion(float(q[6]), float(q[3]), float(q[4]), float(q[5]))

                ## check if this instance belongs to the flight phase and set the momentums accordingly
                for eff in range(len(self.eff_names)):
                    if self.dynamic_sequence.dynamics_states[it].effActivation(eff):
                        is_flight_phase = False
                        break
                    else:
                        is_flight_phase = True
                        
                ## for flight phase keep the desired momentum constant
                if is_flight_phase:
                    lmom_ref[0:2] = mom_ref_flight[0:2]
                    amom_ref = mom_ref_flight[3:6]
                    lmom_ref[2] -= self.planner_setting.get(PlannerDoubleParam_RobotWeight) * self.dt
                else:
                    lmom_ref = self.lmom_dyn[it]
                    amom_ref = (self.reg_orientation * se3.log((quad_goal * quad_q.inverse()).matrix()).T + self.amom_dyn[it]).reshape(-1)

                joint_regularization_ref = self.reg_joint_position * (self.joint_des[:,it] - q[7 : ])

                # Fill the kinematics results for it.
                self.inv_kin.forward_robot(q, dq)
                self.fill_kinematic_result(it, q, dq)

                # Store the momentum to be used in flight phase
                if not is_flight_phase:
                    mom_ref_flight = (self.inv_kin.J[0:6, :].dot(dq)).reshape(-1)

            # Compute the inverse kinematics  
                dq = self.inv_kin.compute(
                        q, dq, self.com_dyn[it], lmom_ref, amom_ref,
                        self.endeff_pos_ref[it], self.endeff_vel_ref[it],
                        self.endeff_contact[it], joint_regularization_ref,
                        is_flight_phase)

                # Integrate to the next state.
                q = se3.integrate(self.robot.model, q, dq * self.dt)
                
                '''
                print("comdyn")
                print(self.com_dyn[it]) 
                print("comkin")
                print(self.com_kin[it]) 
                print("lmom_ref")
                print(lmom_ref) 
                print("lmom_kin")
                print(self.lmom_kin[it])
                print("amom_ref")
                print(amom_ref) 
                print("amom_kin")
                print(self.amom_kin[it]) 
                '''                
                
            '''
            print("comdyn")
            print(self.com_dyn) 
            print("comkin")
            print(self.com_kin) 
            print("lmom_ref")
            print(lmom_ref) 
            print("lmom_kin")
            print(self.lmom_kin)
            print("amom_ref")
            print(amom_ref) 
            print("amom_kin")
            print(self.amom_kin) 
            '''
         #   print("end-posref")
         #   print(self.endeff_pos_ref)
         #   print(np.shape(self.endeff_pos_ref))
         #   print("End-real")
         #   print(self.motion_eff['trajectory'])
         #   print("end=eff contact")
         #   print(self.contact_plan[0])
            