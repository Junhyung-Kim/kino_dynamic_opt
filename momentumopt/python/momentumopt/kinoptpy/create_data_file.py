'''
@file create_data_file.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

import numpy as np

from pymomentum import *

from momentumopt.motion_execution import desired_state, interpolate

sample_frequency = 1 # 1kHz

def create_file(time_vector, optimized_sequence, optimized_dyn_plan, dynamics_feedback, robot_weight):
    sample_frequency = 1000 # 1kHz
    desired_pos = desired_state("POSITION", time_vector, optimized_sequence=optimized_sequence)
    desired_vel = desired_state("VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_gen_pos = desired_state("GENERALIZED_POSITION", time_vector, optimized_sequence=optimized_sequence)
    desired_gen_vel = desired_state("GENERALIZED_VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_gen_acc = desired_state("GENERALIZED_ACCELERATION", time_vector, optimized_sequence=optimized_sequence)
    desired_com = desired_state("COM", time_vector, optimized_sequence=optimized_sequence)
    desired_lmom = desired_state("LMOM", time_vector, optimized_sequence=optimized_sequence)
    desired_amom = desired_state("AMOM", time_vector, optimized_sequence=optimized_sequence)
    desired_comdy = desired_state("COMDY", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)
    desired_lmomdy = desired_state("LMOMDY", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)
    desired_amomdy = desired_state("AMOMDY", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)
    #desired_torquesc = desired_state("TORQUESC", time_vector, optimized_sequence=optimized_sequence,
    #                                optimized_dyn_plan=optimized_dyn_plan)
    #desired_torques = desired_state("TORQUES", time_vector, optimized_sequence=optimized_sequence,
    #                                optimized_dyn_plan=optimized_dyn_plan)
    #desired_EFF = desired_state("EFF_POSITION", time_vector, optimized_sequence=optimized_sequence, kin_optimizer=kin_optimizer)
    # desired_dyn_feedback = desired_state("DYN_FEEDBACK", time_vector, optimized_sequence=optimized_sequence,
    #                                 dynamics_feedback=dynamics_feedback)
    #desired_forces = desired_state("FORCES", time_vector, optimized_sequence=optimized_sequence,
    #                                optimized_dyn_plan=optimized_dyn_plan)
    desired_cop = desired_state("COP", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)
    desired_eori = desired_state("ENDEFFECTORORI", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)

    num_points = int(round(time_vector[-1] * sample_frequency))
    using_quadruped = True
    print(num_points)

    def dump_data(output_file, desired_fn, scale=1.):
        np.savetxt(output_file, np.vstack([
            np.hstack((i, scale * desired_fn(i / 1e3))) for i in range(num_points)
        ]), fmt='%.8e')

    if using_quadruped:
        dump_data("quadruped_positions.dat", desired_pos)
        dump_data("quadruped_velocities.dat", desired_vel)
        #dump_data("quadruped_forces.dat", desired_forces, robot_weight)
        #dump_data("quadruped_torques.dat", desired_torques)
        #dump_data("quadruped_torquesc.dat", desired_torquesc)
        dump_data("quadruped_generalized_positions.dat", desired_gen_pos)
        dump_data("quadruped_generalized_velocities.dat", desired_gen_vel)
        dump_data("quadruped_generalized_acceleration.dat", desired_gen_acc)
        dump_data("quadruped_com_old.dat", desired_com)
        dump_data("quadruped_lmom_old.dat", desired_lmom)
        dump_data("quadruped_amom_old.dat", desired_amom)
        dump_data("quadruped_cop_old.dat", desired_cop)
        dump_data("quadruped_end_ori.dat", desired_eori)
        dump_data("quadruped_comDyn_old.dat", desired_comdy)
        dump_data("quadruped_lmomDyn_old.dat", desired_lmomdy)
        dump_data("quadruped_amomDyn_old.dat", desired_amomdy)
    #    dump_data("quadruped_EFF.dat", desired_EFF)
        # dump_data("quadruped_dyn_feedback.dat", desired_dyn_feedback)
    else:  # using teststand
        des_positions = np.zeros((num_points, 3))
        des_velocities = np.zeros((num_points, 3))

        for i in range(num_points):
            # Only using the motion of the BL leg with the joints BL_HFE and BL_KFE
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)[:2]))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)[:2]))

        np.savetxt("teststand_positions.dat", des_positions)
        np.savetxt("teststand_velocities.dat", des_velocities)

def create_file1(time_vector, ini_state, optimized_sequence, optimized_dyn_plan, dynamics_feedback, robot_weight, kd_iter):
    sample_frequency = 1000 # 1kHz

    desired_pos = desired_state("POSITION", time_vector, optimized_sequence=optimized_sequence)
    desired_vel = desired_state("VELOCITY", time_vector, optimized_sequence=optimized_sequence)
   # desired_gen_pos = desired_state("GENERALIZED_POSITION", time_vector, optimized_sequence=optimized_sequence)
   # desired_gen_vel = desired_state("GENERALIZED_VELOCITY", time_vector, optimized_sequence=optimized_sequence)
   # desired_gen_acc = desired_state("GENERALIZED_ACCELERATION", time_vector, optimized_sequence=optimized_sequence)
    desired_com = desired_state("COM", time_vector, optimized_sequence=optimized_sequence)
    desired_lmom = desired_state("LMOM", time_vector, optimized_sequence=optimized_sequence)
    desired_amom = desired_state("AMOM", time_vector, optimized_sequence=optimized_sequence)
    desired_comdy = desired_state("COMDY", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)
    desired_lmomdy = desired_state("LMOMDY", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)
    desired_amomdy = desired_state("AMOMDY", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)
    desired_cop = desired_state("COP", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)
    desired_eori = desired_state("ENDEFFECTORORI", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)

    num_points = int(round(time_vector[-1] * sample_frequency))
    using_quadruped = True
    init_pos = np.vstack([ini_state.com, ini_state.lmom, ini_state.amom, ini_state.amomd])
    init_pos = init_pos.reshape(1,12)

    def dump_data(output_file, desired_fn, scale=1.):
        np.savetxt(output_file, np.vstack([
            np.hstack((i, scale * desired_fn(i / 1e3))) for i in range(num_points)
        ]), fmt='%.8e')

    def dump_data1(output_file, desired_fn, scale=1.):
        np.savetxt(output_file, desired_fn, fmt='%.8e')

    if using_quadruped:
        iter = str(kd_iter)
        text = "quadruped_positions" + iter + ".dat"
        dump_data(text, desired_pos)
        text = "quadruped_velocities" + iter + ".dat"
        dump_data(text, desired_vel)
        #  text = "quadruped_generalized_positions" + iter + ".dat"
      #  dump_data(text, desired_gen_pos)
      #  text = "quadruped_generalized_velocities" + iter + ".dat"
      #  dump_data(text, desired_gen_vel)
      #  text = "quadruped_generalized_acceleration" + iter + ".dat"
      #  dump_data(text, desired_gen_acc)
        text = "quadruped_com_old" + iter + ".dat"
        dump_data(text, desired_com)
        text = "quadruped_lmom_old" + iter + ".dat"
        dump_data(text, desired_lmom)
        text = "quadruped_amom_old" + iter + ".dat"
        dump_data(text, desired_amom)
        text = "quadruped_comDyn_old" + iter + ".dat"
        dump_data(text, desired_comdy)
        text = "quadruped_lmomDyn_old" + iter + ".dat"
        dump_data(text, desired_lmomdy)
        text = "quadruped_amomDyn_old" + iter + ".dat"
        dump_data(text, desired_amomdy)
        text = "quadruped_cop_old" + iter + ".dat"
        dump_data(text, desired_cop)
        text = "quadruped_end_ori" + iter + ".dat"
        dump_data(text, desired_eori)
        #    dump_data("quadruped_EFF.dat", desired_EFF)
            # dump_data("quadruped_dyn_feedback.dat", desired_dyn_feedback)
    else:  # using teststand
        des_positions = np.zeros((num_points, 3))
        des_velocities = np.zeros((num_points, 3))

        for i in range(num_points):
                # Only using the motion of the BL leg with the joints BL_HFE and BL_KFE
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)[:2]))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)[:2]))

        np.savetxt("teststand_positions.dat", des_positions)
        np.savetxt("teststand_velocities.dat", des_velocities)

def create_lqr_files1(time_vector, optimized_motion_eff, optimized_sequence, optimized_dyn_plan, dynamics_feedback, robot_weight, kd_iter):
    desired_pos = interpolate("POSITION", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_vel = interpolate("VELOCITY", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_com = interpolate("COM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_lmom = interpolate("LMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_amom = interpolate("AMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_forces = interpolate("FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, dynamics_feedback = dynamics_feedback)
    #desired_lqr_gains = interpolate("DYN_FEEDBACK", time_vector, dynamics_feedback = dynamics_feedback)
    desired_quaternion = interpolate("QUATERNION", time_vector, optimized_sequence=optimized_sequence)
    desired_centroidal_forces = interpolate("CENTROIDAL_FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_centroidal_moments = interpolate("CENTROIDAL_MOMENTS", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_base_ang_velocity = interpolate("BASE_ANGULAR_VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_pos_abs = interpolate("POSITION_ABSOLUTE", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_ori_rf = interpolate("ORI_RF", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_ori_lf = interpolate("ORI_LF", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
   
    max_time = 0 # time horizon in seconds
    save_horizon = 0

    if time_vector[-1] - int(time_vector[-1]) > 0.001:
        max_time = int(time_vector[-1]) + 1
    else:
        max_time = int(time_vector[-1])

    print("max_time:" , max_time)
    num_points = max_time * sample_frequency

    using_quadruped = True

    def dump_data(output_file, desired_fn):
        np.savetxt(output_file, np.vstack([
            np.hstack((i, desired_fn(i / 1e3))) for i in range(num_points)
        ]))

    if using_quadruped:
        des_positions = np.zeros((num_points, 7))
        des_velocities = np.zeros((num_points, 7))
        des_forces = np.zeros((num_points, 12*(save_horizon+1)+1))
        des_com = np.zeros((num_points, 3*(save_horizon+1)+1))
        des_lmom = np.zeros((num_points, 3*(save_horizon+1)+1))
        des_quaternion = np.zeros((num_points, 4*(save_horizon+1)+1))
        des_base_ang_velocities = np.zeros((num_points, 3*(save_horizon+1)+1))

        des_centroidal_forces = np.zeros((num_points, 4))
        des_centroidal_moments = np.zeros((num_points, 4))
        des_positions_abs = np.zeros((num_points, 7))
        des_ori_rf = np.zeros((num_points, 4))
        des_ori_lf = np.zeros((num_points, 4))
        des_contact_activation = np.zeros((num_points, 2*(save_horizon+1)+1))
        
        for i in range(num_points):
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))
            des_forces[i, 0:13] = np.hstack((i, desired_forces(i /1e3)))
            des_com[i, 0:4] = np.hstack((i, desired_com(i / 1e3)))
            des_lmom[i, 0:4] = np.hstack((i, desired_lmom(i / 1e3)))
            des_quaternion[i, 0:5] = np.hstack((i, desired_quaternion(i / 1e3)))
            des_base_ang_velocities[i, 0:4] = np.hstack((i, desired_base_ang_velocity(i / 1e3)))
            des_positions_abs[i, 0:13] = np.hstack((i, desired_pos_abs(i /1e3)))
            des_ori_rf[i, :] = np.hstack((i, desired_ori_rf(i /1e3)))
            des_ori_lf[i, :] = np.hstack((i, desired_ori_lf(i /1e3)))
            des_centroidal_forces[i,:] = np.hstack((i, desired_centroidal_forces(i /1e3)))
            des_centroidal_moments[i,:] = np.hstack((i, desired_centroidal_moments(i /1e3)))
            '''
            for j in range(save_horizon):
                if i<num_points-save_horizon:
                    des_com[i, 3*(j+1)+1:3*(j+2)+1] = desired_com((i+j+1) / 1e3)
                    des_lmom[i, 3*(j+1)+1:3*(j+2)+1] = desired_lmom((i+j+1) / 1e3)
                    des_base_ang_velocities[i, 3*(j+1)+1:3*(j+2)+1] = desired_base_ang_velocity((i+j+1) / 1e3)
                    des_quaternion[i, 4*(j+1)+1:4*(j+2)+1] = desired_quaternion((i+j+1) / 1e3)
                    des_forces[i, 12*(j+1)+1:12*(j+2)+1] = desired_forces((i+j+1) / 1e3)
                    des_positions_abs[i, 12*(j+1)+1:12*(j+2)+1] = desired_pos_abs((i+j+1) / 1e3)
                else:
                    des_com[i, 3*(j+1)+1:3*(j+2)+1] = desired_com((num_points) / 1e3)
                    des_lmom[i, 3*(j+1)+1:3*(j+2)+1] = desired_lmom((num_points) / 1e3)
                    des_base_ang_velocities[i, 3*(j+1)+1:3*(j+2)+1] = desired_base_ang_velocity((num_points) / 1e3)
                    des_quaternion[i, 4*(j+1)+1:4*(j+2)+1] = desired_quaternion((num_points) / 1e3)
                    des_forces[i, 12*(j+1)+1:12*(j+2)+1] = desired_forces((num_points) / 1e3)
                    des_positions_abs[i, 12*(j+1)+1:12*(j+2)+1] = desired_pos_abs((num_points) / 1e3)

        ## resequencing the eff sequence
        # des_forces[: ,[1,2,3]], des_forces[: ,[10,11,12]] =\
        # des_forces[: ,[10,11,12]], des_forces[:, [1,2,3]].copy()
        # des_forces[: ,[4,5,6]], des_forces[: ,[7,8,9]] = \
        # des_forces[: ,[7,8,9]], des_forces[:, [4,5,6]].copy()
        for j in range(save_horizon+1):
            des_forces[: ,[12*j+1,12*j+2,12*j+3]], des_forces[: ,[12*j+10,12*j+11,12*j+12]] =\
                    des_forces[: ,[12*j+10,12*j+11,12*j+12]], des_forces[:, [12*j+1,12*j+2,12*j+3]].copy()
            des_forces[: ,[12*j+4,12*j+5,12*j+6]], des_forces[: ,[12*j+7,12*j+8,12*j+9]] = \
                    des_forces[: ,[12*j+7,12*j+8,12*j+9]], des_forces[:, [12*j+4,12*j+5,12*j+6]].copy()

            des_positions_abs[: ,[12*j+1,12*j+2,12*j+3]], des_positions_abs[: ,[12*j+10,12*j+11,12*j+12]] =\
                    des_positions_abs[: ,[12*j+10,12*j+11,12*j+12]], des_positions_abs[:, [12*j+1,12*j+2,12*j+3]].copy()
            des_positions_abs[: ,[12*j+4,12*j+5,12*j+6]], des_positions_abs[: ,[12*j+7,12*j+8,12*j+9]] = \
                    des_positions_abs[: ,[12*j+7,12*j+8,12*j+9]], des_positions_abs[:, [12*j+4,12*j+5,12*j+6]].copy()

        # filling contact switch vector in the horizon
        for i in range (num_points):
            des_contact_activation[i, 0]=i
            for j in range(4*(save_horizon+1)):
                if des_forces[i, 3*j+3]>0:
                    des_contact_activation[i, j+1]=1
                else:
                    des_contact_activation[i, j+1]=0

        des_positions[: ,[1,2,3]], des_positions[: ,[4,5,6]] = des_positions[: ,[4,5,6]], des_positions[:, [1,2,3]].copy()
        des_positions[: ,[7,8,9]], des_positions[: ,[10,11,12]] = des_positions[: ,[10,11,12]], des_positions[:, [7,8,9]].copy()

        des_velocities[: ,[1,2,3]], des_velocities[: ,[4,5,6]] = des_velocities[: ,[4,5,6]], des_velocities[:, [1,2,3]].copy()
        des_velocities[: ,[7,8,9]], des_velocities[: ,[10,11,12]] = des_velocities[: ,[10,11,12]], des_velocities[:, [7,8,9]].copy()

        des_positions_final = np.zeros((num_points, 24))
        des_velocities_final = np.zeros((num_points, 24))


        ## Converting des_pos and des_vel of end effector to a 6d vector per leg
        ## for the impedance controller
        ## so in total n*24
        for i in range(num_points):
            for eff in range(4):
                des_positions_final[i][6*eff:6*(eff+1)] = np.hstack((des_positions[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))
                des_velocities_final[i][6*eff:6*(eff+1)] = np.hstack((des_velocities[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))
        '''
        des_forces*=robot_weight
        #print(desired_pos)
        print("saving trajectories....")
        iter = str(kd_iter)
        text = "quadruped_positions_eff" + iter + ".dat"
        np.savetxt(text, des_positions)
        text = "quadruped_velocities_eff" + iter + ".dat"
        np.savetxt(text, des_velocities)
        text = "quadruped_com_with_horizon" + iter + ".dat"
        np.savetxt(text, des_com)
        text = "quadruped_lmom_with_horizon" + iter + ".dat"
        np.savetxt(text, des_lmom)
        text = "quadruped_forces_with_horizon" + iter + ".dat"
        np.savetxt(text, des_forces)
        text = "quadruped_quaternion_with_horizon" + iter + ".dat"
        np.savetxt(text, des_quaternion)
        text = "quadruped_base_ang_velocities_with_horizon" + iter + ".dat"
        np.savetxt(text, des_base_ang_velocities)
        text = "quadruped_positions_abs_with_horizon_part" + iter + ".dat"
        np.savetxt(text, des_positions_abs)
        text = "des_contact_activation_with_horizon" + iter + ".dat"
        np.savetxt(text, des_contact_activation)
        text = "quadruped_desired_centroidal_forces" + iter + ".dat"
        np.savetxt(text, des_centroidal_forces)
        text = "quadruped_desired_centroidal_moments" + iter + ".dat"
        np.savetxt(text, des_centroidal_moments)
        text = "quadruped_rf_ori" + iter + ".dat"
        np.savetxt(text, des_ori_rf)
        text = "quadruped_lf_ori" + iter + ".dat"
        np.savetxt(text, des_ori_lf)


def create_qp_files(time_vector, optimized_motion_eff, optimized_sequence, optimized_dyn_plan, dynamics_feedback, robot_weight):
    desired_joint_pos = desired_state("POSITION", time_vector, optimized_sequence=optimized_sequence)
    desired_joint_vel = desired_state("VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_pos = interpolate("POSITION", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_vel = interpolate("VELOCITY", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_com = interpolate("COM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_lmom = interpolate("LMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    # desired_amom = interpolate("AMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_forces = interpolate("FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, dynamics_feedback = dynamics_feedback)
    # desired_lqr_gains = interpolate("DYN_FEEDBACK", time_vector, dynamics_feedback = dynamics_feedback)
    desired_quaternion = interpolate("QUATERNION", time_vector, optimized_sequence=optimized_sequence)
    desired_centroidal_forces = interpolate("CENTROIDAL_FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_centroidal_moments = interpolate("CENTROIDAL_MOMENTS", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_base_ang_velocity = interpolate("BASE_ANGULAR_VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_pos_abs = interpolate("POSITION_ABSOLUTE", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_vel_abs = interpolate("VELOCITY_ABSOLUTE", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_ori_rf = interpolate("ORI_RF", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_ori_lf = interpolate("ORI_LF", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    
    max_time = 0 # time horizon in seconds

    if time_vector[-1] - int(time_vector[-1]) > 0.001:
        max_time = int(time_vector[-1]) + 1
    else:
        max_time = int(time_vector[-1])

    print("max_time:" , max_time)
    num_points = max_time * sample_frequency

    using_quadruped = True
    num_joints = len(optimized_sequence.kinematics_states[0].robot_posture.generalized_joint_positions) - 7

    def dump_data(output_file, desired_fn):
        np.savetxt(output_file, np.vstack([
            np.hstack((i, desired_fn(i / 1e3))) for i in range(num_points)
        ]))

    if using_quadruped:
        desired_joint_positions = np.zeros((num_points, num_joints + 1))
        desired_joint_velocities = np.zeros((num_points, num_joints + 1))
        des_positions = np.zeros((num_points, 13))
        des_velocities = np.zeros((num_points, 13))
        des_com = np.zeros((num_points, 4))
        des_lmom = np.zeros((num_points, 4))
        des_com_vel = np.zeros((num_points, 4))
        # des_amom = np.zeros((num_points, 4))
        # des_lqr_gains = np.zeros((num_points, 108))
        des_forces = np.zeros((num_points, 13))
        des_centroidal_forces = np.zeros((num_points, 4))
        des_centroidal_moments = np.zeros((num_points, 4))
        des_quaternion = np.zeros((num_points, 5))
        des_base_ang_velocities = np.zeros((num_points, 4))
        des_positions_abs = np.zeros((num_points, 13))
        des_velocities_abs = np.zeros((num_points, 13))
        des_contact_activation = np.zeros((num_points, 5))
        des_ori_rf = np.zeros((num_points, 4))
        des_ori_lf = np.zeros((num_points, 4))
        gain_ratio = np.zeros((num_points, 9))


        for i in range(num_points):
            ## making des_pos and des_vel a 6d vector
            ### re arranging the sequence of legs to the latest from (FR, FL, HR, HL)
            ### to (FL, FR, HL, HR) for des_pos, des_vel and forces
            des_forces[i:, ] = np.hstack((i, desired_forces(i /1e3)))
            des_centroidal_forces[i:, ] = np.hstack((i, desired_centroidal_forces(i /1e3)))
            des_centroidal_moments[i:, ] = np.hstack((i, desired_centroidal_moments(i /1e3)))
            desired_joint_positions[i, :] = np.hstack((i, desired_joint_pos(i / 1e3)))
            desired_joint_velocities[i, :] = np.hstack((i, desired_joint_vel(i / 1e3)))
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))
            des_com[i, :] = np.hstack((i, desired_com(i / 1e3)))
            des_com_vel[i, :] = np.hstack((i, desired_lmom(i / 1e3)/(robot_weight/9.81)))
            # des_amom[i, :] = np.hstack((i, desired_amom(i / 1e3)))
            # des_lqr_gains_tmp = desired_lqr_gains(i / 1e3)
            # ## Converting lqr matrix (12 * 9) to a 108d vector
            # des_lqr_gains_tmp = np.reshape(des_lqr_gains_tmp, (108,))
            # des_lqr_gains[i,: ] = des_lqr_gains_tmp
            des_quaternion[i, :] = np.hstack((i, desired_quaternion(i / 1e3)))
            des_base_ang_velocities[i, :] = np.hstack((i, desired_base_ang_velocity(i / 1e3)))
            des_positions_abs[i, :] = np.hstack((i, desired_pos_abs(i /1e3)))
            des_velocities_abs[i, :] = np.hstack((i, desired_vel_abs(i /1e3)))
            des_ori_rf[i, :] = np.hstack((i, desired_ori_rf(i /1e3)))
            des_ori_lf[i, :] = np.hstack((i, desired_ori_lf(i /1e3)))
            gain_ratio[i, 0] = i

        ## resequencing the eff sequence

        def swap_coordinates_vec12(arr):
            arr[: ,[1,2,3]], arr[: ,[4,5,6]] = arr[: ,[4,5,6]], arr[:, [1,2,3]].copy()
            arr[: ,[7,8,9]], arr[: ,[10,11,12]] = arr[: ,[10,11,12]], arr[:, [7,8,9]].copy()

        swap_coordinates_vec12(des_forces)
        swap_coordinates_vec12(des_positions)
        swap_coordinates_vec12(des_velocities)
        swap_coordinates_vec12(des_positions_abs)
        swap_coordinates_vec12(des_velocities_abs)

        # filling contact switch vector in the horizon
        for i in range (num_points):
            des_contact_activation[i, 0]=i
            for j in range(4):
                if des_forces[i, 3*j+3]>0:
                    des_contact_activation[i, j+1]=1
                else:
                    des_contact_activation[i, j+1]=0

        des_positions_final = np.zeros((num_points, 24))
        des_velocities_final = np.zeros((num_points, 24))


        ## Converting des_pos and des_vel of end effector to a 6d vector per leg
        ## for the impedance controller
        ## so in total n*24
        for i in range(num_points):
            for eff in range(4):
                des_positions_final[i][6*eff:6*(eff+1)] = np.hstack((des_positions[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))
                des_velocities_final[i][6*eff:6*(eff+1)] = np.hstack((des_velocities[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))

        print("saving trajectories....")
        np.savetxt("quadruped_positions.dat", desired_joint_positions)
        np.savetxt("quadruped_velocities.dat", des_velocities_final)
        np.savetxt("quadruped_positions_eff.dat", des_positions_final)
        np.savetxt("quadruped_velocities_eff.dat", des_velocities_final)
        np.savetxt("quadruped_rf_ori.dat", des_ori_rf)
        np.savetxt("quadruped_lf_ori.dat", des_ori_lf)
        np.savetxt("quadruped_com.dat", des_com)
        np.savetxt("quadruped_com_vel.dat", des_com_vel)
        np.savetxt("quadruped_centroidal_forces.dat", des_centroidal_forces)
        np.savetxt("quadruped_centroidal_moments.dat", des_centroidal_moments)
        np.savetxt("quadruped_quaternion.dat", des_quaternion)
        np.savetxt("quadruped_base_ang_velocities.dat", des_base_ang_velocities)
        np.savetxt("quadruped_positions_abs.dat", des_positions_abs)
        np.savetxt("quadruped_velocities_abs.dat", des_velocities_abs)
        np.savetxt("quadruped_contact_activation.dat", des_contact_activation)
        np.savetxt("quadruped_gain_ratio.dat", gain_ratio)



def create_lqr_files(time_vector, optimized_motion_eff, optimized_sequence, optimized_dyn_plan, dynamics_feedback, robot_weight):
    desired_pos = interpolate("POSITION", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_vel = interpolate("VELOCITY", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_com = interpolate("COM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_lmom = interpolate("LMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_amom = interpolate("AMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_forces = interpolate("FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, dynamics_feedback = dynamics_feedback)
    #desired_lqr_gains = interpolate("DYN_FEEDBACK", time_vector, dynamics_feedback = dynamics_feedback)
    desired_quaternion = interpolate("QUATERNION", time_vector, optimized_sequence=optimized_sequence)
    desired_centroidal_forces = interpolate("CENTROIDAL_FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_centroidal_moments = interpolate("CENTROIDAL_MOMENTS", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_base_ang_velocity = interpolate("BASE_ANGULAR_VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_pos_abs = interpolate("POSITION_ABSOLUTE", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_ori_rf = interpolate("ORI_RF", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_ori_lf = interpolate("ORI_LF", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
   
    max_time = 0 # time horizon in seconds
    save_horizon = 0

    if time_vector[-1] - int(time_vector[-1]) > 0.001:
        max_time = int(time_vector[-1]) + 1
    else:
        max_time = int(time_vector[-1])

    print("max_time:" , max_time)
    num_points = max_time * sample_frequency

    using_quadruped = True

    def dump_data(output_file, desired_fn):
        np.savetxt(output_file, np.vstack([
            np.hstack((i, desired_fn(i / 1e3))) for i in range(num_points)
        ]))

    if using_quadruped:
        des_positions = np.zeros((num_points, 7))
        des_velocities = np.zeros((num_points, 7))
        des_forces = np.zeros((num_points, 12*(save_horizon+1)+1))
        des_com = np.zeros((num_points, 3*(save_horizon+1)+1))
        des_lmom = np.zeros((num_points, 3*(save_horizon+1)+1))
        des_quaternion = np.zeros((num_points, 4*(save_horizon+1)+1))
        des_base_ang_velocities = np.zeros((num_points, 3*(save_horizon+1)+1))

        des_centroidal_forces = np.zeros((num_points, 4))
        des_centroidal_moments = np.zeros((num_points, 4))
        des_positions_abs = np.zeros((num_points, 7))
        des_ori_rf = np.zeros((num_points, 4))
        des_ori_lf = np.zeros((num_points, 4))
        des_contact_activation = np.zeros((num_points, 2*(save_horizon+1)+1))
        
        for i in range(num_points):
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))
            des_forces[i, 0:13] = np.hstack((i, desired_forces(i /1e3)))
            des_com[i, 0:4] = np.hstack((i, desired_com(i / 1e3)))
            des_lmom[i, 0:4] = np.hstack((i, desired_lmom(i / 1e3)))
            des_quaternion[i, 0:5] = np.hstack((i, desired_quaternion(i / 1e3)))
            des_base_ang_velocities[i, 0:4] = np.hstack((i, desired_base_ang_velocity(i / 1e3)))
            des_positions_abs[i, 0:13] = np.hstack((i, desired_pos_abs(i /1e3)))
            des_ori_rf[i, :] = np.hstack((i, desired_ori_rf(i /1e3)))
            des_ori_lf[i, :] = np.hstack((i, desired_ori_lf(i /1e3)))
            des_centroidal_forces[i,:] = np.hstack((i, desired_centroidal_forces(i /1e3)))
            des_centroidal_moments[i,:] = np.hstack((i, desired_centroidal_moments(i /1e3)))
            '''
            for j in range(save_horizon):
                if i<num_points-save_horizon:
                    des_com[i, 3*(j+1)+1:3*(j+2)+1] = desired_com((i+j+1) / 1e3)
                    des_lmom[i, 3*(j+1)+1:3*(j+2)+1] = desired_lmom((i+j+1) / 1e3)
                    des_base_ang_velocities[i, 3*(j+1)+1:3*(j+2)+1] = desired_base_ang_velocity((i+j+1) / 1e3)
                    des_quaternion[i, 4*(j+1)+1:4*(j+2)+1] = desired_quaternion((i+j+1) / 1e3)
                    des_forces[i, 12*(j+1)+1:12*(j+2)+1] = desired_forces((i+j+1) / 1e3)
                    des_positions_abs[i, 12*(j+1)+1:12*(j+2)+1] = desired_pos_abs((i+j+1) / 1e3)
                else:
                    des_com[i, 3*(j+1)+1:3*(j+2)+1] = desired_com((num_points) / 1e3)
                    des_lmom[i, 3*(j+1)+1:3*(j+2)+1] = desired_lmom((num_points) / 1e3)
                    des_base_ang_velocities[i, 3*(j+1)+1:3*(j+2)+1] = desired_base_ang_velocity((num_points) / 1e3)
                    des_quaternion[i, 4*(j+1)+1:4*(j+2)+1] = desired_quaternion((num_points) / 1e3)
                    des_forces[i, 12*(j+1)+1:12*(j+2)+1] = desired_forces((num_points) / 1e3)
                    des_positions_abs[i, 12*(j+1)+1:12*(j+2)+1] = desired_pos_abs((num_points) / 1e3)

        ## resequencing the eff sequence
        # des_forces[: ,[1,2,3]], des_forces[: ,[10,11,12]] =\
        # des_forces[: ,[10,11,12]], des_forces[:, [1,2,3]].copy()
        # des_forces[: ,[4,5,6]], des_forces[: ,[7,8,9]] = \
        # des_forces[: ,[7,8,9]], des_forces[:, [4,5,6]].copy()
        for j in range(save_horizon+1):
            des_forces[: ,[12*j+1,12*j+2,12*j+3]], des_forces[: ,[12*j+10,12*j+11,12*j+12]] =\
                    des_forces[: ,[12*j+10,12*j+11,12*j+12]], des_forces[:, [12*j+1,12*j+2,12*j+3]].copy()
            des_forces[: ,[12*j+4,12*j+5,12*j+6]], des_forces[: ,[12*j+7,12*j+8,12*j+9]] = \
                    des_forces[: ,[12*j+7,12*j+8,12*j+9]], des_forces[:, [12*j+4,12*j+5,12*j+6]].copy()

            des_positions_abs[: ,[12*j+1,12*j+2,12*j+3]], des_positions_abs[: ,[12*j+10,12*j+11,12*j+12]] =\
                    des_positions_abs[: ,[12*j+10,12*j+11,12*j+12]], des_positions_abs[:, [12*j+1,12*j+2,12*j+3]].copy()
            des_positions_abs[: ,[12*j+4,12*j+5,12*j+6]], des_positions_abs[: ,[12*j+7,12*j+8,12*j+9]] = \
                    des_positions_abs[: ,[12*j+7,12*j+8,12*j+9]], des_positions_abs[:, [12*j+4,12*j+5,12*j+6]].copy()

        # filling contact switch vector in the horizon
        for i in range (num_points):
            des_contact_activation[i, 0]=i
            for j in range(4*(save_horizon+1)):
                if des_forces[i, 3*j+3]>0:
                    des_contact_activation[i, j+1]=1
                else:
                    des_contact_activation[i, j+1]=0

        des_positions[: ,[1,2,3]], des_positions[: ,[4,5,6]] = des_positions[: ,[4,5,6]], des_positions[:, [1,2,3]].copy()
        des_positions[: ,[7,8,9]], des_positions[: ,[10,11,12]] = des_positions[: ,[10,11,12]], des_positions[:, [7,8,9]].copy()

        des_velocities[: ,[1,2,3]], des_velocities[: ,[4,5,6]] = des_velocities[: ,[4,5,6]], des_velocities[:, [1,2,3]].copy()
        des_velocities[: ,[7,8,9]], des_velocities[: ,[10,11,12]] = des_velocities[: ,[10,11,12]], des_velocities[:, [7,8,9]].copy()

        des_positions_final = np.zeros((num_points, 24))
        des_velocities_final = np.zeros((num_points, 24))


        ## Converting des_pos and des_vel of end effector to a 6d vector per leg
        ## for the impedance controller
        ## so in total n*24
        for i in range(num_points):
            for eff in range(4):
                des_positions_final[i][6*eff:6*(eff+1)] = np.hstack((des_positions[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))
                des_velocities_final[i][6*eff:6*(eff+1)] = np.hstack((des_velocities[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))
        '''
        des_forces*=robot_weight
        #print(desired_pos)
        print("saving trajectories....")
        np.savetxt("quadruped_positions_eff.dat", des_positions)
        np.savetxt("quadruped_velocities_eff.dat", des_velocities)
        np.savetxt("quadruped_com_with_horizon.dat", des_com)
        np.savetxt("quadruped_lmom_with_horizon.dat", des_lmom)
        np.savetxt("quadruped_forces_with_horizon.dat", des_forces)
        np.savetxt("quadruped_quaternion_with_horizon.dat", des_quaternion)
        np.savetxt("quadruped_base_ang_velocities_with_horizon.dat", des_base_ang_velocities)
        np.savetxt("quadruped_positions_abs_with_horizon_part.dat", des_positions_abs)
        np.savetxt("des_contact_activation_with_horizon.dat", des_contact_activation)
        np.savetxt("quadruped_desired_centroidal_forces.dat", des_centroidal_forces)
        np.savetxt("quadruped_desired_centroidal_moments.dat", des_centroidal_moments)
        np.savetxt("quadruped_rf_ori.dat", des_ori_rf)
        np.savetxt("quadruped_lf_ori.dat", des_ori_lf)
