'''
@file blmc_robot_wrapper.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

import os

import numpy as np
import pinocchio
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import zero
from robot_properties_bolt.config import BoltConfig
from robot_properties_tocabi.config import TocabiConfig
from robot_properties_solo.config import Solo8Config, Solo12Config

class BasicRobotWrapper(object):

    def __init__(self):
        self.model = None
        self.data = None
        self.q = None

    def set_configuration(self, q):
        self.q = q
        pinocchio.forwardKinematics(self.model, self.data, self.q)

    def set_velocity(self, dq):
        self.dq = dq

    def set_acceleration(self, ddq):
        self.ddq = ddq

    def update_configuration(self, delta_q):
        self.q = pinocchio.integrate(self.model, self.q, delta_q)

        # pinocchio.forwardKinematics(self.model, self.data, self.q)
        # pinocchio.framesKinematics(self.model, self.data)
    def get_difference(self, q_1, q_2):
        return pinocchio.difference(self.model, q_1, q_2)

    def get_world_oriented_frame_jacobian(self, index):
        self.robot.forwardKinematics(self.q, self.dq)
        self.robot.computeJointJacobians(self.q)
        self.robot.framesForwardKinematics(self.q)
        jac = pinocchio.getFrameJacobian(self.model, self.data, index, pinocchio.ReferenceFrame.LOCAL)
        world_R_joint = pinocchio.SE3(self.data.oMf[index].rotation, zero(3))
        return world_R_joint.action.dot(jac)

    def get_jacobian(self, name, dofs=None, internal=True):
        if not self.model.existFrame(name) and not name == "COM":
            raise ValueError("Joint %s is not available." %name)
        if name == "universe" or name == "root_joint":
            raise ValueError("Joint %s is not available." %name)

        range_ = None
        if dofs == "TRANSLATION":
            range_ = range(3)
        elif dofs == "ROTATION":
            if name == "COM":
                raise ValueError("No rotation for COM available")
            range_ = range(3, 6)
        else:
            range_ = range(6)

        if internal:
            if name == "COM":
                def eval_jac_internal_com():
                    return self.robot.Jcom(self.q)
                return eval_jac_internal_com
            else:
                index = self.model.getFrameId(name)
                def eval_jac_internal():
                    return self.get_world_oriented_frame_jacobian(index)[range_, :]
                    # return pinocchio.frameJacobian(self.model, self.data, self.q, index, pinocchio.ReferenceFrame.LOCAL)[range_, :]
                return eval_jac_internal
        else:
            if name == "COM":
                return self.Jcom
            else:
                index = self.model.getFrameId(name)
                def eval_jac_at_q(q):
                    return self.get_world_oriented_frame_jacobian(index)[range_, :]
                    # return pinocchio.frameJacobian(self.model, self.data, q, index, pinocchio.ReferenceFrame.LOCAL)[range_, :]
                return eval_jac_at_q

    def get_centroidal_momentum(self):
        def eval_centroidal_momentum():
            self.robot.centroidalMomentum(self.q, self.dq)
            centroidal_momentum_matrix = self.data.Ag
            return centroidal_momentum_matrix

        return eval_centroidal_momentum

    def get_d_centroidal_momentum(self):
        def eval_d_centroidal_momentum():
            self.robot.centroidalMomentum(self.q, self.dq)
            d_centroidal_momentum_matrix = self.data.dAg
            return d_centroidal_momentum_matrix

        return eval_d_centroidal_momentum

    def get_transformation(self, name, dofs=None):
        if not self.model.existFrame(name) and not name == "COM":
            raise ValueError("Transformation for %s is not available." %name)
        if name == "universe" or name == "root_joint":
            raise ValueError("Transformation for %s is not available." %name)

        def transformation():
            index = self.model.getFrameId(name)
            if dofs == "TRANSLATION":
                return self.data.oMf[index].translation
            elif dofs == "ROTATION":
                if name == "COM":
                    raise ValueError("No rotation for COM available")
                return self.data.oMf[index].rotation
            else:
                return self.data.oMf[index]

        def transformation_com():
            return self.robot.com(self.q)

        if name == "COM":
            return transformation_com
        else:
            return transformation

    def get_desired_velocity(self, goal, transformation_func, dofs=None):
        def eval_vel(delta_t):
            if dofs == "TRANSLATION":
                #print("delta_t:" , delta_t)
                return (goal - transformation_func()) / delta_t
            elif dofs is None:
                return pinocchio.log(transformation_func().inverse() * goal).vector / delta_t
            else:
                raise ValueError("Implementation for %s not available" %dofs)

        return eval_vel

    def initDisplay(self, loadModel=True):
        self.robot.initViewer(loadModel=loadModel)
        #self.viz.initViewer(True)
        self.robot.viewer.gui.addFloor('world/floor')
        self.robot.viewer.gui.applyConfiguration('world/floor', [
            0.0, 0.0, self.floor_height,  0.0, 0.0, 0.0, 1.0])
        self.robot.viewer.gui.refresh()

    def ensureDisplay(self):
        if not hasattr(self.robot, 'viewer'):
            self.initDisplay()

    def display(self,q):
        #RobotWrapper.display(self,q)
        self.robot.display(q)
        pinocchio.updateFramePlacements(self.model,self.data)
        self.robot.viewer.gui.refresh()

    def initMeshcat(self):
        viz = pinocchio.visualize.MeshcatVisualizer(self.robot.model, self.robot.collision_model, self.robot.visual_model)
        viz.initViewer(open=True)
        return viz


class QuadrupedWrapper(BasicRobotWrapper):

    def __init__(self, q=None):
        super(QuadrupedWrapper, self).__init__()

        self.effs = ["FR", "FL", "HR", "HL"]  # order is important
        self.colors = {"HL": "r", "HR": "y", "FL": "b", "FR": "g"}
        self.joints_list = ["HFE", "KFE", "ANKLE"]
        self.floor_height = 0.

        self.robot = Solo8Config.buildRobotWrapper()

        self.num_ctrl_joints = 8

        # Create data again after setting frames
        self.model = self.robot.model
        self.data = self.robot.data
        q = pinocchio.neutral(self.robot.model)
        if not q is None:
            self.set_configuration(q)
        else:
            self.q = None
        self.M_com = None
        self.mass = sum([i.mass for i in self.model.inertias[1:]])
        self.set_init_config()

    def set_init_config(self):
        model = self.model
        data = self.data
        NQ = model.nq
        NV = model.nv
        self.q, self.dq, self.ddq, tau = zero(NQ), zero(NV), zero(NV), zero(NV)

        self.q = pinocchio.neutral(self.robot.model)
        self.q[2] = self.floor_height

        # Set initial configuration
        angle = np.deg2rad(60.0)
        q_dummy = np.zeros(self.num_ctrl_joints)
        q_dummy[:] = angle
        q_dummy[::2] = -0.5 * angle
        self.q[7:] = q_dummy

        # print(self.q)
        self.set_configuration(self.q)


class Quadruped12Wrapper(BasicRobotWrapper):

    def __init__(self, q=None):
        super(Quadruped12Wrapper, self).__init__()

        self.effs = ["FR", "FL", "HR", "HL"]  # order is important
        self.colors = {"HL": "r", "HR": "y", "FL": "b", "FR": "g"}
        self.joints_list = ["HAA", "HFE", "KFE", "ANKLE"]
        self.floor_height = 0.
        print("solo")
        self.robot = Solo12Config.buildRobotWrapper()

        self.num_ctrl_joints = 12

        # Create data again after setting frames
        self.model = self.robot.model
        self.data = self.robot.data
        q = pinocchio.neutral(self.robot.model)
        if not q is None:
            self.set_configuration(q)
        else:
            self.q = None
        self.M_com = None
        self.mass = sum([i.mass for i in self.model.inertias[1:]])
        self.set_init_config()

    def set_init_config(self):
        model = self.model
        data = self.data
        NQ = model.nq
        NV = model.nv
        self.q, self.dq, self.ddq, tau = zero(NQ), zero(NV), zero(NV), zero(NV)

        self.q = pinocchio.neutral(self.robot.model)
        self.q[2] = self.floor_height

        # Set initial configuration
        angle = np.deg2rad(60.0)
        q_dummy = np.zeros(self.num_ctrl_joints)
        q_dummy[2::3] = angle
        q_dummy[1:3] = -0.5 * angle

        self.q[7:] = q_dummy

        # print(self.q)
        self.set_configuration(self.q)


class BipedWrapper(BasicRobotWrapper):

    def __init__(self, q=None):
        super(BipedWrapper, self).__init__()

        self.effs = ["FL", "FR"]
        # self.colors = {"L": "r", "R": "y"}
        self.joints_list = ["HAA", "HFE", "KFE", "ANKLE"]
        self.floor_height = 0.
        self.robot = BoltConfig.buildRobotWrapper()

        self.num_ctrl_joints = 6

        # Create data again after setting frames
        self.model = self.robot.model
        self.data = self.robot.data
        q = pinocchio.neutral(self.robot.model)
        if not q is None:
            self.set_configuration(q)
        else:
            self.q = None
        self.M_com = None
        self.mass = sum([i.mass for i in self.model.inertias[1:]])
        self.set_init_config()

    def set_init_config(self):
        model = self.model
        data = self.data
        NQ = model.nq
        NV = model.nv
        self.q, self.dq, self.ddq, tau = zero(NQ), zero(NV), zero(NV), zero(NV)

        self.q = pinocchio.neutral(self.robot.model)
        self.q[2] = self.floor_height

        # Set initial configuration
        angle = np.deg2rad(60.0)
        q_dummy = np.zeros(self.num_ctrl_joints)
        q_dummy[2::3] = angle
        q_dummy[1:3] = -0.5 * angle

        self.q[7:] = q_dummy

        # print(self.q)
        self.set_configuration(self.q)

class BipedTocabiWrapper(BasicRobotWrapper):

    def __init__(self, q=None):
        super(BipedTocabiWrapper, self).__init__()

        self.effs = ["RF_contact", "LF_contact"]
        # self.colors = {"L": "r", "R": "y"}
        self.joints_list = ["R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint", "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint", "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint", "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint"]
        self.floor_height = 0.0

        self.robot = TocabiConfig.buildRobotWrapper()
        self.num_ctrl_joints = 12

        # Create data again after setting frames
        self.model = self.robot.model

        contactPointLF = pinocchio.SE3.Identity()
        contactPointRF = pinocchio.SE3.Identity()
        
        contactPointLF.translation.T.flat = [0.03, 0, -0.1585]
        contactPointRF.translation.T.flat = [0.03, 0, -0.1585]

        self.LFframe_id = self.model.getFrameId("L_Foot_Link")
        self.RFframe_id = self.model.getFrameId("R_Foot_Link")

        self.RFjoint_id = self.model.getJointId("R_AnkleRoll_Joint")
        self.LFjoint_id = self.model.getJointId("L_AnkleRoll_Joint")

        self.LHjoint_id = self.model.getJointId("L_HipYaw_Joint")
        self.RHjoint_id = self.model.getJointId("R_HipYaw_Joint")

        self.PELVframe_id = self.model.getFrameId("Pelvis_Link")
        self.PELVjoint_id = self.model.getJointId("root_joint")
        
        self.model.addBodyFrame("LF_contact", self.LFjoint_id, contactPointLF, self.LFframe_id)
        self.model.addBodyFrame("RF_contact", self.RFjoint_id, contactPointRF, self.RFframe_id)

        self.LFcframe_id = self.model.getFrameId("LF_contact")
        self.RFcframe_id = self.model.getFrameId("RF_contact")

        self.robot.data = self.model.createData()
        self.data = self.robot.data
        initial_configuration = np.array(
            [0, 0, 0.82473, 0, 0, 0, 1, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]
        )        

        self.q = initial_configuration
        self.qinit = initial_configuration
        if not q is None:
            self.set_configuration(q)
        else:
            self.q = None
        self.M_com = None
        self.PelvJ = pinocchio.utils.zero((6, self.model.nv))
        self.LFcJ = pinocchio.utils.zero((6, self.model.nv))
        self.RFcJ = pinocchio.utils.zero((6, self.model.nv))

        self.mass = sum([i.mass for i in self.model.inertias[1:]])
        self.modelUpdateinit()
        self.inverseKinematics(0.0, self.LF_rot, self.RF_rot, self.PELV_rot, self.LF_tran, self.RF_tran, self.PELV_tran, self.HRR_tran_init, self.HLR_tran_init, self.HRR_rot_init, self.HLR_rot_init, self.PELV_tran_init, self.PELV_rot_init, self.CPELV_tran_init)
        print("pelv")
        print(self.PELV_tran)
        print(self.RF_tran)
        print(self.LF_tran)
        print(self.LFc_tran)
        print(self.leg_q)
        self.set_init_config()

    def modelUpdateinit(self):
        q = pinocchio.randomConfiguration(self.model)
        qdot = pinocchio.utils.zero(self.model.nv)
        q_init = [0, 0, 0.82473, 0, 0, 0, 1, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]
  
        for i in range(0, len(q)):
            q[i] = q_init[i]

        pinocchio.forwardKinematics(self.model, self.data, q, qdot)
        pinocchio.updateFramePlacements(self.model, self.data)
        pinocchio.updateGlobalPlacements(self.model, self.data)
        pinocchio.centerOfMass(self.model, self.data, q, False)
        pinocchio.computeJointJacobians(self.model, self.data)
        self.PelvJ = pinocchio.computeFrameJacobian(self.model, self.data, q, self.PELVframe_id)
        self.RFcJ = pinocchio.computeFrameJacobian(self.model, self.data, q, self.RFcframe_id)
        self.LFcJ = pinocchio.computeFrameJacobian(self.model, self.data, q, self.LFcframe_id)
        self.LF_tran = self.data.oMi[self.LFjoint_id].translation
        self.RF_tran = self.data.oMi[self.RFjoint_id].translation
        self.LFc_tran = self.data.oMf[self.LFcframe_id].translation
        self.RFc_tran = self.data.oMf[self.RFcframe_id].translation
        self.LF_rot = self.data.oMi[self.LFjoint_id].rotation
        self.RF_rot = self.data.oMi[self.RFjoint_id].rotation    
        self.HLR_rot_init = self.data.oMi[self.LHjoint_id].rotation
        self.HRR_rot_init = self.data.oMi[self.RHjoint_id].rotation
        self.HLR_tran_init = self.data.oMi[self.LHjoint_id].translation
        self.HRR_tran_init = self.data.oMi[self.RHjoint_id].translation
        self.PELV_tran = self.data.oMf[self.PELVframe_id].translation
        self.PELV_rot = self.data.oMf[self.PELVframe_id].rotation
        self.PELV_tran_init = np.add(self.data.oMi[self.PELVjoint_id].translation, self.model.inertias[self.PELVjoint_id].lever)
        self.CPELV_tran_init = self.data.oMi[self.PELVjoint_id].translation 
        self.PELV_rot_init = self.data.oMi[self.PELVjoint_id].rotation
        self.PELV_tran = np.add(self.data.oMi[self.PELVjoint_id].translation, self.model.inertias[self.PELVjoint_id].lever)

    def modelUpdate(self, q_, qdot_):
        q = pinocchio.randomConfiguration(self.model)
        qdot = pinocchio.utils.zero(self.model.nv)

        for i in range(0, len(q)):
            q[i] = q_[i]
        for i in range(0, len(qdot)):
            qdot[i] = qdot_[i]
        
        pinocchio.forwardKinematics(self.model, self.data, q, qdot)
        pinocchio.updateFramePlacements(self.model, self.data)
        pinocchio.centerOfMass(self.model, self.data, q, False)
        pinocchio.computeCentroidalMomentum(self.model, self.data, q, qdot)
        self.PelvJ = pinocchio.computeFrameJacobian(self.model, self.data, q, self.PELVframe_id)
        self.RFcJ = pinocchio.computeFrameJacobian(self.model, self.data, q, self.RFcframe_id)
        self.LFcJ = pinocchio.computeFrameJacobian(self.model, self.data, q, self.LFcframe_id)
        self.LF_tran = self.data.oMi[self.LFjoint_id].translation
        self.RF_tran = self.data.oMi[self.RFjoint_id].translation
        self.LFc_tran = self.data.oMf[self.LFcframe_id].translation
        self.RFc_tran = self.data.oMf[self.RFcframe_id].translation
        self.LF_rot = self.data.oMi[self.LFjoint_id].rotation
        self.RF_rot = self.data.oMi[self.RFjoint_id].rotation    
        self.PELV_tran = self.data.oMf[self.PELVframe_id].translation
        self.PELV_rot = self.data.oMf[self.PELVframe_id].rotation
        self.PELV_tran = np.add(self.data.oMi[self.PELVjoint_id].translation, self.model.inertias[self.PELVjoint_id].lever)
        self.hg = self.data.hg

    def set_init_config(self):
        model = self.model
        data = self.data
        NQ = model.nq
        NV = model.nv
        self.q, self.dq, self.ddq, tau = zero(NQ), zero(NV), zero(NV), zero(NV)

#        self.q = pinocchio.neutral(self.robot.model)
        initial_configuration = np.array(
            [0, 0, 0.82473, 0, 0, 0, 1, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]
        )        
        self.q = initial_configuration

        self.set_configuration(self.q)

    def rotateWithY(self, pitch_angle):
        rotate_with_y = np.zeros((3,3))

        rotate_with_y[0, 0] = np.cos(pitch_angle)
        rotate_with_y[1, 0] = 0.0
        rotate_with_y[2, 0] = -1 * np.sin(pitch_angle)

        rotate_with_y[0, 1] = 0.0
        rotate_with_y[1, 1] = 1.0
        rotate_with_y[2, 1] = 0.0

        rotate_with_y[0, 2] = np.sin(pitch_angle)
        rotate_with_y[1, 2] = 0.0
        rotate_with_y[2, 2] = np.cos(pitch_angle)

        return rotate_with_y 

    def rotateWithX(self, roll_angle):
        rotate_with_x = np.zeros((3,3))

        rotate_with_x[0, 0] = 1.0
        rotate_with_x[1, 0] = 0.0
        rotate_with_x[2, 0] = 0.0

        rotate_with_x[0, 1] = 0.0
        rotate_with_x[1, 1] = np.cos(roll_angle)
        rotate_with_x[2, 1] = np.sin(roll_angle)

        rotate_with_x[0, 2] = 0.0
        rotate_with_x[1, 2] = -1 * np.sin(roll_angle)
        rotate_with_x[2, 2] = np.cos(roll_angle)

        return rotate_with_x  

    def rotateWithZ(self, yaw_angle):
        rotate_with_z = np.zeros((3,3))

        rotate_with_z[0, 0] = np.cos(yaw_angle)
        rotate_with_z[1, 0] = np.sin(yaw_angle)
        rotate_with_z[2, 0] = 0.0

        rotate_with_z[0, 1] = -1 * np.sin(yaw_angle)
        rotate_with_z[1, 1] = np.cos(yaw_angle)
        rotate_with_z[2, 1] = 0.0

        rotate_with_z[0, 2] = 0.0
        rotate_with_z[1, 2] = 0.0
        rotate_with_z[2, 2] = 1.0

        return rotate_with_z

    def inverseKinematics(self, time, LF_rot_c, RF_rot_c, PELV_rot_c, LF_tran_c, RF_tran_c, PELV_tran_c, HRR_tran_init_c, HLR_tran_init_c, HRR_rot_init_c, HLR_rot_init_c, PELV_tran_init_c, PELV_rot_init_c, CPELV_tran_init_c):
        M_PI = 3.14159265358979323846
        if time == 0:
            leg_q = np.zeros(12)
            leg_qdot = np.zeros(12)
            leg_qddot = np.zeros(12)
            leg_qs = np.zeros((int(1), 12))
            leg_qdots = np.zeros((int(1), 12))
            leg_qddots = np.zeros((int(1), 12))

        l_upper = 0.35
        l_lower = 0.35

        offset_hip_pitch = 0.0
        offset_knee_pitch = 0.0
        offset_ankle_pitch = 0.0

        lpt = np.subtract(PELV_tran_c, LF_tran_c)
        rpt = np.subtract(PELV_tran_c, RF_tran_c)
        lp = np.matmul(np.transpose(LF_rot_c), lpt)
        rp = np.matmul(np.transpose(RF_rot_c), rpt)
        
        PELF_rot = np.matmul(np.transpose(PELV_rot_c), LF_rot_c)
        PERF_rot = np.matmul(np.transpose(PELV_rot_c), RF_rot_c)

        ld = np.zeros(3)  
        rd = np.zeros(3)

        ld[0] = HLR_tran_init_c[0] - PELV_tran_init_c[0]
        ld[1] = HLR_tran_init_c[1] - PELV_tran_init_c[1]
        ld[2] = -(CPELV_tran_init_c[2] - HLR_tran_init_c[2]) + (CPELV_tran_init_c[2] - PELV_tran_init_c[2])

        rd[0] = HRR_tran_init_c[0] - PELV_tran_init_c[0]
        rd[1] = HRR_tran_init_c[1] - PELV_tran_init_c[1]
        rd[2] = -(CPELV_tran_init_c[2] - HRR_tran_init_c[2]) + (CPELV_tran_init_c[2] - PELV_tran_init_c[2])

        ld = np.matmul(np.transpose(LF_rot_c), ld)
        rd = np.matmul(np.transpose(RF_rot_c), rd)

        lr = np.add(lp, ld)
        rr = np.add(rp, rd)

        lc = np.linalg.norm(lr)

        leg_q[3] = -1 * np.arccos((l_upper * l_upper + l_lower * l_lower - lc * lc) / (2 * l_upper * l_lower)) + M_PI
        l_ankle_pitch = np.arcsin((l_upper * np.sin(M_PI - leg_q[3])) / lc)
        leg_q[4] = -1 * np.arctan2(lr[0], np.sqrt(lr[1] * lr[1] + lr[2] * lr[2])) - l_ankle_pitch
        leg_q[5] = np.arctan2(lr[1], lr[2])

        r_tl2 = np.zeros((3,3))
        r_l2l3 = np.zeros((3,3))
        r_l3l4 = np.zeros((3,3))
        r_l4l5 = np.zeros((3,3))

        r_l2l3 = self.rotateWithY(leg_q[3])
        r_l3l4 = self.rotateWithY(leg_q[4])
        r_l4l5 = self.rotateWithX(leg_q[5])

        r_tl2 = np.matmul(np.matmul(np.matmul(PELF_rot, np.transpose(r_l4l5)),np.transpose(r_l3l4)),np.transpose(r_l2l3))
        leg_q[1] = np.arcsin(r_tl2[2, 1])

        c_lq5 = np.divide(-r_tl2[0, 1], np.cos(leg_q[1]))

        if c_lq5 > 1.0:
            c_lq5 = 1.0
        elif c_lq5 < -1.0:
            c_lq5 = -1.0
        
        leg_q[0] = -1 * np.arcsin(c_lq5)
        leg_q[2] = -1 * np.arcsin(r_tl2[2, 0] / np.cos(leg_q[1])) + offset_hip_pitch
        leg_q[3] = leg_q[3] - offset_knee_pitch
        leg_q[4] = leg_q[4] - offset_ankle_pitch

        rc = np.linalg.norm(rr)
        leg_q[9] = -1 * np.arccos((l_upper * l_upper + l_lower * l_lower - rc * rc) / (2 * l_upper * l_lower)) + M_PI

        r_ankle_pitch = np.arcsin((l_upper * np.sin(M_PI - leg_q[9])) / rc)
        leg_q[10] = -1 * np.arctan2(rr[0], np.sqrt(rr[1] * rr[1] + rr[2] * rr[2])) - r_ankle_pitch
        leg_q[11] = np.arctan2(rr[1], rr[2])
        r_tr2 = np.zeros((3,3))
        r_r2r3 = np.zeros((3,3))
        r_r3r4 = np.zeros((3,3))
        r_r4r5 = np.zeros((3,3))

        r_r2r3 = self.rotateWithY(leg_q[9])
        r_r3r4 = self.rotateWithY(leg_q[10])
        r_r4r5 = self.rotateWithX(leg_q[11])

        r_tr2 = np.matmul(np.matmul(np.matmul(PERF_rot, np.transpose(r_r4r5)),np.transpose(r_r3r4)),np.transpose(r_r2r3))
        leg_q[7] = np.arcsin(r_tr2[2,1])
        c_rq5 = -r_tr2[0, 1] / np.cos(leg_q[7])

        if c_rq5 > 1.0:
            c_rq5 = 1.0
        elif c_rq5 < -1.0:
            c_rq5 = -1.0 
        
        leg_q[6] = -1* np.arcsin(c_rq5)
        leg_q[8] = np.arcsin(r_tr2[2, 0] / np.cos(leg_q[7])) - offset_hip_pitch
        leg_q[9] = -1 * leg_q[9] + offset_knee_pitch
        leg_q[10] = -1 * leg_q[10] + offset_ankle_pitch

        leg_q[0] = leg_q[0] * (-1)
        leg_q[6] = leg_q[6] * (-1)
        leg_q[8] = leg_q[8] * (-1)
        leg_q[9] = leg_q[9] * (-1)
        leg_q[10] = leg_q[10] * (-1)
        self.leg_q = leg_q