/**
 * @file DynamicsOptimizer.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <iomanip>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <momentumopt/dynopt/DynamicsOptimizer.hpp>

using namespace solver;

namespace momentumopt {

  void DynamicsOptimizer::initialize(PlannerSetting& planner_setting)
  {
    planner_setting_ = &planner_setting;

    if (!this->getSetting().get(PlannerBoolParam_UseDefaultSolverSetting)) { model_.configSetting(this->getSetting().get(PlannerStringParam_ConfigFile)); }
    else                                                                   { model_.configSetting(this->getSetting().get(PlannerStringParam_DefaultSolverSettingFile)); }

    friction_cone_.getCone(this->getSetting().get(PlannerDoubleParam_FrictionCoefficient), cone_matrix_);
    dynamicsSequence().resize(this->getSetting().get(PlannerIntParam_NumTimesteps)+1);
    for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps)+1; time_id++) { dynamicsSequence().dynamicsState(time_id).time() = this->getSetting().get(PlannerDoubleParam_TimeStep); }
  }

  void DynamicsOptimizer::initializeOptimizationVariables()
  {
    num_vars_ = 0.;
    double inf_value = SolverSetting::inf;

    // center of mass, linear and angular momentum
    com_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    lmom_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    amom_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    ZMP_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    ZMPd_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);

    // time variable, linear and angular momentum rates
    if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
      dt_.initialize('C', 1, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    }

    lmomd_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    amomd_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);

    // upper and lower bound variables, forces, cops, torques
    for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
      lb_var_[eff_id].initialize('C', 3, dynamicsSequence().activeEndeffectorSteps()[eff_id], -inf_value, inf_value, num_vars_);
      ub_var_[eff_id].initialize('C', 3, dynamicsSequence().activeEndeffectorSteps()[eff_id], -inf_value, inf_value, num_vars_);
    }
  }

  void DynamicsOptimizer::updateTrackingObjective()
  {
    weight_desired_com_tracking_ = this->getSetting().get(PlannerVectorParam_WeightDynamicTrackingCenterOfMass);
    this->getSetting().get(PlannerVectorParam_WeightLinearMomentum) = this->getSetting().get(PlannerVectorParam_WeightDynamicTrackingLinearMomentum);
    this->getSetting().get(PlannerVectorParam_WeightAngularMomentum) = this->getSetting().get(PlannerVectorParam_WeightDynamicTrackingAngularMomentum);
  }

  ExitCode DynamicsOptimizer::optimize(const DynamicsState& ini_state, ContactPlanInterface* contact_plan,
                                       const KinematicsSequence& kin_sequence, bool update_tracking_objective)
  {
    ini_state_ = ini_state;
    std::cout << ini_state_.centerOfMass() << std::endl;
    std::cout << "zmp11" << std::endl;
    std::cout << ini_state_.ZMP() << std::endl;
    contact_plan_ = contact_plan;
    weight_desired_com_tracking_.setZero();
    //com_pos_goal_ = ini_state_.centerOfMass() + this->getSetting().get(PlannerVectorParam_CenterOfMassMotion);
    com_pos_goal_<< 0.2500, 0.0, ini_state_.centerOfMass()(2) - 0.08;
    int timey = 0;
    int timex = 0;
    com_pos_goal_(0) = com_pos_goal_(0) + timex * 0.13/50;
    com_pos_goal_(1) = com_pos_goal_(1) + timey * 0.205/50;
    com_pos_goal_(2) = com_pos_goal_(2);
    std::cout << "com_pos_goal" << com_pos_goal_.transpose() << std::endl;

    std::cout << "lmom" << ini_state_.linearMomentum().transpose() << std::endl;
    
    contact_plan_->fillDynamicsSequence(ini_state, this->dynamicsSequence());
    this->initializeOptimizationVariables();

  if (update_tracking_objective)
    this->updateTrackingObjective();

  solve_time_ = 0.0;
  has_converged_ = false;
  internalOptimize(kin_sequence, true);

  for(int i=0; i < this->getSetting().get(PlannerIntParam_NumTimesteps); i ++)
  {
   
  }
  //this->saveToFile(kin_sequence);
    return exitcode_;
  }

  void DynamicsOptimizer::internalOptimize(const KinematicsSequence& kin_sequence, bool is_first_time)
  {
    try
    {
      // add variables to model
      vars_.clear();
      for (int var_id=0; var_id<num_vars_; var_id++)
        vars_.push_back(Var());

      model_.clean();
      addVariableToModel(com_, model_, vars_);
      addVariableToModel(lmom_, model_, vars_);
      addVariableToModel(amom_, model_, vars_);
      addVariableToModel(ZMP_, model_, vars_);
      addVariableToModel(ZMPd_, model_, vars_);
      addVariableToModel(lmomd_, model_, vars_);
      addVariableToModel(amomd_, model_, vars_);

      /*if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
          addVariableToModel(dt_, model_, vars_);
      }*/

      for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
        addVariableToModel(lb_var_[eff_id], model_, vars_);
        addVariableToModel(ub_var_[eff_id], model_, vars_);
      }
      // adding quadratic objective
      quad_objective_.clear();


      double ZMP_ux, ZMP_lx, ZMP_uy, ZMP_ly;
      std::fstream file1;
      file1.open("/home/jhk/walkingdata1/stairdown/25cm/ssp2/timestep=2/timestep0_zmp3_ssp1_1.txt",std::ios_base::out);    
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
        for (int axis_id=0; axis_id<3; axis_id++) {
          // penalty on center of mass, linear and angular momentum with Kinematics
          if (time_id==this->getSetting().get(PlannerIntParam_NumTimesteps)-1) {
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightCenterOfMass)[axis_id], LinExpr(vars_[com_.id(axis_id,time_id)]) - LinExpr(com_pos_goal_[axis_id]));
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightFinalLinearMomentum)[axis_id], LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(kin_sequence.kinematicsState(time_id).linearMomentum()[axis_id]));
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightFinalAngularMomentum)[axis_id], LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(kin_sequence.kinematicsState(time_id).angularMomentum()[axis_id]));
          } else {
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightLinearMomentum)[axis_id], LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(kin_sequence.kinematicsState(time_id).linearMomentum()[axis_id]));
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightAngularMomentum)[axis_id], LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(kin_sequence.kinematicsState(time_id).angularMomentum()[axis_id]));
            quad_objective_.addQuaTerm(weight_desired_com_tracking_[axis_id], vars_[com_.id(axis_id,time_id)] - kin_sequence.kinematicsState(time_id).centerOfMass()[axis_id]);
          }
          
          
          if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
          } else {
            if(axis_id != 2)
            {
              quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightZMP)[axis_id], LinExpr(vars_[ZMPd_.id(axis_id,time_id)]));
            }
          }
        }

        

        bool ZMP_l = false;
        bool ZMP_r = false; 
      
        for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
          if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
            if(eff_id == 0)
            {
              ZMP_l = true;
            }
            if(eff_id == 1)
            {
              ZMP_r = true;
            }
          }
        }

        if(ZMP_l == true && ZMP_r == true)
        {
          ZMP_lx = (this->contactLocation(time_id, 0, 0) + this->contactLocation(time_id, 1, 0))/2;
          ZMP_ly = (this->contactLocation(time_id, 0, 1) + this->contactLocation(time_id, 1, 1))/2;
        }
        else if(ZMP_l == true)
        {
          ZMP_lx = this->contactLocation(time_id, 0, 0);
          ZMP_ly = this->contactLocation(time_id, 0, 1);
        }
        else if(ZMP_r == true)
        {
          ZMP_lx = this->contactLocation(time_id, 1, 0);
          ZMP_ly = this->contactLocation(time_id, 1, 1);
        }

            
        // penalty on linear and angular momentum rate with Kinematics
        for (int axis_id = 0; axis_id < 3; axis_id++)
        {
          if (time_id==0) {
          quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightLinearMomentumRate)[axis_id], (LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(ini_state_.linearMomentum()[axis_id]))*(1.0/dynamicsSequence().dynamicsState(time_id).time()));
          quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightAngularMomentumRate)[axis_id], (LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(ini_state_.angularMomentum()[axis_id]))*(1.0/dynamicsSequence().dynamicsState(time_id).time()));
          } else {
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightLinearMomentumRate)[axis_id], (LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(vars_[lmom_.id(axis_id,time_id-1)]))*(1.0/dynamicsSequence().dynamicsState(time_id).time()));
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightAngularMomentumRate)[axis_id], (LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(vars_[amom_.id(axis_id,time_id-1)]))*(1.0/dynamicsSequence().dynamicsState(time_id).time()));
          }
        }
      }
      
      
      bool zmp_bool = true;
      bool zmp_double = false;
      std::fstream file;
      file.open("/home/jhk/walkingdata1/stairdown/25cm/ssp2/timestep=2/timestep0_zmp2_ssp1_1.txt",std::ios_base::out);
      // center of mass above endeffector positions, ZMP constraint
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
        if(dynamicsSequence().dynamicsState(time_id).endeffectorActivation(0) == true && dynamicsSequence().dynamicsState(time_id).endeffectorActivation(1) == true)
        {
          zmp_double = true;
        }
        else
        {
          zmp_double = false;
        }

        
        if(zmp_double == true)// && 0.2/this->getSetting().get(PlannerDoubleParam_TimeStep)>time_id)
        {
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
              if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
                  if(ZMP_lx - this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][0] - this->contactLocation(time_id, eff_id, 0) > 0)
                  {
                    ZMP_lx = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][0] + this->contactLocation(time_id, eff_id, 0);
                  }
                  if(ZMP_ux - this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][1]-this->contactLocation(time_id, eff_id, 0) < 0)
                  {
                    ZMP_ux = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][1] + this->contactLocation(time_id, eff_id, 0);
                  }
                  if(ZMP_ly - this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][2]-this->contactLocation(time_id, eff_id, 1) > 0)
                  {
                    ZMP_ly = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][2] + this->contactLocation(time_id, eff_id, 1);
                  }
                  if(ZMP_uy - this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][3] -this->contactLocation(time_id, eff_id, 1)< 0)
                  {
                    ZMP_uy = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][3] + this->contactLocation(time_id, eff_id, 1);
                  }
                }
              }
        }
        else if(dynamicsSequence().dynamicsState(time_id).endeffectorActivation(0) == true)
        {
            ZMP_lx = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[0][0] + this->contactLocation(time_id, 0, 0);
            ZMP_ux = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[0][1] + this->contactLocation(time_id, 0, 0);
            ZMP_ly = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[0][2] + this->contactLocation(time_id, 0, 1);
            ZMP_uy = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[0][3] + this->contactLocation(time_id, 0, 1);
        }
        else if(dynamicsSequence().dynamicsState(time_id).endeffectorActivation(1) == true)
        {
            ZMP_lx = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[1][0] + this->contactLocation(time_id, 1, 0);
            ZMP_ux = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[1][1] + this->contactLocation(time_id, 1, 0);
            ZMP_ly = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[1][2] + this->contactLocation(time_id, 1, 1);
            ZMP_uy = this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[1][3] + this->contactLocation(time_id, 1, 1);
        }
        
      
        if(time_id >= 0)
        {
          double zmpx, zmpy;
          zmpx = (ZMP_lx + ZMP_ux)/2;
          zmpy = (ZMP_ly + ZMP_uy)/2;
          if(zmp_double == false)
          {
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightZMPC)[0], (LinExpr(vars_[ZMP_.id(0,time_id)]) - LinExpr(zmpx)));
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightZMPC)[1], (LinExpr(vars_[ZMP_.id(1,time_id)]) - LinExpr(zmpy)));
          }
          if(zmp_double == true)
          {
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightZMPC)[0]/10, (LinExpr(vars_[ZMP_.id(0,time_id)]) - LinExpr(zmpx)));
            quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightZMPC)[1]/10, (LinExpr(vars_[ZMP_.id(1,time_id)]) - LinExpr(zmpy)));
          }
          file1 << zmpx << " "<<zmpy << std::endl;
        }
        
        // ZMP constraint
        file << ZMP_lx << " "<< ZMP_ux << " "<<ZMP_ly << " " << ZMP_uy << std::endl;
        model_.addLinConstr(LinExpr(vars_[ZMP_.id(0,time_id)]) - LinExpr(ZMP_lx), ">", 0.0);//see temp
        model_.addLinConstr(LinExpr(vars_[ZMP_.id(0,time_id)]) - LinExpr(ZMP_ux), "<", 0.0);
        model_.addLinConstr(LinExpr(vars_[ZMP_.id(1,time_id)]) - LinExpr(ZMP_ly), ">", 0.0);//see temp
        model_.addLinConstr(LinExpr(vars_[ZMP_.id(1,time_id)]) - LinExpr(ZMP_uy), "<", 0.0);
       
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
          model_.addLinConstr(LinExpr(vars_[com_.id(2,time_id)]) - LinExpr(this->contactLocation(0, 0, 2)), ">", this->getSetting().get(PlannerDoubleParam_MinRelHeight));
          model_.addLinConstr(LinExpr(vars_[com_.id(2,time_id)]) - LinExpr(this->contactLocation(0, 0, 2)), "<", ini_state_.centerOfMass()(2)+0.2);
          //for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
          //  if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
          //      model_.addLinConstr(vars_[com_.id(2,time_id)] - this->contactLocation(time_id, eff_id, 2), ">", this->getSetting().get(PlannerDoubleParam_MinRelHeight));
          //  }
          //}
        }
        // Dynamics MODEL
        
        lin_cons_ = 0;
        if(time_id == 0)
        {
          //std::cout << "TImeID " << dynamicsSequence().dynamicsState(time_id).centerOfMass()[0] << std::endl;
          /*double  comddz, comz, zmpz, comx, zmpx, angy, w_1, w_2; 
          comddz = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[2]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          comx = dynamicsSequence().dynamicsState(time_id).centerOfMass()[0];
          zmpz = dynamicsSequence().dynamicsState(time_id).ZMP()[2];
          comz = dynamicsSequence().dynamicsState(time_id).centerOfMass()[2];
          zmpx = dynamicsSequence().dynamicsState(time_id).ZMP()[0];
          angy = dynamicsSequence().dynamicsState(time_id).angularMomentumRate()[1];
          
          w_1 = (9.81+comddz)/(comz-zmpz);
          w_2 = zmpx - comx + angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(9.81+comddz));
          lin_cons_ += w_1 * (LinExpr(vars_[com_.id(0,time_id)]) - comx) - w_1 *(LinExpr(vars_[ZMP_.id(0,time_id)]) - zmpx) - w_1*w_2 + (comx - zmpx)/(comz-zmpz) * (LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - comddz) - (LinExpr(vars_[amomd_.id(1,time_id)]) - angy)/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(comz - zmpz));
          lin_cons_ += w_1/(comz-zmpz) *(LinExpr(vars_[com_.id(2,time_id)])-comz)*w_2 - w_1/(comz-zmpz)*w_2*(LinExpr(vars_[ZMP_.id(2,time_id)])-zmpz);
          //lin_cons_ += (LinExpr(vars_[lmomd_.id(0,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - LinExpr(ini_state_.linearMomentumRate()[0])/this->getSetting().get(PlannerDoubleParam_RobotMass)) - (9.81 + comddz)/(comz-zmpz) * (LinExpr(vars_[com_.id(0,time_id)]) - LinExpr(ini_state_.centerOfMass()[0]) - (LinExpr(vars_[ZMP_.id(0,time_id)]) - LinExpr(ini_state_.ZMP()[0]))) - (LinExpr(vars_[amomd_.id(1,time_id)])-LinExpr(ini_state_.angularMomentumRate()[1]))/(this->getSetting().get(PlannerDoubleParam_RobotMass)*(comz-zmpz));
          //lin_cons_ += 1/(comz-zmpz)*(comx-zmpx-angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz))) *(LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - LinExpr(ini_state_.linearMomentumRate()[2])/this->getSetting().get(PlannerDoubleParam_RobotMass));
          //lin_cons_ += -(9.81 + comddz)/((comz-zmpz)*(comz-zmpz))*(comx-zmpx-angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz)))*(LinExpr(vars_[com_.id(2,time_id)]) - LinExpr(ini_state_.centerOfMass()[2]) - (LinExpr(vars_[ZMP_.id(2,time_id)]) - LinExpr(ini_state_.ZMP()[2])));
          model_.addLinConstr(lin_cons_, "=", 0.0);
          //((g + z_c_ddot0)*(x_c - x_c0))/(z_c0 - z_p0) - ((x_p0 - x_c0 + tau_y0/(m*(g + z_c_ddot0)))/(z_c0 - z_p0) - tau_y0/(m*(g + z_c_ddot0)*(z_c0 - z_p0)))*(z_c_ddot - z_c_ddot0) - ((g + z_c_ddot0)*(x_p - x_p0))/(z_c0 - z_p0) - ((g + z_c_ddot0)*(x_p0 - x_c0 + tau_y0/(m*(g + z_c_ddot0))))/(z_c0 - z_p0) - (tau_y - tau_y0)/(m*(z_c0 - z_p0)) + ((g + z_c_ddot0)*(z_c - z_c0)*(x_p0 - x_c0 + tau_y0/(m*(g + z_c_ddot0))))/(z_c0 - z_p0)^2 - ((g + z_c_ddot0)*(z_p - z_p0)*(x_p0 - x_c0 + tau_y0/(m*(g + z_c_ddot0))))/(z_c0 - z_p0)^2
          */

          double  comddz, comz, zmpz, comx, zmpx, angy, w_1, w_2, comddx; 
          comddz = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[2]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          comddx = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[0]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          
          comx = dynamicsSequence().dynamicsState(time_id).centerOfMass()[0];
          zmpz = dynamicsSequence().dynamicsState(time_id).ZMP()[2];
          comz = dynamicsSequence().dynamicsState(time_id).centerOfMass()[2];
          zmpx = dynamicsSequence().dynamicsState(time_id).ZMP()[0];
          angy = dynamicsSequence().dynamicsState(time_id).angularMomentumRate()[1];
          
          //lin_cons_ += (LinExpr(vars_[lmomd_.id(0,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - LinExpr(vars_[lmomd_.id(0,time_id-1)])/this->getSetting().get(PlannerDoubleParam_RobotMass)) - (9.81 + comddz)/(comz-zmpz) * (LinExpr(vars_[com_.id(0,time_id)]) - LinExpr(vars_[com_.id(0,time_id-1)])- (LinExpr(vars_[ZMP_.id(0,time_id)]) - LinExpr(vars_[ZMP_.id(0,time_id-1)]))) - (LinExpr(vars_[amomd_.id(1,time_id)])-LinExpr(vars_[amomd_.id(1,time_id-1)]))/(this->getSetting().get(PlannerDoubleParam_RobotMass)*(comz-zmpz));
          //lin_cons_ += 1/(comz-zmpz)*(comx-zmpx-angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz))) *(LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - LinExpr(vars_[lmomd_.id(2,time_id-1)])/this->getSetting().get(PlannerDoubleParam_RobotMass));
          //lin_cons_ += -(9.81 + comddz)/((comz-zmpz)*(comz-zmpz))*(comx-zmpx-angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz)))*(LinExpr(vars_[com_.id(2,time_id)]) - LinExpr(vars_[com_.id(2,time_id-1)]) - (LinExpr(vars_[ZMP_.id(2,time_id)]) - LinExpr(vars_[ZMP_.id(2,time_id-1)])));
          w_1 = (9.81+comddz)/(comz-zmpz);
          w_2 = zmpx - comx + angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(9.81+comddz));
          lin_cons_ += (LinExpr(vars_[lmomd_.id(0,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass)) - comddx -  w_1 * (LinExpr(vars_[com_.id(0,time_id)]) - comx) + w_1 *(LinExpr(vars_[ZMP_.id(0,time_id)]) - zmpx) + w_1*w_2 - (comx - zmpx)/(comz-zmpz) * (LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - comddz) + (LinExpr(vars_[amomd_.id(1,time_id)]) - angy)/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(comz - zmpz));
          lin_cons_ += -w_1/(comz-zmpz) *(LinExpr(vars_[com_.id(2,time_id)])-comz)*w_2 + w_1/(comz-zmpz)*w_2*(LinExpr(vars_[ZMP_.id(2,time_id)])-zmpz);
         
          model_.addLinConstr(lin_cons_, "=", 0.0);
        }
        
        else
        {
          double  comddz, comz, zmpz, comx, zmpx, angy, w_1, w_2, comddx; 
          comddz = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[2]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          comddx = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[0]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          
          comx = dynamicsSequence().dynamicsState(time_id).centerOfMass()[0];
          zmpz = dynamicsSequence().dynamicsState(time_id).ZMP()[2];
          comz = dynamicsSequence().dynamicsState(time_id).centerOfMass()[2];
          zmpx = dynamicsSequence().dynamicsState(time_id).ZMP()[0];
          angy = dynamicsSequence().dynamicsState(time_id).angularMomentumRate()[1];
          
          //lin_cons_ += (LinExpr(vars_[lmomd_.id(0,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - LinExpr(vars_[lmomd_.id(0,time_id-1)])/this->getSetting().get(PlannerDoubleParam_RobotMass)) - (9.81 + comddz)/(comz-zmpz) * (LinExpr(vars_[com_.id(0,time_id)]) - LinExpr(vars_[com_.id(0,time_id-1)])- (LinExpr(vars_[ZMP_.id(0,time_id)]) - LinExpr(vars_[ZMP_.id(0,time_id-1)]))) - (LinExpr(vars_[amomd_.id(1,time_id)])-LinExpr(vars_[amomd_.id(1,time_id-1)]))/(this->getSetting().get(PlannerDoubleParam_RobotMass)*(comz-zmpz));
          //lin_cons_ += 1/(comz-zmpz)*(comx-zmpx-angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz))) *(LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - LinExpr(vars_[lmomd_.id(2,time_id-1)])/this->getSetting().get(PlannerDoubleParam_RobotMass));
          //lin_cons_ += -(9.81 + comddz)/((comz-zmpz)*(comz-zmpz))*(comx-zmpx-angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz)))*(LinExpr(vars_[com_.id(2,time_id)]) - LinExpr(vars_[com_.id(2,time_id-1)]) - (LinExpr(vars_[ZMP_.id(2,time_id)]) - LinExpr(vars_[ZMP_.id(2,time_id-1)])));
          w_1 = (9.81+comddz)/(comz-zmpz);
          w_2 = zmpx - comx + angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(9.81+comddz));
          lin_cons_ += (LinExpr(vars_[lmomd_.id(0,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass)) - comddx -  w_1 * (LinExpr(vars_[com_.id(0,time_id)]) - comx) + w_1 *(LinExpr(vars_[ZMP_.id(0,time_id)]) - zmpx) + w_1*w_2 - (comx - zmpx)/(comz-zmpz) * (LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - comddz) + (LinExpr(vars_[amomd_.id(1,time_id)]) - angy)/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(comz - zmpz));
          lin_cons_ += -w_1/(comz-zmpz) *(LinExpr(vars_[com_.id(2,time_id)])-comz)*w_2 + w_1/(comz-zmpz)*w_2*(LinExpr(vars_[ZMP_.id(2,time_id)])-zmpz);
         
          model_.addLinConstr(lin_cons_, "=", 0.0);
        } 
        
        lin_cons_ = 0;
        if(time_id == 0)
        {
          /*double  comddz, comz, zmpz, comy, zmpy, angx,  w_1, w_2;
          comddz = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[2]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          comy = dynamicsSequence().dynamicsState(time_id).centerOfMass()[1];
          zmpz = dynamicsSequence().dynamicsState(time_id).ZMP()[2];
          comz = dynamicsSequence().dynamicsState(time_id).centerOfMass()[2];
          zmpy = dynamicsSequence().dynamicsState(time_id).ZMP()[1];
          angx = dynamicsSequence().dynamicsState(time_id).angularMomentumRate()[0];
          w_1 = (9.81+comddz)/(comz-zmpz);
          w_2 = -zmpy + comy + angx/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(9.81+comddz));
          lin_cons_ += w_1 * (LinExpr(vars_[com_.id(1,time_id)]) - comy) - w_1 *(LinExpr(vars_[ZMP_.id(1,time_id)]) - zmpy) + w_1*w_2 + (comy - zmpy)/(comz-zmpz) * (LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - comddz) + (LinExpr(vars_[amomd_.id(0,time_id)]) - angx)/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(comz - zmpz));
          lin_cons_ += -w_1/(comz-zmpz) *(LinExpr(vars_[com_.id(2,time_id)])-comz)*w_2 + w_1/(comz-zmpz)*w_2*(LinExpr(vars_[ZMP_.id(2,time_id)])-zmpz);
         
          //lin_cons_ += (LinExpr(vars_[lmomd_.id(1,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - LinExpr(ini_state_.linearMomentumRate()[1])/this->getSetting().get(PlannerDoubleParam_RobotMass)) - (9.81 + comddz)/(comz-zmpz) * (LinExpr(vars_[com_.id(1,time_id)]) - LinExpr(ini_state_.centerOfMass()[1]) - (LinExpr(vars_[ZMP_.id(1,time_id)]) - LinExpr(ini_state_.ZMP()[1]))) + (LinExpr(vars_[amomd_.id(0,time_id)])-LinExpr(ini_state_.angularMomentumRate()[0]))/(this->getSetting().get(PlannerDoubleParam_RobotMass)*(comz-zmpz));
          //lin_cons_ += 1/(comz-zmpz)*(comy-zmpy+angx/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz))) *(LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - LinExpr(ini_state_.linearMomentumRate()[2])/this->getSetting().get(PlannerDoubleParam_RobotMass));
          //lin_cons_ += -(9.81 + comddz)/((comz-zmpz)*(comz-zmpz))*(comy-zmpy+angx/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz)))*(LinExpr(vars_[com_.id(2,time_id)]) - LinExpr(ini_state_.centerOfMass()[2]) - (LinExpr(vars_[ZMP_.id(2,time_id)]) - LinExpr(ini_state_.ZMP()[2])));
          model_.addLinConstr(lin_cons_, "=", 0.0);*/
          
          double  comddz, comz, zmpz, comy, zmpy, angx,  w_1, w_2, comddy;
          comddz = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[2]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          comddy = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[1]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          
          comy = dynamicsSequence().dynamicsState(time_id).centerOfMass()[1];
          zmpz = dynamicsSequence().dynamicsState(time_id).ZMP()[2];
          comz = dynamicsSequence().dynamicsState(time_id).centerOfMass()[2];
          zmpy = dynamicsSequence().dynamicsState(time_id).ZMP()[1];
          angx = dynamicsSequence().dynamicsState(time_id).angularMomentumRate()[0];

          w_1 = (9.81+comddz)/(comz-zmpz);
          w_2 = -zmpy + comy + angx/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(9.81+comddz));
          lin_cons_ +=  (LinExpr(vars_[lmomd_.id(1,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass)) - comddy - w_1 * (LinExpr(vars_[com_.id(1,time_id)]) - comy) + w_1 *(LinExpr(vars_[ZMP_.id(1,time_id)]) - zmpy) - w_1*w_2 - (comy - zmpy)/(comz-zmpz) * (LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - comddz) - (LinExpr(vars_[amomd_.id(0,time_id)]) - angx)/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(comz - zmpz));
          lin_cons_ += w_1/(comz-zmpz) *(LinExpr(vars_[com_.id(2,time_id)])-comz)*w_2 - w_1/(comz-zmpz)*w_2*(LinExpr(vars_[ZMP_.id(2,time_id)])-zmpz);
         
          model_.addLinConstr(lin_cons_, "=", 0.0);
        }
        else
        {
          double  comddz, comz, zmpz, comy, zmpy, angx,  w_1, w_2, comddy;
          comddz = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[2]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          comddy = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[1]/this->getSetting().get(PlannerDoubleParam_RobotMass);
          
          comy = dynamicsSequence().dynamicsState(time_id).centerOfMass()[1];
          zmpz = dynamicsSequence().dynamicsState(time_id).ZMP()[2];
          comz = dynamicsSequence().dynamicsState(time_id).centerOfMass()[2];
          zmpy = dynamicsSequence().dynamicsState(time_id).ZMP()[1];
          angx = dynamicsSequence().dynamicsState(time_id).angularMomentumRate()[0];
          w_1 = (9.81+comddz)/(comz-zmpz);
          w_2 = -zmpy + comy + angx/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(9.81+comddz));
          lin_cons_ +=  (LinExpr(vars_[lmomd_.id(1,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass)) - comddy - w_1 * (LinExpr(vars_[com_.id(1,time_id)]) - comy) + w_1 *(LinExpr(vars_[ZMP_.id(1,time_id)]) - zmpy) - w_1*w_2 - (comy - zmpy)/(comz-zmpz) * (LinExpr(vars_[lmomd_.id(2,time_id)])/this->getSetting().get(PlannerDoubleParam_RobotMass) - comddz) - (LinExpr(vars_[amomd_.id(0,time_id)]) - angx)/(this->getSetting().get(PlannerDoubleParam_RobotMass) *(comz - zmpz));
          lin_cons_ += w_1/(comz-zmpz) *(LinExpr(vars_[com_.id(2,time_id)])-comz)*w_2 - w_1/(comz-zmpz)*w_2*(LinExpr(vars_[ZMP_.id(2,time_id)])-zmpz);
         
          model_.addLinConstr(lin_cons_, "=", 0.0);
        }
        
      }
      model_.setObjective(quad_objective_, 0.0);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) 
      {
        for (int axis_id=0; axis_id<3; axis_id++) {
          if (time_id==0) { lin_cons_ = LinExpr(vars_[ZMP_.id(axis_id,time_id)]) - LinExpr(ini_state_.ZMP()[axis_id]) - LinExpr(vars_[ZMPd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time();}
          else            { lin_cons_ = LinExpr(vars_[ZMP_.id(axis_id,time_id)]) - LinExpr(vars_[ZMP_.id(axis_id,time_id-1)])  - LinExpr(vars_[ZMPd_.id(axis_id,time_id)])*(dynamicsSequence().dynamicsState(time_id).time()); }
          model_.addLinConstr(lin_cons_, "=", 0.0);
        }

        for (int axis_id=2; axis_id<3; axis_id++) {
          if (time_id==0) { lin_cons_ = LinExpr(vars_[ZMP_.id(axis_id,time_id)]);}
          else            { lin_cons_ = LinExpr(vars_[ZMP_.id(axis_id,time_id)]); }
          model_.addLinConstr(lin_cons_, "=", 0.0);
        }

        // linear momentum constraint
        for (int axis_id=0; axis_id<3; axis_id++) {
          if (time_id==0) { lin_cons_ = LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(ini_state_.linearMomentum()[axis_id]) - LinExpr(vars_[lmomd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time(); }
          else            { lin_cons_ = LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(vars_[lmom_.id(axis_id,time_id-1)])   - LinExpr(vars_[lmomd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time(); }
          model_.addLinConstr(lin_cons_, "=", 0.0);
        }

        // angular momentum constraint
        for (int axis_id=0; axis_id<3; axis_id++) {
          if (time_id==0) { lin_cons_ = LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(ini_state_.angularMomentum()[axis_id]) - LinExpr(vars_[amomd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time(); }
          else            { lin_cons_ = LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(vars_[amom_.id(axis_id,time_id-1)])    - LinExpr(vars_[amomd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time(); }
          model_.addLinConstr(lin_cons_, "=", 0.0);
        }

        for (int axis_id=0; axis_id<3; axis_id++) {
          if (time_id==0) { lin_cons_ = LinExpr(vars_[com_.id(axis_id,time_id)]) - LinExpr(ini_state_.centerOfMass()[axis_id]) - LinExpr(vars_[lmom_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time()/this->getSetting().get(PlannerDoubleParam_RobotMass); }
          else            { lin_cons_ = LinExpr(vars_[com_.id(axis_id,time_id)]) - LinExpr(vars_[com_.id(axis_id,time_id-1)])  - LinExpr(vars_[lmom_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time()/this->getSetting().get(PlannerDoubleParam_RobotMass); }
          model_.addLinConstr(lin_cons_, "=", 0.0);
        }

        model_.addLinConstr(lin_cons_, "=", 0.0);
      }

    QuadConstrApprox qapprox = QuadConstrApprox::None;
      switch (this->getSetting().heuristic()) {
        case Heuristic::TimeOptimization: { qapprox = QuadConstrApprox::None; break; }
        case Heuristic::TrustRegion: { qapprox = QuadConstrApprox::TrustRegion; break; }
        case Heuristic::SoftConstraint: { qapprox = QuadConstrApprox::SoftConstraint; break; }
        default: { qapprox = QuadConstrApprox::None; break; }
    }

      // formulate problem in standard conic form and solve it
      timer_.start();
      exitcode_ = model_.optimize();
      solve_time_ += timer_.stop();

      // extract solution
      solution_.resize(num_vars_, 1);  solution_.setZero();
      for (int var_id=0; var_id<num_vars_; var_id++) { solution_(var_id) = vars_[var_id].get(SolverDoubleParam_X); }

    }
    catch(...)
    {
      std::cout << "Exception during optimization" << std::endl;
    }

    saveSolution(com_);
    saveSolution(amom_);
    saveSolution(lmom_);
    saveSolution(ZMP_);
    saveSolution(ZMPd_);
    saveSolution(lmomd_);
    saveSolution(amomd_);
    
    // computation of angular momentum out of forces and lengths
    for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
      
    }
    storeSolution();
  }

  void DynamicsOptimizer::addVariableToModel(const OptimizationVariable& opt_var, Model& model, std::vector<Var>& vars)
  {
    opt_var.getValues(mat_lb_, mat_ub_, mat_guess_, size_, variable_type_);
    for (int col_id=0; col_id<opt_var.getNumCols(); col_id++)
      for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
        switch (variable_type_) {
          case 'C': { vars[opt_var.id(row_id,col_id)] = model.addVar(VarType::Continuous, double(mat_lb_(row_id,col_id)), double(mat_ub_(row_id,col_id)), double(mat_guess_(row_id,col_id))); break; }
          default: { throw std::runtime_error("At add_var_to_model, variable type not handled"); }
        }

    for (int col_id=0; col_id<opt_var.getNumCols(); col_id++)
      for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
        vars[opt_var.id(row_id,col_id)].set(SolverDoubleParam_X, mat_guess_(row_id,col_id));
  }

  void DynamicsOptimizer::saveSolution(OptimizationVariable& opt_var)
  {
  mat_guess_.resize(opt_var.getNumRows(), opt_var.getNumCols()); mat_guess_.setZero();
        for (int col_id=0; col_id<opt_var.getNumCols(); col_id++) {
      for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
          mat_guess_(row_id,col_id) = solution_(opt_var.id(row_id,col_id));
    }
    opt_var.setGuessValue(mat_guess_);
  }

  void DynamicsOptimizer::storeSolution()
  {
    dynamicsSequence().dynamicsState(0).centerOfMass() = ini_state_.centerOfMass();
    dynamicsSequence().dynamicsState(0).linearMomentum() = ini_state_.linearMomentum();
   // dynamicsSequence().dynamicsState(0).linearMomentumRate() = ini_state_.linearMomentumRate();
    dynamicsSequence().dynamicsState(0).angularMomentum() = ini_state_.angularMomentum();
   // dynamicsSequence().dynamicsState(0).angularMomentumRate() = ini_state_.angularMomentumRate();
    dynamicsSequence().dynamicsState(0).ZMP() = ini_state_.ZMP();
   // dynamicsSequence().dynamicsState(0).ZMPd() = ini_state_.ZMPd();
    //DynamicsSequence().dynamicsState(0).endeffectorCoP(0)  = ini_state_.ZMP();
    //std::cout <<"zmp" << std::endl;
    //std::cout << ini_state_.ZMP() << std::endl;
    com_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        dynamicsSequence().dynamicsState(time_id+1).centerOfMass() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));

    lmom_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        dynamicsSequence().dynamicsState(time_id+1).linearMomentum() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));

    lmomd_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        dynamicsSequence().dynamicsState(time_id).linearMomentumRate() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));

    amomd_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        dynamicsSequence().dynamicsState(time_id).angularMomentumRate() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));

    amom_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        dynamicsSequence().dynamicsState(time_id+1).angularMomentum() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));

    ZMP_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
      {
        dynamicsSequence().dynamicsState(time_id+1).endeffectorCoP(0) = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));
        dynamicsSequence().dynamicsState(time_id+1).ZMP() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));
      } 
    ZMPd_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
      {
        dynamicsSequence().dynamicsState(time_id).ZMPd() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));
      } 
    for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps)+1; time_id++)
    {
      /*std::cout << time_id << std::endl;
      std::cout << dynamicsSequence().dynamicsState(time_id).centerOfMass().transpose() <<std::endl;
      std::cout << dynamicsSequence().dynamicsState(time_id).linearMomentumRate().transpose() << std::endl;
      double  comddz, comz, zmpz, comx, zmpx, angy;
      
      comddz = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[2]/this->getSetting().get(PlannerDoubleParam_RobotMass);
      comx = dynamicsSequence().dynamicsState(time_id).centerOfMass()[0];
      zmpz = dynamicsSequence().dynamicsState(time_id).ZMP()[2];
      comz = dynamicsSequence().dynamicsState(time_id).centerOfMass()[2];
      zmpx = dynamicsSequence().dynamicsState(time_id).ZMP()[0];
      angy = dynamicsSequence().dynamicsState(time_id).angularMomentumRate()[1];
      double a = 0;
      a += (dynamicsSequence().dynamicsState(time_id).linearMomentumRate()(0)/this->getSetting().get(PlannerDoubleParam_RobotMass) - dynamicsSequence().dynamicsState(time_id-1).linearMomentumRate()(0)/this->getSetting().get(PlannerDoubleParam_RobotMass)) - (9.81 + comddz)/(comz-zmpz) * (dynamicsSequence().dynamicsState(time_id).centerOfMass()(0) - dynamicsSequence().dynamicsState(time_id-1).centerOfMass()(0)- (dynamicsSequence().dynamicsState(time_id).ZMP()(0) - dynamicsSequence().dynamicsState(time_id-1).ZMP()(0))) - (dynamicsSequence().dynamicsState(time_id).angularMomentumRate()(1)-dynamicsSequence().dynamicsState(time_id-1).angularMomentumRate()(0))/(this->getSetting().get(PlannerDoubleParam_RobotMass)*(comz-zmpz));
      a += 1/(comz-zmpz)*(comx-zmpx-angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz))) *(dynamicsSequence().dynamicsState(time_id).linearMomentumRate()(2)/this->getSetting().get(PlannerDoubleParam_RobotMass) - dynamicsSequence().dynamicsState(time_id-1).linearMomentumRate()(2)/this->getSetting().get(PlannerDoubleParam_RobotMass));
      a += -(9.81 + comddz)/((comz-zmpz)*(comz-zmpz))*(comx-zmpx-angy/(this->getSetting().get(PlannerDoubleParam_RobotMass) * (9.81+comddz)))*(dynamicsSequence().dynamicsState(time_id).centerOfMass()(2) - dynamicsSequence().dynamicsState(time_id-1).centerOfMass()(0) - (dynamicsSequence().dynamicsState(time_id).ZMP()(2) - dynamicsSequence().dynamicsState(time_id-1).ZMP()(2)));
      
      std::cout << a << std::endl;*/

      
      //std::cout << time_id << std::endl;
      //std::cout << dynamicsSequence().dynamicsState(time_id+1).centerOfMass().transpose() << "  " << std::endl;
      /* 
      std::cout << time_id << std::endl;
      std::cout << dynamicsSequence().dynamicsState(time_id).centerOfMass().transpose()  <<std::endl;
      std::cout << dynamicsSequence().dynamicsState(time_id).centerOfMass().transpose() - dynamicsSequence().dynamicsState(time_id-1).centerOfMass().transpose()  - dynamicsSequence().dynamicsState(time_id-1).linearMomentum().transpose()/95.941282 *0.010 <<std::endl;
      std::cout << dynamicsSequence().dynamicsState(time_id-1).centerOfMass().transpose()  <<std::endl;
      
      std::cout << dynamicsSequence().dynamicsState(time_id-1).linearMomentum().transpose()/95.941282 << "   " << dynamicsSequence().dynamicsState(time_id).time() <<" " << this->getSetting().get(PlannerDoubleParam_RobotMass)<< std::endl;
      std::cout << dynamicsSequence().dynamicsState(time_id).linearMomentumRate().transpose()/95.941282 << std::endl;
      /*double x = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()(0)/95.941282 -  12.3526*(dynamicsSequence().dynamicsState(time_id-1).centerOfMass()(0)-dynamicsSequence().dynamicsState(time_id-1).ZMP()(0)) + dynamicsSequence().dynamicsState(time_id).angularMomentumRate()(1)/76.19282014079963;
      double y = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()(1)/95.941282 -  12.3526*(dynamicsSequence().dynamicsState(time_id-1).centerOfMass()(1)-dynamicsSequence().dynamicsState(time_id-1).ZMP()(1)) - dynamicsSequence().dynamicsState(time_id).angularMomentumRate()(0)/76.19282014079963;
      std::cout <<"x " << x <<"y " << y << std::endl;
      x = dynamicsSequence().dynamicsState(time_id-1).linearMomentum()(0)/95.941282 * 0.010 + dynamicsSequence().dynamicsState(time_id-1).centerOfMass()(0) - dynamicsSequence().dynamicsState(time_id).centerOfMass()(0);

      y = dynamicsSequence().dynamicsState(time_id-1).linearMomentum()(1)/95.941282 * 0.010 + dynamicsSequence().dynamicsState(time_id-1).centerOfMass()(1) - dynamicsSequence().dynamicsState(time_id).centerOfMass()(1);
      std::cout <<"x_ " << x <<"y_ " << y << std::endl;
     */
    }
  }

  void DynamicsOptimizer::saveToFile(const KinematicsSequence& kin_sequence)
  {
    std::cout << "SAVE" << std::endl;
    if (this->getSetting().get(PlannerBoolParam_StoreData)) {
      try
      {
        YAML::Node cfg_pars = YAML::LoadFile(this->getSetting().get(PlannerStringParam_ConfigFile).c_str());
        YAML::Node qcqp_cfg;
        qcqp_cfg["dynopt_params"]["time_step"] = this->getSetting().get(PlannerDoubleParam_TimeStep);
        qcqp_cfg["dynopt_params"]["end_com"] = com_pos_goal_;
        qcqp_cfg["dynopt_params"]["robot_mass"] = this->getSetting().get(PlannerDoubleParam_RobotMass);
        qcqp_cfg["dynopt_params"]["n_act_eefs"] = this->getSetting().get(PlannerIntParam_NumActiveEndeffectors);
        qcqp_cfg["dynopt_params"]["ini_com"] = ini_state_.centerOfMass();
        qcqp_cfg["dynopt_params"]["time_horizon"] = this->getSetting().get(PlannerDoubleParam_TimeHorizon);
        qcqp_cfg["contact_plan"] = cfg_pars["contact_plan"];

        com_.getGuessValue(mat_guess_);   qcqp_cfg["dynopt_params"]["com_motion"] = mat_guess_;
        lmom_.getGuessValue(mat_guess_);  qcqp_cfg["dynopt_params"]["lin_mom"] = mat_guess_;
        amom_.getGuessValue(mat_guess_);  qcqp_cfg["dynopt_params"]["ang_mom"] = mat_guess_;

        // building momentum references
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { mat_guess_.col(time_id) = kin_sequence.kinematicsState(time_id).centerOfMass();  } qcqp_cfg["dynopt_params"]["com_motion_ref"] = mat_guess_;
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { mat_guess_.col(time_id) = kin_sequence.kinematicsState(time_id).linearMomentum(); } qcqp_cfg["dynopt_params"]["lin_mom_ref"] = mat_guess_;
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { mat_guess_.col(time_id) = kin_sequence.kinematicsState(time_id).angularMomentum(); } qcqp_cfg["dynopt_params"]["ang_mom_ref"] = mat_guess_;

        // building sequences of joint velocities and accelerations
        mat_guess_.resize(this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs), this->getSetting().get(PlannerIntParam_NumTimesteps));    mat_guess_.setZero();
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { mat_guess_.col(time_id) = kin_sequence.kinematicsState(time_id).robotVelocity().generalizedJointVelocities();  } qcqp_cfg["dynopt_params"]["jnt_vel"] = mat_guess_;
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { mat_guess_.col(time_id) = kin_sequence.kinematicsState(time_id).robotAcceleration().generalizedJointAccelerations();  } qcqp_cfg["dynopt_params"]["jnt_acc"] = mat_guess_;

        // saving vector of time-steps
        mat_guess_.resize(1, this->getSetting().get(PlannerIntParam_NumTimesteps)); mat_guess_.setZero();
        mat_guess_(0,0) = dynamicsSequence().dynamicsState(0).time();
        for (int time_id=1; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
          mat_guess_(0,time_id) = mat_guess_(0,time_id-1) + dynamicsSequence().dynamicsState(time_id).time();
        qcqp_cfg["dynopt_params"]["time_vec"] = mat_guess_;

        // saving vector of forces, torques and cops
        ZMP_.getGuessValue(mat_guess_);
        
        qcqp_cfg["dynopt_params"]["eef_cop_"] = mat_guess_;
        std::ofstream file_out(this->getSetting().get(PlannerStringParam_SaveDynamicsFile)); 
        file_out << qcqp_cfg;
      }
      catch (YAML::ParserException &e) { std::cout << e.what() << "\n"; }
    }
    std::cout << "SAVE FINISH" << std::endl;
  }

  const ContactType& DynamicsOptimizer::contactType(int time_id, int eff_id) const
  {
    int contact_id = dynamicsSequence().dynamicsState(time_id).endeffectorContactId(eff_id);
    return contact_plan_->contactSequence().endeffectorContacts(eff_id)[contact_id].contactType();
  }

  Eigen::Matrix3d DynamicsOptimizer::contactRotation(int time_id, int eff_id) const
  {
    int contact_id = dynamicsSequence().dynamicsState(time_id).endeffectorContactId(eff_id);
    return contact_plan_->contactSequence().endeffectorContacts(eff_id)[contact_id].contactOrientation().toRotationMatrix();
  }

  double DynamicsOptimizer::contactLocation(int time_id, int eff_id, int axis_id) const
  {
    int contact_id = dynamicsSequence().dynamicsState(time_id).endeffectorContactId(eff_id);
    return contact_plan_->contactSequence().endeffectorContacts(eff_id)[contact_id].contactPosition()[axis_id];
  }

}

