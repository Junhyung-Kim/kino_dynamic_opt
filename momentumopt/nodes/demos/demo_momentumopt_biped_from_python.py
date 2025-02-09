initial_robot_configuration:

  ###############################
  # Initial robot configuration #
  ###############################

  com: [8.61938268e-02, 5.18219890e-06, 7.94160099e-01]
  zmp: [8.61938268e-02, 5.18219890e-06]
  amom: [0.0, 0.0, 0.0]
  lmom: [0.0, 0.0, 0.0]
  # weight distribution, normalized by gravity * robot_weight
  eef_ctrl:
    eef_frc_rf: [0.0, 0.0, 0.5] 
    eef_frc_lf: [0.0, 0.0, 0.5]
    eef_frc_rh: [0.0, 0.0, 0.0]
    eef_frc_lh: [0.0, 0.0, 0.0]

  eef_pose:
    eef_rf: [1.0, 9.47987102e-02, -0.1025, 0.0, 1.0, 0.0, 0.0, 0.0]
    eef_lf: [1.0, 9.47987102e-02, 0.1025, 0.0, 1.0, 0.0, 0.0, 0.0]
    eef_rh: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    eef_lh: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
  

terrain_description:

  #######################
  # Terrain description #
  #######################

  regions: [[[1.00, 1.00, 0.0], [-1.00, 1.00, 0.0], [-1.00, -1.00, 0.0], [1.00, -1.00,
        0.0]]]


contact_plan:

  ################
  # Contact plan #
  ################

  # contact specification
  #  [0.0,                  # start time of contact
  #   1.9,                  # end time of contact
  #   0.10, -0.20, -0.32,   # position (x, y, z)
  #   1.0, 0.0, 0.0, 0.0,   # orientation in quaternions (w, q_x, q_y, q_z)
  #   1.0,                  # selecting contact type
  #                         # 1.0 : flat contact, forces subject to friction cone constrains of flat surface
  #                         # 2.0 : full contact, forces free in any direction (e.g. hand grasp)
  #  -1.0]                  # select the number of terrains in which the contact should lie
                            # -1.0 : no contact planning
  #zmp 수정 필요
  effcnt_rf:
  - [0.0, 1.4, 9.47987102e-02, -0.1025, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0]
  - [2.0, 3.40, 0.2947, -0.1025, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0]
  - [4.0, 4.401, 0.3947, -0.1025, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0]

  effcnt_lf:
  - [0.0, 0.4, 9.47987102e-02, 0.1025, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0]
  - [1.0, 2.40, 0.1947, 0.1025, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0]
  - [3.0, 4.401, 0.3947, 0.1025, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0]
  
  effcnt_rh: #redundant for bolt because it does not have hands
  - [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  effcnt_lh: #redundant for bolt because it does not have hands
  - [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  effvia_rf: []
  effvia_lf: []
  effvia_rh: []
  effvia_lh: []

planner_variables:

  #########################
  # Kinematics parameters #
  #########################

  load_kinematics: true

  display_motion: False
  num_dofs: 12

  # Number of dynamic and kinematic optimization iterations
  kd_iterations: 1 #25
  num_subsamples: 3
  active_dofs: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
  min_joint_limits: [-2, -2, -3, -2, -2, -2, -2, -2, -3, -2, -2, -2]
  max_joint_limits: [2, 2, 3, 2, 2, 2, 2, 2, 3, 2, 2, 2]

  default_joint_positions: [0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]

  #########################
  # IK formulation #
  #########################

  use_second_order_inv_kin: True # True: Second order IK,  False: First order IK,

  #########################
  # First Order IK parameters #
  #########################

  reg_joint_position: 0.0
  num_joint_viapoints: 0
  num_base_viapoints: 0
  # joint via point definition
  #  [time,                      # activation time of the via point
  # Front_left_hip, Front_left_knee,
  # Front_right_hip, Front_right_knee,build_and_optimize_motion
  # Back_left_hip, Back_left_knee,
  # Back_right_hip, Back_right_knee ]        # desired joint position
  joint_viapoints:
    via0: [0, 0, 0.80783, 0, 0, 0, 1, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]
  # base via point definition
  #  [time,                      # activation time of the via point
  # roll, pitch, yaw]        # desired base Euler angles in radian
  base_viapoints:
    via0: [0.0, 0., 0., 0.]

  #########################
  # Second Order IK parameters #
  #########################

  swing_traj_via_z_second: 0.03
  w_lin_mom_tracking_second: 1e1
  w_ang_mom_tracking_second: 1e1
  w_endeff_contact_second: 1e0
  w_endeff_tracking_second: 1e0
  w_endori_contact_second: 1e0
  w_endori_tracking_second: 1e0
  w_joint_regularization_second: 0.0
  p_endeff_tracking_second: 1e1
  d_endeff_tracking_second: 1e0
  p_com_tracking_second: 1e0
  p_orient_tracking_second: 1e1
  d_orient_tracking_second: 1e0
  p_orientf_tracking_second: 1e1
  d_orientf_tracking_second: 1e0
  p_joint_regularization_second: 0.0
  d_joint_regularization_second: 0.00
  p_mom_tracking_second: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  num_joint_viapoints_second: 0
  num_base_viapoints_second: 0

  # joint via point definition
  #  [time,                      # activation time of the via point
  # Front_left_hip, Front_left_knee,
  # Front_right_hip, Front_right_knee,
  # Back_left_hip, Back_left_knee,
  # Back_right_hip, Back_right_knee ]        # desired joint position
  joint_viapoints_second:
    via0: [0, 0, 0, 0.80783, 0, 0, 0, 1, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]

  # base via point definition
  #  [time,                      # activation time of the via point
  # roll, pitch, yaw]        # desired base Euler angles in radian
  base_viapoints_second:
    via0: [0.0, 0., 0., 0.]

  #########################
  # Kinematics parameters #
  #########################

  # load_kinopt: False

  #######################
  # Dynamics parameters #
  #######################

  heuristic: TrustRegion  # Types: TrustRegion, SoftConstraint, TimeOptimization #
  n_act_eefs: 2
  # time step discretization
  time_step: 0.02 #0.005
  time_horizon: 4.400
  external_force: [0.00, 0.00, 0.00]
  com_displacement: [0.10, 0.0, 0.00]
  # Will only read via points if this number is great than 0
  num_com_viapoints: 0
  # via point definition
  # [2.50                 # activation time of the via point
  #  0.500, 0.45,  0.00]  # position (x, y, z275
  com_viapoints:
    via0: [2.20, 0.2020414, 0.00, 0.82153224]

  #####################
  # Timing parameters #
  #####################

  max_time_iterations: 1
  max_time_residual_tolerance: 1e-3
  min_time_residual_improvement: 1e-5

  ############################
  # Configuration parameters #
  ############################

  gravity: 9.81
  robot_mass: 95.941282
  friction_coeff: 0.5
  floor_height: 0.0
  # minimum allowed distance from the robot base to the floor
  min_rel_height: 0.80783
  friction_cone: LinearCone  # Types: LinearCone, SocCone #
  time_range: [0.02, 0.02]
  is_time_horizon_fixed: true
  torque_range: [-10000.0, 10000.0]
  eff_offset_rf: [9.47987102e-02, -1.02500000e-01, 6.43027992e-10]
  eff_offset_lf: [9.47987102e-02, 1.02500000e-01, 6.43027992e-10]
  eff_offset_rh: []
  eff_offset_lh: []
  cop_range_rf: [-0.10, 0.10, -0.04, 0.04]
  cop_range_lf: [-0.10, 0.10, -0.04, 0.04]
  cop_range_rh: []
  cop_range_lh: []
  max_eef_lengths: [1.3, 1.3, 1.3, 1.3]
  has_torque_limits: false
  min_torque_limits: [-5.675, -5.675, -5.675, -5.675, -5.675, -5.675]
  max_torque_limits: [5.675, 5.675, 5.675, 5.675, 5.675, 5.675]

  ####################
  # Dynamics weights #
  ####################

  w_trq_arm: 0.000
  w_trq_leg: 0.000
  w_time_penalty: 0.000
  w_time: 1000.0
  w_com: [100000000., 100000000.0, 550000] #cost weight of final com position
  w_amom: [1.0, 1.0, 0.0]
  w_lmom: [0.01, 0.1, 0.0]
  w_amomd: [1.0, 1.0, 0.00]
  w_lmomd: [0.01, 0.01, 0.0]
  w_zmp: [1, 5]
  w_zmpd: [500,500]
  w_zmpc: [600, 600]
  
  w_amom_final: [10.00, 10.0, 10.00] # cost weight of final linear momentum
  w_lmom_final: [10.00, 10.0, 10.00] # cost weight of final angular momentum
  w_com_via: [0.000, 0.0, 0.000]
  w_frc_arm: [0.0, 0.0, 0.0]
  w_frc_leg: [0.0, 0.0, 0.0]
  w_dfrc_arm: [0.050, 0.05, 0.020]
  w_dfrc_leg: [0.050, 0.05, 0.020]
  w_com_track: [800.00, 800.00, 0.000] #cost weight of tracking of com trajectory from kinematic planner
  w_lmom_track: [30.00, 30.00, 0.00] #cost weight of tracking of linear momentum trajectory from kinematic planner
  w_amom_track: [280.000, 280.00, 0.000] #cost weight of tracking of angular momentum trajectory from kinematic planner

  ######################
  # Kinematics weights #
  ######################

  max_trajectory_iters: 20
  max_convergence_iters: 300
  convergence_tolerance: 1e-5
  integration_step: 1e-5
  slacks_penalty: 1e-5
  lambda_regularization: 1e-5

  w_kin_com: [10.00, 10.0, 10.00]
  w_kin_lmom: [1.e-3, 0.0, 1.e-4]
  w_kin_amom: [4.e-3, 0.00568, 0.000]
  w_kin_lmomd: [1.e-4, 0.000142, 1.e-4]
  w_kin_amomd: [1.e-3, 0.00142, 0.000]
  w_kin_eff_pos: [3.00, 3.2, 3.00]
  w_kin_eff_pos_nonact: [5.00, 1.42, 5.00]
  w_kin_base_ori: [1.e+0, 1.42e-08, 1.e-8]
  w_kin_default_joints: [ 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]
  w_kin_joint_vel: [1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-8, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
    1e-6, 1e-6, 1e-6]
  w_kin_joint_acc: [1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-8, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
    1e-6, 1e-6, 1e-6]
                          # base                                 # joints

  #######################
  # Storage information #
  #######################

  store_data: true

  ##################
  # Solver setting #
  ##################

  use_default_solver_setting: false


solverlqr_dynamics:
  ############################
  # Lqr algorithm parameters #
  ############################
  verbosity: 0
  decimal_digits: 4
  lqr_max_iters: 50
  linesearch_coeff: 1.1
  linesearch_num_coeffs: 10
  cost_change_tolerance: 1e-9
  divergence_limit_check: 1e6
  bpass_regularization_type: 2
  bpass_min_regularization: 1e-8
  bpass_max_regularization: 1e10
  control_gradient_tolerance: 1e-8
  bpass_initial_regularization: 0.1
  min_expected_cost_improvement: 0.2
  bpass_mult_regularization_incr: 1.2
  bpass_initial_mult_regularization_incr: 1.0

  ##########################
  # Lqr problem parameters #
  ##########################
  problem_name: DynamicMomentumOptimization
  time_step: 0.02
  time_horizon: 4.400
  state_dimension: 9
  control_dimension: 12
  initial_state: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  has_control_limits: false

  #######################
  # Storage information #
  #######################

  store_data: true

  ###################
  # User parameters #
  ###################

  user_parameters:
    control_cost: [0.001, 0.00142, 0.001]
    com_tracking: [1.0, 1.42, 1.0]
    lmom_tracking: [0.1, 0.142, 0.1]
    amom_tracking: [0.1, 0.142, 0.1]
    com_final_tracking: [10.0, 10.0, 10.0]
    lmom_final_tracking: [0.1, 0., 0.1]
    amom_final_tracking: [0.1, 0., 0.1]


solver_variables:

  ###########################
  # Branch and Bound solver #
  ###########################
  BnB_verbose: false
  BnB_max_iterations: 1000
  BnB_integer_tolerance: 1e-4
  BnB_absolute_suboptimality_gap: 1e-3
  BnB_relative_suboptimality_gap: 1e-6

  ##########################
  # Convergence tolerances #
  ##########################

  feasibility_tolerance: 1e-7
  max_residual_tolerance: 1e-7
  absolute_suboptimality_gap: 1e-7
  relative_suboptimality_gap: 1e-6
  max_indeterminate_tolerance: 1e-9
  feasibility_tolerance_inaccurate: 1e-4
  absolute_suboptimality_gap_inaccurate: 5e-5
  relative_suboptimality_gap_inaccurate: 5e-5

  ############################
  # Equilibration parameters #
  ############################

  equil_iterations: 3
  scaling_factor: 1.0
  equil_upper_bound: 1.0e3
  equil_lower_bound: 1.0e-3

  ############################
  # Linear System parameters #
  ############################

  dyn_reg_thresh: 1e-13
  lin_sys_accuracy: 1e-14
  err_reduction_factor: 6.0
  num_iter_ref_lin_solve: 9
  static_regularization: 7e-8
  dynamic_regularization: 2e-7

  cg_step_rate: 2.0
  cg_full_precision: 1e-9
  cg_residual_bound: 1e-18
  cg_reduced_precision: 1e-1

  ########################
  # Algorithm parameters #
  ########################

  safeguard: 500.0
  min_step_length: 1e-6
  max_step_length: 0.999
  min_centering_step: 1e-4
  max_centering_step: 1.00
  step_length_scaling: 0.99

  over_relaxation: 1.5
  optinfo_interval: 100
  converged_interval: 20
  consensus_penalty: 1e-3

  ####################
  # Model parameters #
  ####################

  verbose: false
  warm_start: True

  max_iters: 100
  ipsolver_max_iters: 100
  ipsolver_warm_iters: 100
  conicsolver_max_iters: 300
  conicsolver_warm_iters: 300

  num_itrefs_trustregion: 2
  trust_region_threshold: 0.15
  soft_constraint_weight_full: 1.0e4
  soft_constraint_weight_reduced: 1.0e4