#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller_hw.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"
#include <fstream>
#include <tf/tf.h>

Vars vars;
Params params;
Workspace work;
Settings settings;

namespace dyros_jet_controller
{ 
  //HW
  ofstream e_zmp_graph("/home/econom2/data/hw/e_zmp_graph.txt");
  ofstream e_com_graph("/home/econom2/data/hw/e_com_graph.txt");
  ofstream e_foot_graph("/home/econom2/data/hw/e_foot_graph.txt");
  ofstream e_cp_p_graph("/home/econom2/data/hw/e_cp_p_graph.txt");
  ofstream e_cp_t_graph("/home/econom2/data/hw/e_cp_t_graph.txt");

  ofstream e_mpc_graphxy("/home/econom2/data/hw/e_mpc_graphxy.txt");
  ofstream e_mpc_graphz("/home/econom2/data/hw/e_mpc_graphz.txt");
  ofstream e_tmp_graph("/home/econom2/data/hw/e_tmp_graph1.txt");
  ofstream e_tmp_graph2("/home/econom2/data/hw/e_tmp_graph2.txt");
  ofstream e_tmp_graph3("/home/econom2/data/hw/e_tmp_graph3.txt");

void WalkingController::compute()
{   
  if(walking_enable_ == true)
  {
    updateInitialState();
    getRobotState();  
    floatToSupportFootstep(); 
    
    if(ready_for_thread_flag_ == false)
    { ready_for_thread_flag_ = true; }
    
    if(ready_for_compute_flag_ == true)
    {
      if(current_step_num_< total_step_num_)
      {               
        getZmpTrajectory();
        getComTrajectory();
        circling_motion();
        getCPTrajectory();
        getFootTrajectory();
        getPelvTrajectory();
        supportToFloatPattern();
        computeIK_e(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des);
        dataPlot();
  
        for(int i=0; i<12; i++)
        { desired_q_(i) = q_des(i); } 

        hip_compensator();

        //DoB
        Eigen::Vector12d d_q;
        for(int i = 0; i < 12; i++)
        { d_q(i) = desired_q_(i); }
        //Compliant_control(d_q);

        updateNextStepTime();
      }
      else
      {
        desired_q_ = current_q_;
      }
    }
    else
    {
      desired_q_ = current_q_;
    }
  }
}

void WalkingController::dataPlot()
{
  double com_tempy_b = com_tempy;
  com_tempy = com_support_current_(1) - 0.5*foot_step_support_frame_(current_step_num_,1);
  double com_tempy_dot = (com_tempy - com_tempy_b)*hz_;
  e_com_graph << com_desired_(0) << "," << com_desired_(1) << "," << com_desired_(2) << "," << com_support_current_(0) << "," << com_support_current_(1) << "," << com_support_current_(2) << endl;
  e_zmp_graph << zmp_desired_(0) << "," << zmp_desired_(1) << "," << zmp_measured_LPF(0) << "," << zmp_measured_LPF(1) << endl;
  e_foot_graph << lfoot_trajectory_support_.translation()(2) << "," << rfoot_trajectory_support_.translation()(2) << "," << lfoot_support_current_.translation()(2) << "," << rfoot_support_current_.translation()(2) << endl;
}

void WalkingController::setTarget(int walk_mode, bool hip_compensation, bool lqr, int ik_mode, bool heel_toe,
                                  bool is_right_foot_swing, double x, double y, double z, double height, double theta,
                                  double step_length_x, double step_length_y, bool walking_pattern)
{
  target_x_ = x;
  target_y_ = y;
  target_z_ = z;
  com_height_ = height;
  target_theta_ = theta;
  step_length_x_ = step_length_x;
  step_length_y_ = step_length_y;
  ik_mode_ = ik_mode;
  walk_mode_ = walk_mode;
  hip_compensator_mode_ = hip_compensation; //uint32 compensator_mode[0] : HIP_COMPENSTOR    uint32 compensator_mode[1] : EXTERNAL_ENCODER   
  is_right_foot_swing_ = is_right_foot_swing;  
  walkingPatternDCM_ = walking_pattern; 

  parameterSetting();
}

void WalkingController::setEnable(bool enable)
{
  walking_enable_ = enable;
  desired_q_ = current_q_;
}

void WalkingController::updateControlMask(unsigned int *mask)
{
  if(walking_enable_)
  {
    for (int i=0; i<total_dof_-18; i++) //control only leg
    {
      mask[i] = (mask[i] | PRIORITY);
    }
    mask[total_dof_-1] = (mask[total_dof_-1] & ~PRIORITY); //Gripper
    mask[total_dof_-2] = (mask[total_dof_-2] & ~PRIORITY); //Gripper
    mask[total_dof_-3] = (mask[total_dof_-3] & ~PRIORITY); //Head
    mask[total_dof_-4] = (mask[total_dof_-4] & ~PRIORITY); //Head
  }
  else
  {
    for (int i=0; i<total_dof_; i++)
    {
      mask[i] = (mask[i] & ~PRIORITY);
    }
  }
}

void WalkingController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {     
    if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
    {
      if(hip_compensator_mode_ == true)
      {
        if(walking_tick_ == 0)
        { desired_q(i) = desired_q_(i); }
        else
        { desired_q(i) = DOB_IK_output_b_(i); }
      }
      else
      { 
        desired_q(i) = desired_q_(i);
      }            
    }         
  } 
}

void WalkingController::parameterSetting()
{
  t_double1_ = 0.10*hz_; 
  t_double2_ = 0.10*hz_;
  t_rest_init_ = 0.05*hz_;
  t_rest_last_ = 0.05*hz_;
  t_total_= 1.2*hz_;
  t_temp_ = 3.0*hz_;
  t_last_ = t_total_ + t_temp_ ; //4.1*hz_;
  t_start_ = t_temp_ + 1 ;
 
  t_start_real_ = t_start_ + t_rest_init_;

  current_step_num_ = 0;
  walking_tick_ = 0;
  foot_height_ = 0.05; 

  foot_step_tmp_ = 6;

  gyro_frame_flag_ = false;
  com_control_mode_ = true;
  estimator_flag_ = false; 
}

void WalkingController::hip_compensator()
{
  double mass_total_ = 0, alpha;
  mass_total_ = 47;

  Eigen::Vector12d grav_ground_torque_;
  Eigen::Vector12d grav_ground_torque_pre_;
  Eigen::Vector12d grav_ground_torque_filtered_;

  Eigen::Vector6d lTau, rTau;
  lTau.setZero();
  rTau.setZero();

  //Eigen::Matrix<double, 6, 6> j_lleg_foot = model_.getLegLinkJacobian(5);
  //Eigen::Matrix<double, 6, 6> j_rleg_foot = model_.getLegLinkJacobian(11);

  Eigen::Matrix<double, 6, 6> j_lleg_foot = model_.getLegJacobian((DyrosJetModel::EndEffector) 0);
  Eigen::Matrix<double, 6, 6> j_rleg_foot = model_.getLegJacobian((DyrosJetModel::EndEffector) 1);

  Eigen::Matrix6d adjoint_lleg_foot;
  Eigen::Matrix6d adjoint_rleg_foot;
  Eigen::Matrix3d skew_lleg_foot;
  Eigen::Matrix3d skew_rleg_foot;
  Eigen::Vector3d foot_offset;
  Eigen::Vector6d gravity_force;
  Eigen::Vector3d supportfoot_offset_float_current;

  gravity_force.setZero();
  gravity_force(2) = -mass_total_*GRAVITY;
  foot_offset.setZero();
  foot_offset(2) = -0.095;
  supportfoot_offset_float_current.setZero();

  skew_lleg_foot = DyrosMath::skew(lfoot_float_current_.linear()*foot_offset);
  skew_rleg_foot = DyrosMath::skew(rfoot_float_current_.linear()*foot_offset);

  adjoint_lleg_foot.setIdentity();
  adjoint_rleg_foot.setIdentity();

  adjoint_lleg_foot.block<3, 3>(0, 3) = -skew_lleg_foot;
  adjoint_rleg_foot.block<3, 3>(0, 3) = -skew_rleg_foot;

  j_lleg_foot = adjoint_lleg_foot*j_lleg_foot;
  j_rleg_foot = adjoint_rleg_foot*j_rleg_foot;

  alpha = (com_float_current_(1) - rfoot_float_current_.translation()(1))/(lfoot_float_current_.translation()(1) - rfoot_float_current_.translation()(1));
  if(alpha > 1)
  {
    alpha = 1;
  }
  else if(alpha < 0)
  {
    alpha = 0;
  }

  if(foot_step_(current_step_num_,6)==1 && walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_) //left foot support
  {
    lTau = j_lleg_foot.transpose()*gravity_force;
    rTau = model_.getLegGravTorque(1);
  }
  else if(foot_step_(current_step_num_,6)==0 && walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_) //right foot support
  {
    rTau = j_rleg_foot.transpose()*gravity_force;
    lTau = model_.getLegGravTorque(0);
  }
  else
  {
    lTau = (j_lleg_foot.transpose()*gravity_force)*alpha;
    rTau = (j_rleg_foot.transpose()*gravity_force)*(1-alpha);
  }

  if(walking_tick_ == 0)
  {grav_ground_torque_pre_.setZero();}

  for (int i = 0; i < 6; i ++)
  {
    grav_ground_torque_(i) = lTau[i];
    grav_ground_torque_(i+6) = rTau[i];
  }

  desired_q_(1) = desired_q_(1) + 0.0002*grav_ground_torque_(1);
  desired_q_(2) = desired_q_(2) + 0.0004*grav_ground_torque_(2);
  desired_q_(3) = desired_q_(3) + 0.0004*grav_ground_torque_(3);

  desired_q_(7) = desired_q_(7) + 0.0002*grav_ground_torque_(7);
  desired_q_(8) = desired_q_(8) + 0.0004*grav_ground_torque_(8);
  desired_q_(9) = desired_q_(9) + 0.0004*grav_ground_torque_(9);
}

void WalkingController::circling_motion()
{
  cout << "hello" << endl;
}

void WalkingController::getRobotState()
{
  UpdateCentroidalMomentumMatrix();

  Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
  q_temp.setZero();
  qdot_temp;  
  q_temp.segment<28>(6) = current_q_.segment<28>(0);   
  qdot_temp.segment<28>(6)= current_qdot_.segment<28>(0);

  //if(walking_tick_ > 0) 
  //{ q_temp.segment<12>(6) = desired_q_not_compensated_.segment<12>(0);}

  model_.updateKinematics(q_temp, qdot_temp);
  com_float_current_ = model_.getCurrentCom();
  com_float_current_dot_= model_.getCurrentComDot();
  lfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)0); 
  rfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)1);
    
  if(foot_step_(current_step_num_, 6) == 0) 
  { supportfoot_float_current_ = rfoot_float_current_; }
  else if(foot_step_(current_step_num_, 6) == 1)
  { supportfoot_float_current_ = lfoot_float_current_; }

  if(current_step_num_ != 0 && walking_tick_ == t_start_)
  {
    if(foot_step_(current_step_num_, 6) == 0)
    {
      com_support_current_dot = (DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(lfoot_float_current_), com_float_current_) - com_support_current_)*hz_;
    }
    else if(foot_step_(current_step_num_, 6) == 1)
    {
      com_support_current_dot = (DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(rfoot_float_current_), com_float_current_) - com_support_current_)*hz_;
    }
  }

  pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_);
  pelv_float_current_.setIdentity();
  lfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,lfoot_float_current_);
  rfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,rfoot_float_current_);     
  
  com_support_current_b = com_support_current_;
  com_support_current_ =  DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);
  
  if(walking_tick_ != t_start_)
  {
    com_support_current_dot = (com_support_current_ - com_support_current_b)*hz_;
    com_support_current_ddot = (com_support_current_dot - com_support_current_dot_LPF_b_)*hz_;
  }

  if(walking_tick_ == 0)
  {
    com_support_current_dot.setZero();
    com_support_current_ddot.setZero();
    com_support_current_dot_LPF = com_support_current_dot;
    com_support_current_ddot_LPF = com_support_current_ddot;
  }
  com_support_current_dot_LPF_b_ = com_support_current_dot_LPF;
  com_support_current_dot_LPF = (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*com_support_current_dot + 1/(1+2*M_PI*8.0*del_t)*com_support_current_dot_LPF;
  com_support_current_ddot_LPF = (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*com_support_current_ddot + 1/(1+2*M_PI*8.0*del_t)*com_support_current_ddot_LPF;

  r_ft_ = model_.getRightFootForce();
  l_ft_ = model_.getLeftFootForce();
  
  current_motor_q_leg_ = current_q_.segment<12>(0);
}

void WalkingController::calculateFootStepTotal()
{
  double initial_rot = 0.0;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  initial_rot = atan2(target_y_, target_x_);

  if(initial_rot > 0.0)
    initial_drot = 10*DEG2RAD;
  else
    initial_drot = -10*DEG2RAD;

  unsigned int initial_total_step_number = initial_rot/initial_drot;
  double initial_residual_angle = initial_rot - initial_total_step_number*initial_drot;

  final_rot = target_theta_ - initial_rot;
  if(final_rot > 0.0)
    final_drot = 10*DEG2RAD;
  else
    final_drot = -10*DEG2RAD;

  unsigned int final_total_step_number = final_rot/final_drot;
  double final_residual_angle = final_rot - final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_ + target_y_*target_y_);
  double dlength = step_length_x_;
  unsigned int middle_total_step_number = length_to_target/dlength;
  double middle_residual_length = length_to_target - middle_total_step_number*dlength;

  total_length_input = length_to_target;

  if(length_to_target == 0)
  {
    middle_total_step_number = foot_step_tmp_;
    dlength = 0;
  }


  unsigned int number_of_foot_step;

  int del_size;

  del_size = 1;
  number_of_foot_step = initial_total_step_number*del_size + middle_total_step_number*del_size + final_total_step_number*del_size;

  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    if(initial_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(initial_residual_angle)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(middle_residual_length)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    if(abs(final_residual_angle) >= 0.0001)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
      number_of_foot_step = number_of_foot_step + del_size;
  }


  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();

  int index = 0;
  int temp, temp2, temp3, is_right;

  if(is_right_foot_swing_ == true)
    is_right = 1;
  else
    is_right = -1;


  temp = -is_right;
  temp2 = -is_right;
  temp3 = -is_right;


  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    for (int i =0 ; i < initial_total_step_number; i++)
    {
      temp *= -1;
      foot_step_(index,0) = temp*0.127794*sin((i+1)*initial_drot);
      foot_step_(index,1) = -temp*0.127794*cos((i+1)*initial_drot);
      foot_step_(index,5) = (i+1)*initial_drot;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index++;
    }

    if(temp == is_right)
    {
      if(abs(initial_residual_angle) >= 0.0001)
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

      }
      else
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;
      }
    }
    else if(temp == -is_right)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;

      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
  {
    for (int i = 0 ; i < middle_total_step_number; i++)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(i+1)) + temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(i+1)) - temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index ++;
    }

    if(temp2 == is_right)
    {
      if(abs(middle_residual_length) >= 0.0001)
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;

        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
      else
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
    }
    else if(temp2 == -is_right)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;

      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;
    }
  }

  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    for(int i = 0 ; i < final_total_step_number; i++)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.127794*sin((i+1)*final_drot + initial_rot);
      foot_step_(index,1) = final_position_y - temp3*0.127794*cos((i+1)*final_drot + initial_rot);
      foot_step_(index,5) = (i+1)*final_drot + initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }

    if(abs(final_residual_angle) >= 0.0001)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;

      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
    else
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
  }
}

void WalkingController::floatToSupportFootstep()
{
  Eigen::Isometry3d reference;

  if(current_step_num_ == 0)
  {
    if(foot_step_(0,6) == 0)
    {
      reference.translation() = rfoot_float_init_.translation();
      reference.translation()(2) = 0.0;
      reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
      reference.translation()(0) = 0.0;
    }
    else
    {
      reference.translation() = lfoot_float_init_.translation();
      reference.translation()(2) = 0.0;
      reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
      reference.translation()(0) = 0.0;
    }
  }
  else
  {
    reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1,5));
    for(int i=0 ;i<3; i++)
      reference.translation()(i) = foot_step_(current_step_num_-1,i);
  }

  Eigen::Vector3d temp_local_position;
  Eigen::Vector3d temp_global_position;

  for(int i = 0; i < total_step_num_; i++)
  {
    for(int j = 0; j < 3; j ++)
      temp_global_position(j) = foot_step_(i,j);

    temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

    for(int j=0; j<3; j++)
      foot_step_support_frame_(i,j) = temp_local_position(j);
    
    //econom2
    foot_step_support_frame_(i,1) = temp_local_position(1) + foot_width_tmp_;

    foot_step_support_frame_(i,3) = foot_step_(i,3);
    foot_step_support_frame_(i,4) = foot_step_(i,4);
    foot_step_support_frame_(i,5) = foot_step_(i,5) - foot_step_(current_step_num_-1,5);

    if(current_step_num_ == 0)
    { foot_step_support_frame_(i,5) = foot_step_(i,5) - supportfoot_float_init_(5); }
  }

  for(int j=0;j<3;j++)
    temp_global_position(j) = supportfoot_float_init_(j);

  temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

  for(int j=0;j<3;j++)
    supportfoot_support_init_(j) = temp_local_position(j);

  supportfoot_support_init_(3) = supportfoot_float_init_(3);
  supportfoot_support_init_(4) = supportfoot_float_init_(4);

  if(current_step_num_ == 0)
    supportfoot_support_init_(5) = 0;
  else
    supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_-1,5);
}

void WalkingController::updateInitialState()
{
  if(walking_tick_ == 0)
  {
    calculateFootStepTotal(); 
 
    com_float_init_ = model_.getCurrentCom();
    pelv_float_init_.setIdentity();
    lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));    
    
    Eigen::Isometry3d ref_frame;

    if(foot_step_(0, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }    
    else if(foot_step_(0, 6) == 1)
    { ref_frame = lfoot_float_init_; }
    
    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
    
    com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();
    
    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

    supportfoot_float_init_.setZero();
    swingfoot_float_init_.setZero();

    if(foot_step_(0,6) == 1) //left suppport foot
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }
    else
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }
    pelv_support_start_ = pelv_support_init_;
    total_step_num_ = foot_step_.col(1).size();
    xi_ = com_support_init_(0); // preview parameter
    yi_ = com_support_init_(1);
    zc_ = com_support_init_(2);     
    
  }
  else if(current_step_num_ != 0 && walking_tick_ == t_start_) // step change 
  {  
    Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
    q_temp.setZero();
    qdot_temp;
    q_temp.segment<28>(6) = current_q_.segment<28>(0);
    qdot_temp.segment<28>(6)= current_qdot_.segment<28>(0);  
    //q_temp.segment<12>(6) = desired_q_not_compensated_.segment<12>(0);
     
    model_.updateKinematics(q_temp, qdot_temp);

    lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
    com_float_init_ = model_.getCurrentCom();
    pelv_float_init_.setIdentity();  

    Eigen::Isometry3d ref_frame;

    if(foot_step_(current_step_num_, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }
    else if(foot_step_(current_step_num_, 6) == 1)
    { ref_frame = lfoot_float_init_; }

    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
    com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation(); 
    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear()); 

    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);    
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear()); 
  }
} 

void WalkingController::getZmpTrajectory()
{
  unsigned int planning_step_num_ = 3;
  unsigned int planning_zmp_size_tick = 0;

  planning_zmp_size_tick = t_total_*(min((int)(total_step_num_ - current_step_num_), (int)planning_step_num_)) // time for rest step
                         + (1-(bool)current_step_num_)*(t_temp_ + 1)                                           // time for init
                         + (current_step_num_>=total_step_num_ - planning_step_num_)*20*hz_;                   // time for last

  zmpGenerator(planning_zmp_size_tick, planning_step_num_);
  zmpCalc();

  zmp_desired_ = ref_zmp_.block<1,2>(walking_tick_ - (bool)current_step_num_*t_start_,0);
}

void WalkingController::zmpCalc()
{
  Eigen::Vector2d left_zmp, right_zmp;
  left_zmp(0) = l_ft_(4)/l_ft_(2) + lfoot_trajectory_support_.translation()(0);
  left_zmp(1) = l_ft_(3)/l_ft_(2) + lfoot_trajectory_support_.translation()(1);

  right_zmp(0) = r_ft_(4)/r_ft_(2) + rfoot_trajectory_support_.translation()(0);
  right_zmp(1) = r_ft_(3)/r_ft_(2) + rfoot_trajectory_support_.translation()(1);

  zmp_measured_(0) = (left_zmp(0)*l_ft_(2) + right_zmp(0)*r_ft_(2))/(l_ft_(2) + r_ft_(2));
  zmp_measured_(1) = (left_zmp(1)*l_ft_(2) + right_zmp(1)*r_ft_(2))/(l_ft_(2) + r_ft_(2));

  zmp_measured_LPF = (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*zmp_measured_ + 1/(1+2*M_PI*8.0*del_t)*zmp_measured_LPF;
}

void WalkingController::kajita_alpha_zmp()
{
  double t_mass = 43.8;
  double alpha = 0;
  if(foot_step_(current_step_num_,6) == 1)//left foot support
  { alpha = (foot_step_support_frame_(current_step_num_,1) - zmp_desired_(1))/(foot_step_support_frame_(current_step_num_,1)); }
  else
  { alpha = 1 - (foot_step_support_frame_(current_step_num_,1) - zmp_desired_(1))/(foot_step_support_frame_(current_step_num_,1)); }
  alpha = (1 - (foot_step_support_frame_(current_step_num_,1) - zmp_desired_(1))/(foot_step_support_frame_(current_step_num_,1))) + foot_step_(current_step_num_,6)*(2*(foot_step_support_frame_(current_step_num_,1) - zmp_desired_(1))/(foot_step_support_frame_(current_step_num_,1)) - 1);
  if(alpha > 1 || alpha < 0) { alpha = 0.5; }
  if(walking_tick_ == 1) { alpha = 0.5; }

  kajita_alpha_zmp_calc.resize(6); // frd, trxd, tryd, fld, tlxd, tlyd
  kajita_alpha_zmp_calc.setZero();
  kajita_alpha_zmp_calc(0) = - alpha*t_mass*(GRAVITY + MPC_z_(2));
  kajita_alpha_zmp_calc(1) = ((1-foot_step_(current_step_num_,6))*foot_step_support_frame_(current_step_num_,1) - zmp_desired_(1)) * kajita_alpha_zmp_calc(0);
  kajita_alpha_zmp_calc(3) = - (1 - alpha)*t_mass*(GRAVITY + MPC_z_(2));
  kajita_alpha_zmp_calc(4) = (foot_step_(current_step_num_,6)*foot_step_support_frame_(current_step_num_,1) - zmp_desired_(1)) * kajita_alpha_zmp_calc(3);

  double calc_rfoot = 0, calc_lfoot = 0;
  calc_rfoot = zmp_desired_(0);
  calc_lfoot = zmp_desired_(0);
  cout << t_start_ << endl;
  if(walking_tick_ - t_start_ >= 0 && walking_tick_ - t_start_ < t_rest_init_ + t_double1_)
  {
    cout << "ddd" << endl;
    calc_rfoot = zmp_desired_(0) + foot_step_(current_step_num_,6)*foot_step_support_frame_(current_step_num_,0) * (bool)(current_step_num_);
    calc_lfoot = zmp_desired_(0) - foot_step_(current_step_num_,6)*foot_step_support_frame_(current_step_num_,0) * (bool)(current_step_num_);
  }
  else if(walking_tick_ - t_start_ >= t_rest_init_ + t_double1_ && walking_tick_ - t_start_ < t_total_ - t_rest_last_ - t_double2_)
  {
    cout << "sss" << endl;
    calc_rfoot = zmp_desired_(0);
    calc_lfoot = zmp_desired_(0);
  }
  else if(walking_tick_ - t_start_ >= t_total_ - t_rest_last_ - t_double2_ && walking_tick_ - t_start_ < t_total_)
  {
    cout << "ddd" << endl;
    calc_rfoot = zmp_desired_(0) - foot_step_(current_step_num_,6)*foot_step_support_frame_(current_step_num_,0);
    calc_lfoot = zmp_desired_(0) + foot_step_(current_step_num_,6)*foot_step_support_frame_(current_step_num_,0);
  }
  
  e_tmp_graph2 << alpha << "," << calc_rfoot << "," << calc_lfoot << endl;
}

void WalkingController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{ 
  ref_zmp_.resize(norm_size, 2);
  ref_sup_x_.resize(norm_size, 6); ref_sup_x_.setZero();
  ref_sup_y_.resize(norm_size, 6); ref_sup_y_.setZero();
  Eigen::VectorXd temp_px;
  Eigen::VectorXd temp_py;

  unsigned int index = 0; 
  ZMP_margin_(0) = 0.1;
  ZMP_margin_(1) = 0.05;

  // init ZMP
  if(current_step_num_ == 0)
  {
    for (int i = 0; i <= t_temp_; i++)  
    {
      if(i < 0.5*hz_) 
      {
        ref_zmp_(i,0) = com_support_init_(0) ;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else if(i < 1.5*hz_) 
      {
        ref_zmp_(i,0) = com_support_init_(0) - com_support_init_(0)*(i - 0.5*hz_)/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else 
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1) ;
      } 
      ref_sup_x_(i,0) = 0.0 + ZMP_margin_(0);
      ref_sup_x_(i,1) = 0.0 - ZMP_margin_(0);
      ref_sup_x_(i,2) = 0.0 - ZMP_margin_(0);
      ref_sup_x_(i,3) = 0.0 + ZMP_margin_(0);
      ref_sup_x_(i,4) = 0.0 + ZMP_margin_(0);
      ref_sup_x_(i,5) = 0.0 - ZMP_margin_(0);

      ref_sup_y_(i,0) = 0.0 + ZMP_margin_(1);
      ref_sup_y_(i,1) = 0.0 + ZMP_margin_(1);
      ref_sup_y_(i,2) = foot_step_support_frame_(0,1) - ZMP_margin_(1);
      ref_sup_y_(i,3) = foot_step_support_frame_(0,1) - ZMP_margin_(1);
      ref_sup_y_(i,4) = 0.0 + ZMP_margin_(1);
      ref_sup_y_(i,5) = 0.0 + ZMP_margin_(1);
    }   
    index = index + t_temp_; 
  }

  //walking ZMP
  if(current_step_num_ >= total_step_num_ - planning_step_num)
  {  
    for(unsigned int i = current_step_num_; i < total_step_num_; i++)
    {
      onestepZmp(i, temp_px, temp_py);
     
      for(unsigned int j = 0; j < t_total_; j++)
      {
        ref_zmp_(index + j, 0) = temp_px(j);
        ref_zmp_(index + j, 1) = temp_py(j);

        ref_sup_x_(index + j,0) = temp_sup_x_(j,0);
        ref_sup_x_(index + j,1) = temp_sup_x_(j,1);
        ref_sup_x_(index + j,2) = temp_sup_x_(j,2);
        ref_sup_x_(index + j,3) = temp_sup_x_(j,3);

        ref_sup_y_(index + j,0) = temp_sup_y_(j,0);
        ref_sup_y_(index + j,1) = temp_sup_y_(j,1);
        ref_sup_y_(index + j,2) = temp_sup_y_(j,2);
        ref_sup_y_(index + j,3) = temp_sup_y_(j,3);
      }
      index = index + t_total_;
    }

  // last ZMP    
    for(unsigned int j = 0; j < 20*hz_; j++)
    {
      ref_zmp_(index + j, 0) = ref_zmp_(index -1, 0);
      ref_zmp_(index + j, 1) = ref_zmp_(index -1, 1);

      ref_sup_x_(index + j,0) = temp_sup_x_(j,0);
      ref_sup_x_(index + j,1) = temp_sup_x_(j,1);
      ref_sup_x_(index + j,2) = temp_sup_x_(j,2);
      ref_sup_x_(index + j,3) = temp_sup_x_(j,3);

      ref_sup_y_(index + j,0) = temp_sup_y_(j,0);
      ref_sup_y_(index + j,1) = temp_sup_y_(j,1);
      ref_sup_y_(index + j,2) = temp_sup_y_(j,2);
      ref_sup_y_(index + j,3) = temp_sup_y_(j,3);
    }
    index = index + 20*hz_;      
  }
  else  
  { 
    for(unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)  
    {
      onestepZmp(i, temp_px, temp_py);

      for (unsigned int j = 0; j < t_total_; j++)  
      {
        ref_zmp_(index + j,0) = temp_px(j);
        ref_zmp_(index + j,1) = temp_py(j);

        ref_sup_x_(index + j,0) = temp_sup_x_(j,0);
        ref_sup_x_(index + j,1) = temp_sup_x_(j,1);
        ref_sup_x_(index + j,2) = temp_sup_x_(j,2);
        ref_sup_x_(index + j,3) = temp_sup_x_(j,3);

        ref_sup_y_(index + j,0) = temp_sup_y_(j,0);
        ref_sup_y_(index + j,1) = temp_sup_y_(j,1);
        ref_sup_y_(index + j,2) = temp_sup_y_(j,2);
        ref_sup_y_(index + j,3) = temp_sup_y_(j,3);
      }      
      index = index + t_total_;  
    }   
  }
}

void WalkingController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_);  
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  temp_sup_x_.resize(t_total_,6);
  temp_sup_y_.resize(t_total_,6);
  temp_sup_x_.setZero();
  temp_sup_y_.setZero();

  double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0;
  if(current_step_number == 0)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = -(foot_step_support_frame_(current_step_number, 1))/2 ;
    B =  (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45));
        
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      {
        temp_px(i) = 0;
        temp_py(i) = (com_offset_(1) + com_support_init_(1)) + Ky / (t_rest_init_ + t_double1_)* (i+1);

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_ ) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = supportfoot_support_init_(0);
        temp_py(i) = supportfoot_support_init_(1) - zmp_offset_;

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,4) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,5) = temp_px(i) - ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,4) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,5) = temp_py(i) + ZMP_margin_(1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick 
      {
        temp_px(i) = B - Kx + Kx / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
      }
    }
  }
  else if(current_step_number == 1)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - supportfoot_support_init_(1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 10 ~ 30 tick
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1, 0) + supportfoot_support_init_(0))/2 + Kx / (t_rest_init_+ t_double1_) * (i+1);
        temp_py(i) = (foot_step_support_frame_(current_step_number-1, 1) + supportfoot_support_init_(1))/2 + Ky / (t_rest_init_+ t_double1_) * (i+1);

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_(current_step_number-1, 0);
        temp_py(i) = foot_step_support_frame_(current_step_number-1, 1) + zmp_offset_;

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,4) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,5) = temp_px(i) - ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,4) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,5) = temp_py(i) + ZMP_margin_(1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
      {               
        temp_px(i) = (foot_step_support_frame_(current_step_number-1, 0) + foot_step_support_frame_(current_step_number, 0))/2 - Kx + Kx /(t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
      }
    }
  }
  else
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - foot_step_support_frame_(current_step_number-2, 1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45))) ;
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    for(int i = 0; i < t_total_; i++)
    {      
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      { 
        temp_px(i) = (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Kx/(t_rest_init_ + t_double1_)*(i+1);
        temp_py(i) = (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Ky/(t_rest_init_ + t_double1_)*(i+1);

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_)) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_(current_step_number-1, 0) ;
        temp_py(i) = foot_step_support_frame_(current_step_number-1, 1) + pow(-1,current_step_number-1)*zmp_offset_;

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,4) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,5) = temp_px(i) - ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,4) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,5) = temp_py(i) + ZMP_margin_(1);
      }
      else if( i >= (t_total_ - t_rest_last_ - t_double2_) && (i < t_total_) && (current_step_num_ == total_step_num_ - 1))
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number-1, 0))/2;
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
      }       
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
      { 
        temp_px(i) = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));

        temp_sup_x_(i,0) = temp_px(i) + ZMP_margin_(0);
        temp_sup_x_(i,1) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,2) = temp_px(i) - ZMP_margin_(0);
        temp_sup_x_(i,3) = temp_px(i) + ZMP_margin_(0);

        temp_sup_y_(i,0) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,1) = temp_py(i) + ZMP_margin_(1);
        temp_sup_y_(i,2) = temp_py(i) - ZMP_margin_(1);
        temp_sup_y_(i,3) = temp_py(i) - ZMP_margin_(1);
      }
    } 
  } 
}

void WalkingController::getComTrajectory()
{
  unsigned int planning_step_num_ = 3;
  unsigned int planning_com_size_tick = 0;
  
  planning_com_size_tick = t_total_*(min((int)(total_step_num_ - current_step_num_), (int)planning_step_num_)) // time for rest step
                         + (1-(bool)current_step_num_)*(t_temp_ + 1)                                           // time for init
                         + (current_step_num_>=total_step_num_ - planning_step_num_)*20*hz_;                   // time for last

  height_diff = 0.0;
  if(walking_tick_ >= 200)
  {
    if(foot_step_support_frame_(current_step_num_,0) > 0)
    {
      height_diff = 0.2;
    }
  }
  Eigen::VectorXd com_int_; com_int_.resize(total_step_num_); com_int_.setZero();
  com_int_(1) = 1; com_int_(2) = 1;
  com_height_int(com_int_);
  comGenerator(planning_com_size_tick, planning_step_num_);

  com_desired_.segment<2>(0) = ref_com_.block<1,2>(walking_tick_ - (bool)current_step_num_*t_start_,0);
  //com_desired_(2) = pelv_support_start_.translation()(2);
  com_desired_(2) = ref_com_z_(walking_tick_ - (bool)current_step_num_*t_start_,0);

  //getCom_Z_preview_control();
  getCom_Z_mpc();
  //getCom_preview_control();
  getCom_mpc();
  //getCom_preview_control_CPM();
}

void WalkingController::com_height_int(Eigen::VectorXd& com_int)
{
  Eigen::VectorXd temp_calc; temp_calc.resize(total_step_num_); temp_calc.setZero();
  for(int i = 0; i < total_step_num_; i++)
  {
    if(com_int(i) == 1)
    {
      if(current_step_num_ < i+1)
      {
        for(int j = i; j < total_step_num_; j++)
        {
          temp_calc(j) = temp_calc(j) - height_diff;
        }
      }
      else
      {
        for(int j = 0; j < i; j++)
        {
          temp_calc(j) = temp_calc(j) + height_diff;
        }
      }
    }
    foot_step_support_frame_(i,2) = temp_calc(i);
  }
}

void WalkingController::comGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{ 
  ref_com_.resize(norm_size, 2);
  ref_com_z_.resize(norm_size,1);
  Eigen::VectorXd temp_cx;
  Eigen::VectorXd temp_cy;
  Eigen::VectorXd temp_cz;
  
  unsigned int index = 0; 

  if(current_step_num_ == 0)   
  {
    for (int i = 0; i <= t_temp_; i++)  
    {
      double wn, A, Ky;
      wn = sqrt(GRAVITY / com_support_init_(2));
      A = -(foot_step_support_frame_(0, 1))/2 ;
      Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45));

      ref_com_(i,0) = DyrosMath::cubic(i, 0.0, 2.0*hz_, com_support_init_(0), 0.0, 0.0, 0.0);
      Eigen::Vector3d ref_com_y_temp = DyrosMath::QuinticSpline(i, 2.0*hz_, t_temp_, com_support_init_(1), 0.0, 0.0, com_support_init_(1), Ky/(t_rest_init_ + t_double1_), 0.0);
      ref_com_(i,1) = ref_com_y_temp(0);
      ref_com_z_(i,0) = 0.72;

      index++;
    }    
  }

  if(current_step_num_ >= total_step_num_ - planning_step_num)
  {  
    for(unsigned int i = current_step_num_; i < total_step_num_; i++)
    {
      onestepCom(i, temp_cx, temp_cy);
      onestepComz(i, temp_cz);
     
      for(unsigned int j = 0; j < t_total_; j++)
      {
        ref_com_(index + j, 0) = temp_cx(j);
        ref_com_(index + j, 1) = temp_cy(j);
        ref_com_z_(index + j, 0) = temp_cz(j);
      }
      index = index + t_total_;
    }
    
    for(unsigned int j = 0; j < 20*hz_; j++)
    {
      ref_com_(index + j, 0) = ref_com_(index -1, 0);
      ref_com_(index + j, 1) = ref_com_(index -1, 1);
      ref_com_z_(index + j, 0) = ref_com_z_(index -1, 0);
    }

    if((current_step_num_ == total_step_num_ - 1)) 
    {        
      for(int i = 0; i < t_total_; i++)
      {
        //econom2
        Eigen::Vector3d ref_com_y_temp= DyrosMath::QuinticSpline(i + 239, 120, 479, 0.031081, -2.60209e-18, 1.05331e-05, 0.12779, 0, 0);
        ref_com_(index + i, 1) = ref_com_y_temp(0) ;
      }
    }

    index = index + 20*hz_;      
  }
  else  
  { 
    for(unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)  
    {
      onestepCom(i, temp_cx, temp_cy);
      onestepComz(i, temp_cz);

      for (unsigned int j = 0; j < t_total_; j++)  
      {
        ref_com_(index+j,0) = temp_cx(j);
        ref_com_(index+j,1) = temp_cy(j);
        ref_com_z_(index+j,0) = temp_cz(j);
      }      
      index = index + t_total_;  
    }   
  }  
}

void WalkingController::onestepCom(unsigned int current_step_number, Eigen::VectorXd& temp_cx, Eigen::VectorXd& temp_cy)
{
  temp_cx.resize(t_total_);  
  temp_cy.resize(t_total_);
  temp_cx.setZero();
  temp_cy.setZero();

  double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0;
  double Cx1 = 0, Cx2 = 0, Cy1 = 0, Cy2 = 0;

  if(current_step_number == 0)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = -(foot_step_support_frame_(current_step_number, 1))/2 ;
    B =  (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45));
    Cx1 = Kx - B;
    Cx2 = Kx/(0.15*wn);
    Cy1 = Ky - A;
    Cy2 = Ky/(0.15*wn);
    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      {
        temp_cx(i) = 0;
        temp_cy(i) = (com_offset_(1) + com_support_init_(1)) + Ky / (t_rest_init_ + t_double1_)* (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_ ) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_cx(i) = DyrosMath::cubic(i, t_rest_init_ + t_double1_, t_total_ - t_rest_last_ - t_double2_, 0, B - Kx, 0.0, Kx/(0.15*hz_));
        temp_cy(i) = com_support_init_(1) + Cy1*cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_ - 0.15)) + A;
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick 
      {
        temp_cx(i) = B - Kx + Kx / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_cy(i) = Ky + (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }    
  }
  else if(current_step_number == 1)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - supportfoot_support_init_(1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    Cx1 = Kx - B;
    Cx2 = Kx/(0.15*wn);
    Cy1 = Ky - A;
    Cy2 = Ky/(0.15*wn);
    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 10 ~ 30 tick
      {
        temp_cx(i) = (foot_step_support_frame_(current_step_number-1, 0) + supportfoot_support_init_(0))/2 + Kx / (t_rest_init_+ t_double1_) * (i+1);
        temp_cy(i) = (foot_step_support_frame_(current_step_number-1, 1) + supportfoot_support_init_(1))/2 + Ky / (t_rest_init_+ t_double1_) * (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_cx(i) = (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Cx1*cosh(wn*(i/hz_ - 0.15)) + Cx2*sinh(wn*(i/hz_ - 0.15)) + B;
        temp_cy(i) = (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Cy1*cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_ - 0.15)) + A;
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
      {               
        temp_cx(i) = (foot_step_support_frame_(current_step_number-1, 0) + foot_step_support_frame_(current_step_number, 0))/2 - Kx + Kx /(t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_cy(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }
  }
  else
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - foot_step_support_frame_(current_step_number-2, 1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45))) ;
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    Cx1 = Kx - B;
    Cx2 = Kx/(0.15*wn);
    Cy1 = Ky - A;
    Cy2 = Ky/(0.15*wn);

    for(int i = 0; i < t_total_; i++)
    {      
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      { 
        temp_cx(i) = (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Kx/(t_rest_init_ + t_double1_)*(i+1);
        temp_cy(i) = (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Ky/(t_rest_init_ + t_double1_)*(i+1);
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_)) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_cx(i) = (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Cx1*cosh(wn*(i/hz_ - 0.15)) + Cx2*sinh(wn*(i/hz_ - 0.15)) + B;
        temp_cy(i) = (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Cy1*cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_ - 0.15)) + A;
      } 
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
      { 
        temp_cx(i) = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_cy(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }

      if(i >= (t_rest_init_ + t_double1_) && (current_step_number == total_step_num_ - 1) && i < t_total_)
      {
        double temp_x1 = (foot_step_support_frame_(current_step_number-2,0) + foot_step_support_frame_(current_step_number-1,0))/2 + Kx;
        double temp_x2 = foot_step_support_frame_(current_step_number-1,0);
        Eigen::Vector3d ref_com_x_temp = DyrosMath::QuinticSpline(i, t_rest_init_ + t_double1_, t_total_, temp_x1, Kx/(0.15*hz_), 0.0, temp_x2, 0.0, 0.0);
        temp_cx(i) = ref_com_x_temp(0);

        //double temp_y1 = (foot_step_support_frame_(current_step_number-2,1) + foot_step_support_frame_(current_step_number-1,1))/2 + Ky;
        //double temp_y2 = foot_step_support_frame_(current_step_number-1,1) + B + Ky;
        //Eigen::Vector3d ref_com_y_temp = DyrosMath::QuinticSpline(i, t_rest_init_ + t_double1_, t_total_, temp_y1, Ky/(0.15*hz_), 0.0, temp_y2, 0.0, 0.0);
        if(i >= 0.5*t_total_)
        {
          Eigen::Vector3d ref_com_y_temp = DyrosMath::QuinticSpline(i, 120, 2*t_total_ - 1, 0.031081, -2.60209e-18, 1.05331e-05, 0.12779, 0, 0);
          temp_cy(i) = ref_com_y_temp(0);
        }
      }
    } 
  } 
}

void WalkingController::onestepComz(unsigned int current_step_number, Eigen::VectorXd& temp_cz)
{
  temp_cz.resize(t_total_);
  temp_cz.setZero();
  double h_diff;
  h_diff = foot_step_support_frame_(current_step_number,2);
  for(int i = 0; i < t_total_; i++)
  {
    if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 10 ~ 30 tick
    {
      temp_cz(i) = zc_ + h_diff;
    }
    else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
    {
      temp_cz(i) = zc_ + h_diff;
    }
    else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
    {               
      temp_cz(i) = zc_ + h_diff;
    }
  }
}

void WalkingController::previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, 
       Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{
  int zmp_size;
  zmp_size = ref_zmp_.col(1).size();
  Eigen::VectorXd px_ref, py_ref;
  px_ref.resize(zmp_size);
  py_ref.resize(zmp_size);
  
  for(int i = 0; i < zmp_size; i++)
  {
      px_ref(i) = ref_zmp_(i,0);
      py_ref(i) = ref_zmp_(i,1);
  }

  Eigen::VectorXd px, py;
  px.resize(1); py.resize(1);
  
  if(tick == 0 && current_step_num_ == 0)
  {
      preview_x_b.setZero(); preview_y_b.setZero();
      preview_x.setZero(); preview_y.setZero();
      preview_x_b(0) = x_i;  
      preview_y_b(0) = y_i;   
      preview_x(0) = x_i;
      preview_y(0) = y_i;
      UX = 0; UY = 0;
  }
  else
  {     
      preview_x = xs; preview_y = ys;
          
      preview_x_b(0) = preview_x(0) - preview_x(1)*0.005;  
      preview_y_b(0) = preview_y(0) - preview_y(1)*0.005;
      preview_x_b(1) = preview_x(1) - preview_x(2)*0.005;
      preview_y_b(1) = preview_y(1) - preview_y(2)*0.005;
      preview_x_b(2) = preview_x(2) - UX*0.005;
      preview_y_b(2) = preview_y(2) - UY*0.005;
      
  }  
  px = C*preview_x;
  py = C*preview_y;

  double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;
  for(int i = 0; i < NL; i++)
  {
      sum_Gd_px_ref = sum_Gd_px_ref + Gd(i)*(px_ref(tick + 1 + i) - px_ref(tick + i));
      sum_Gd_py_ref = sum_Gd_py_ref + Gd(i)*(py_ref(tick + 1 + i) - py_ref(tick + i));
  }
  
  Eigen::MatrixXd del_ux(1,1);
  Eigen::MatrixXd del_uy(1,1);
  del_ux.setZero();
  del_uy.setZero();
  
  Eigen::VectorXd GX_X(1);
  GX_X = Gx * (preview_x - preview_x_b);
  Eigen::VectorXd GX_Y(1);
  GX_Y = Gx * (preview_y - preview_y_b);
  
  del_ux(0,0) = -(px(0) - px_ref(tick))*Gi(0,0) - GX_X(0) - sum_Gd_px_ref;
  del_uy(0,0) = -(py(0) - py_ref(tick))*Gi(0,0) - GX_Y(0) - sum_Gd_py_ref;

  UX = UX + del_ux(0,0);
  UY = UY + del_uy(0,0);

  XD = A*preview_x + B*UX;
  YD = A*preview_y + B*UY;
}

void WalkingController::getCom_preview_param()
{
  const double dt = 1/hz_;
  A_.resize(3,3);
  A_ << 1, dt, dt*dt/2,
        0,  1,      dt,
        0,  0,       1;

  B_.resize(3);
  B_ << dt*dt*dt/6,
           dt*dt/2,
              dt/1;

  C_.resize(1,3);
  C_ << 1, 0, -zc_/GRAVITY;

  Eigen::MatrixXd B_bar;
  B_bar.resize(4,1);
  B_bar.block<1,1>(0,0) = C_*B_;
  B_bar.block<3,1>(1,0) = B_;
  
  Eigen::MatrixXd I_bar;
  I_bar.resize(4,1);
  I_bar << 1, 0, 0, 0;

  Eigen::MatrixXd F_bar;
  F_bar.resize(4,3);

  F_bar.block<1,3>(0,0) = C_*A_;
  F_bar.block<3,3>(1,0) = A_;
  
  Eigen::MatrixXd A_bar;
  A_bar.resize(4,4);
  A_bar.block<4,1>(0,0) = I_bar;
  A_bar.block<4,3>(0,1) = F_bar;
  
  Eigen::MatrixXd R;
  R.resize(1,1);
  R << 1e-6;
  
  Eigen::Matrix4d K;
  K.resize(4,4);
  K(0,0) = 1.108105414159645e+02;
  K(0,1) = 6.084082768738899e+03;  
  K(0,2) = 1.663971575043506e+03;
  K(0,3) = 4.261729529912996;
  K(1,0) = K(0,1);
  K(1,1) = 3.413861906244857e+05;
  K(1,2) = 9.339362231865562e+04;
  K(1,3) = 2.461512775255278e+02;
  K(2,0) = K(0,2);
  K(2,1) = K(1,2);
  K(2,2) = 2.555016951915173e+04;
  K(2,3) = 67.424716130276040;
  K(3,0) = K(0,3);
  K(3,1) = K(1,3);
  K(3,2) = K(2,3);
  K(3,3) = 0.201166173370332;
  
  Eigen::MatrixXd R_Btran_K_B_;
  R_Btran_K_B_.resize(1,1);
  R_Btran_K_B_ = R + B_bar.transpose()*K*B_bar;

  Eigen::MatrixXd Ac_bar;
  Ac_bar = A_bar - B_bar*R_Btran_K_B_.inverse()*B_bar.transpose()*K*A_bar;
  
  Gi_ = R_Btran_K_B_.inverse()*B_bar.transpose()*K*I_bar;
  Gx_ = R_Btran_K_B_.inverse()*B_bar.transpose()*K*F_bar;

  Eigen::MatrixXd X_bar;
  Eigen::Vector4d X_bar_col;
  X_bar.resize(4, int(1.6*hz_));
  X_bar.setZero();
  X_bar_col.setZero();
  X_bar_col = - Ac_bar.transpose() * K * I_bar;
  for(int i = 0; i < int(1.6*hz_); i++)
  {
      X_bar.block<4,1>(0,i) = X_bar_col;
      X_bar_col = Ac_bar.transpose()*X_bar_col;
  }
  
  Eigen::VectorXd Gd_col(1);
  Gd_col(0) = - Gi_(0,0);
  
  for(int i = 0; i < int(1.6*hz_); i++)
  {
      Gd_.segment(i,1) = Gd_col;
      Gd_col = R_Btran_K_B_.inverse() * B_bar.transpose() * X_bar.col(i) ;
  }
}

void WalkingController::getCom_preview_control()
{
  if(walking_tick_ == 0)  
  {
    Gi_.resize(1,1);           Gi_.setZero();
    Gx_.resize(1,3);           Gx_.setZero();
    Gd_.resize(int(1.6*hz_));  Gd_.setZero();

    getCom_preview_param();
    xs_(0) = xi_; xs_(1) = 0; xs_(2) = 0;
    ys_(0) = yi_; ys_(1) = 0; xs_(2) = 0;
    UX_ = 0; UY_ = 0;
    xd_ = xs_;
  }

  zmp_start_time_ = (bool)current_step_num_*t_start_;
  
  previewcontroller(1/hz_, int(1.6*hz_), walking_tick_ - zmp_start_time_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, C_, xd_, yd_);
   
  xs_ = xd_; ys_ = yd_;

  com_desired_(0) = xd_(0);
  com_desired_(1) = yd_(0);
  //com_desired_(2) = 0.72;//pelv_support_start_.translation()(2);

  com_desired_dot_(0) = xd_(1);
  com_desired_dot_(1) = yd_(1);

  if(walking_tick_ == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1)  
  {
    Eigen::Vector3d com_pos_prev;
    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel_prev;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d com_acc_prev;
    Eigen::Vector3d com_acc;
    Eigen::Matrix3d temp_rot;
    Eigen::Vector3d temp_pos;
    
    temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_,5));
    for(int i=0; i<3; i++)
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);     
    
    com_pos_prev(0) = xs_(0);
    com_pos_prev(1) = ys_(0);
    com_pos = temp_rot*(com_pos_prev - temp_pos);
     
    com_vel_prev(0) = xs_(1);
    com_vel_prev(1) = ys_(1);
    com_vel_prev(2) = 0.0;
    com_vel = temp_rot*com_vel_prev;

    com_acc_prev(0) = xs_(2);
    com_acc_prev(1) = ys_(2);
    com_acc_prev(2) = 0.0;
    com_acc = temp_rot*com_acc_prev;

    xs_(0) = com_pos(0);
    ys_(0) = com_pos(1);
    xs_(1) = com_vel(0);
    ys_(1) = com_vel(1);
    xs_(2) = com_acc(0);
    ys_(2) = com_acc(1);
  }
}

void WalkingController::getCom_mpc()
{
  //MPC Param
  double mpc_hz_ = 10; double calc_hz_ = hz_/mpc_hz_;
  double mpc_dt_ = 1/mpc_hz_;
  Eigen::MatrixXd mpc_A_xy_, mpc_B_xy_;
  mpc_A_xy_.resize(6,6); mpc_B_xy_.resize(6,2);
  mpc_A_xy_ << 1, mpc_dt_, mpc_dt_*mpc_dt_/2,   0,       0,                 0,
               0,       1,         mpc_dt_/1,   0,       0,                 0,
               0,       0,                 1,   0,       0,                 0,
               0,       0,                 0,   1, mpc_dt_, mpc_dt_*mpc_dt_/2,
               0,       0,                 0,   0,       1,         mpc_dt_/1,
               0,       0,                 0,   0,       0,                 1;
  
  mpc_B_xy_ << mpc_dt_*mpc_dt_*mpc_dt_/6,                         0,
                       mpc_dt_*mpc_dt_/2,                         0,
                               mpc_dt_/1,                         0,
                                       0, mpc_dt_*mpc_dt_*mpc_dt_/6,
                                       0,         mpc_dt_*mpc_dt_/2,
                                       0,                 mpc_dt_/1;
          
  Eigen::MatrixXd Czmp_xy_;
  Czmp_xy_.resize(2,6);
  Czmp_xy_ << 1, 0, -0.72/GRAVITY, 0, 0,             0,
              0, 0,             0, 1, 0, -0.72/GRAVITY;

  const int NL_z = 1.6*mpc_hz_;
  Eigen::MatrixXd Pzs_xy_;
  Pzs_xy_.resize(2*NL_z,6);

  Eigen::MatrixXd Pzs_calc_xy_;
  Pzs_calc_xy_.resize(6,6);
  Pzs_calc_xy_ = mpc_A_xy_;
  
  for(int i = 0; i < NL_z; i++)
  {
    Czmp_xy_(0,2) = - MPC_z_prev_(0,i)/(GRAVITY + MPC_z_prev_(2,i));
    Czmp_xy_(1,5) = - MPC_z_prev_(0,i)/(GRAVITY + MPC_z_prev_(2,i));

    Pzs_xy_.block<2,6>(2*i,0) = Czmp_xy_*Pzs_calc_xy_;
    Pzs_calc_xy_ = Pzs_calc_xy_*mpc_A_xy_;
  }

  Eigen::MatrixXd Pzu_xy_;
  Pzu_xy_.resize(2*NL_z,2*NL_z);
  Pzu_xy_.setZero();
  Eigen::MatrixXd Pzu_calc_xy_;
  Pzu_calc_xy_.resize(6,2*NL_z); Pzu_calc_xy_.setZero();
  for(int i = 0; i < NL_z; i++)
  {
    Czmp_xy_(0,2) = - MPC_z_prev_(0,i)/(GRAVITY + MPC_z_prev_(2,i));
    Czmp_xy_(1,5) = - MPC_z_prev_(0,i)/(GRAVITY + MPC_z_prev_(2,i));

    if(i == 0)
    {
      Pzu_calc_xy_.block<6,2>(0,2*i) = mpc_B_xy_;
    }
    else
    {
      Pzu_calc_xy_.block<6,2>(0,2*i) = mpc_B_xy_;
    }
    Pzu_xy_.block<2, 32>(2*i,0) = Czmp_xy_*Pzu_calc_xy_;
    Pzu_calc_xy_ = mpc_A_xy_*Pzu_calc_xy_;
  }

  //MPC Control
  if(walking_tick_ == 0)
  {
    MPC_xy_.setZero();

    MPC_xy_(0) = com_support_init_(0);
    MPC_xy_(3) = com_support_init_(1);
  }

  Eigen::MatrixXd mpc_zmpxy_ref, mpc_ref_zmpx_calc_, mpc_ref_zmpy_calc_;
  
  double tick_start_ = walking_tick_ - (bool)current_step_num_*t_start_;

  mpc_ref_zmpx_calc_.resize(320,1); mpc_ref_zmpy_calc_.resize(320,1);

  for(int i = 0; i < 320; i++)
  {
    mpc_ref_zmpx_calc_(i,0) = ref_zmp_(tick_start_ + i,0);
    mpc_ref_zmpy_calc_(i,0) = ref_zmp_(tick_start_ + i,1);
  }

  mpc_zmpxy_ref.resize(2*NL_z, 1);
  
  for(int i = 0; i < NL_z; i++)
  {
    if(i == 0)
    {
      mpc_zmpxy_ref(2*i,0)     = mpc_ref_zmpx_calc_((i+1) * calc_hz_ - 1, 0);
      mpc_zmpxy_ref(2*i + 1,0) = mpc_ref_zmpy_calc_((i+1) * calc_hz_ - 1, 0);
    }
    else
    {
      mpc_zmpxy_ref(2*i,0)     = mpc_ref_zmpx_calc_((i+1) * calc_hz_ - 1, 0);
      mpc_zmpxy_ref(2*i + 1,0) = mpc_ref_zmpy_calc_((i+1) * calc_hz_ - 1, 0);
    }
  }

  Eigen::MatrixXd Qz, R;
  Qz.resize(2*NL_z, 2*NL_z); R.resize(2*NL_z, 2*NL_z);
  Qz.setIdentity();      R.setIdentity();
  Qz = 1*Qz;             R = 1e-6*R;

  Eigen::MatrixXd Qz_calc, gxy_calc;
  Qz_calc.resize(2*NL_z, 2*NL_z); Qz_calc.setZero();
  Qz_calc = Pzu_xy_.transpose()*Qz*Pzu_xy_ + R;

  gxy_calc.resize(2*NL_z, 1); gxy_calc.setZero();
  gxy_calc = (Pzu_xy_.transpose()*Qz*Pzs_xy_)*MPC_xy_ - Pzu_xy_.transpose()*Qz*mpc_zmpxy_ref;

  Eigen::MatrixXd uOptxy;
  uOptxy.resize(2*NL_z,1);

  Eigen::MatrixXd A_ieq, b_ieq;
  A_ieq.resize(2*NL_z,NL_z); b_ieq.resize(2*NL_z,1);
  A_ieq.setConstant(1);      b_ieq.setConstant(100);
  
  //qpoases_solve(2*NL_z, 2*NL_z, Qz_calc, gxy_calc, uOptxy, A_ieq, b_ieq);
  qpoases_solve(2*NL_z, 2*NL_z, Qz_calc, gxy_calc, uOptxy);
  

  Eigen::Vector6d MPC_xy_calc_;
  MPC_xy_calc_.setZero();
  
  MPC_xy_calc_ = mpc_A_xy_*MPC_xy_ + mpc_B_xy_*uOptxy.block<2,1>(0,0);

  MPC_xy_calc_ = (1/calc_hz_)*MPC_xy_calc_ + (1 - 1/calc_hz_)*MPC_xy_;

  MPC_xy_ = MPC_xy_calc_;
  
  com_desired_(0) = MPC_xy_(0);
  com_desired_(1) = MPC_xy_(3);

  double mpc_zmp_x_, mpc_zmp_y_;
  mpc_zmp_x_ = MPC_xy_(0) - (MPC_z_prev_(0,0) * MPC_xy_(2))/(GRAVITY + MPC_z_prev_(2,0));
  mpc_zmp_y_ = MPC_xy_(3) - (MPC_z_prev_(0,0) * MPC_xy_(5))/(GRAVITY + MPC_z_prev_(2,0));

  e_mpc_graphxy << MPC_xy_(0) << "," << MPC_xy_(1) << "," << MPC_xy_(2) << "," << MPC_xy_(3) << "," << MPC_xy_(4) << "," << MPC_xy_(5) << "," << mpc_zmp_x_ << "," << mpc_zmp_y_ << endl;

  if(tick_start_ == t_last_ - (bool)current_step_num_*t_start_ && current_step_num_ != total_step_num_ - 1)
  {
    MPC_xy_(0) = MPC_xy_(0) - foot_step_support_frame_(current_step_num_,0);
    MPC_xy_(3) = MPC_xy_(3) - foot_step_support_frame_(current_step_num_,1);
  }
}

void WalkingController::getCom_Z_preview_param()
{
  C_z_.resize(2,3);
  C_z_ << 1, 0, 0, 0, 0, 43;

  Eigen::MatrixXd QN, P_temp, R_BTPB_calc, K_temp;
  Q_z_.resize(2,2);
  Q_z_.setZero();
  Q_z_(0,0) = 2e2;
  Q_z_(1,1) = 8e-4;

  QN.resize(3,3);
  QN = C_z_.transpose()*Q_z_*C_z_;

  R_z_.resize(1,1);
  R_z_ << 1e-8;

  P_temp.resize(3,3);
  P_z_ss_.resize(3,3);

  P_temp = QN;
  
  R_BTPB_calc.resize(1,1);

  K_temp.resize(1,3);
  K_z_ss_.resize(1,3);

  double N = 500;
  for(int i = N; i >0; i--)
  {
    R_BTPB_calc = R_z_ + B_.transpose()*P_temp*B_;
    P_temp = A_.transpose()*P_temp*A_+ QN - A_.transpose()*P_temp*B_*R_BTPB_calc.inverse()*B_.transpose()*P_temp*A_;
    double K_temp_comp = K_temp(0,0);
    K_temp = - R_BTPB_calc.inverse()*B_.transpose()*P_temp*A_;
    
    if(abs(K_temp(0,0) - K_temp_comp) < 0.001)
    { break;}
  }

  P_z_ss_ = P_temp;
  K_z_ss_ = K_temp;
}

void WalkingController::getCom_Z_preview_control()
{
  if(walking_tick_ == 0)
  {
    getCom_Z_preview_param();
    preview_z_.setZero();
    preview_z_(0) = 0.72533;
  }

  int comz_size;
  comz_size = ref_com_z_.col(1).size();
  Eigen::MatrixXd z_ref;
  z_ref.resize(2,comz_size);
  
  for(int i = 0; i < comz_size; i++)
  {
      z_ref(0,i) = ref_com_z_(i,0);
      z_ref(1,i) = 0.0;
  }

  Eigen::VectorXd z_real;
  z_real.resize(2);
  
  Eigen::MatrixXd qz;
  double NL_z = 2.0*hz_;
  qz.resize(3,NL_z);
  qz.setZero();

  for(int i = NL_z; i > 0; i--)
  {
    if(i == NL_z)
    {
      qz.block<3,1>(0,i-1) = - C_z_.transpose()*Q_z_*z_ref.block<2,1>(0,i-1);
    }
    else
    {
      qz.block<3,1>(0,i-1) = (A_ + B_*K_z_ss_).transpose()*qz.block<3,1>(0,i) - C_z_.transpose()*Q_z_*z_ref.block<2,1>(0,i-1);
    }
  }

  Eigen::MatrixXd R_BTPB_calc, gz1, uz;
  R_BTPB_calc.resize(1,1);
  R_BTPB_calc = R_z_ + B_.transpose()*P_z_ss_*B_;

  gz1.resize(1,1);
  gz1 = - R_BTPB_calc.inverse()*B_.transpose()*qz.block<3,1>(0,1);

  uz = K_z_ss_*preview_z_ + gz1;
  z_real = C_z_*preview_z_;
  preview_z_ = A_*preview_z_ + B_*uz;

  com_desired_(2) = z_real(0);
}

//void WalkingController::qpoases_solve(int num_var_s, int num_con_s, Eigen::MatrixXd Q_calc_s, Eigen::MatrixXd g_calc_s, Eigen::MatrixXd& uOpt_s, Eigen::MatrixXd& A_ieq_s, Eigen::MatrixXd& b_ieq_s)
void WalkingController::qpoases_solve(int num_var_s, int num_con_s, Eigen::MatrixXd Q_calc_s, Eigen::MatrixXd g_calc_s, Eigen::MatrixXd& uOpt_s)
{
  const int num_var = num_var_s, num_con = num_con_s;
  qpOASES::real_t H_input[num_var_s*num_var_s], g_input[num_var_s], A_input[num_con_s*num_var_s], lbA_input[num_con_s], ubA_input[num_con_s], lb[num_var_s], ub[num_var_s];
  for(int i = 0; i < num_var_s; i++)
  {
    for(int j = 0; j < num_var_s; j++)
    {
      H_input[num_var_s*i + j] = Q_calc_s(i,j);
    }
    g_input[i] = g_calc_s(i,0);
    lb[i] = -100;
    ub[i] = 100;
  }
  

  for(int i = 0; i < num_con_s; i++)
  {
    for(int j = 0; j < num_var_s; j++)
    {
      A_input[num_var_s*i + j] = 1;//A_ieq_s(i,j);
    }
    lbA_input[i] = -100;
    ubA_input[i] = 100;//b_ieq_s(i);
  }
     
  qpOASES::real_t xOpt[num_var_s];
  qpOASES::QProblem test(num_var_s,num_con_s);

  qpOASES::Options optionsMPCZ;
  optionsMPCZ.setToMPC();
  optionsMPCZ.boundRelaxation = 1e-4;
  optionsMPCZ.printLevel = qpOASES::PL_NONE;

  test.setOptions(optionsMPCZ);

  qpOASES::int_t nWSR1= 10;

  test.init(H_input,g_input,A_input,lb,ub,lbA_input,ubA_input,nWSR1);
  test.getPrimalSolution(xOpt);

  for(int i = 0; i < num_var_s; i++)
  {
    uOpt_s(i) = xOpt[i];
  }
}

void WalkingController::getCom_Z_mpc()
{  
  //MPC Param
  double mpc_hz_ = 10; double calc_hz_ = hz_/mpc_hz_;
  double mpc_dt_ = 1/mpc_hz_;
  Eigen::MatrixXd mpc_A_, mpc_B_;
  mpc_A_.resize(3,3); mpc_B_.resize(3,1);
  mpc_A_ << 1, mpc_dt_, mpc_dt_*mpc_dt_/2,
            0,       1,         mpc_dt_/1,
            0,       0,                 1;
  
  mpc_B_ << mpc_dt_*mpc_dt_*mpc_dt_/6,
                    mpc_dt_*mpc_dt_/2,
                            mpc_dt_/1;
          
  Eigen::MatrixXd Czp, Czv, Cza;
  Czp.resize(1,3); Czv.resize(1,3); Cza.resize(1,3);
  Czp << 1, 0, 0;  Czv << 0, 1, 0;  Cza << 0, 0, 1;

  //const int NL_z = 1.6*hz_;
  const int NL_z = 1.6*mpc_hz_;
  Eigen::MatrixXd Pps, Pvs, Pas;
  Pps.resize(NL_z,3); Pvs.resize(NL_z,3); Pas.resize(NL_z,3);

  Eigen::MatrixXd Pps_calc, Pvs_calc, Pas_calc;
  Pps_calc.resize(1,3); Pvs_calc.resize(1,3); Pas_calc.resize(1,3);
  Pps_calc = Czp*mpc_A_;    Pvs_calc = Czv*mpc_A_;    Pas_calc = Cza*mpc_A_;
  
  for(int i = 0; i < NL_z; i++)
  {
    Pps.block<1,3>(i,0) = Pps_calc;
    Pvs.block<1,3>(i,0) = Pvs_calc;
    Pas.block<1,3>(i,0) = Pas_calc;

    Pps_calc = Pps_calc*mpc_A_;
    Pvs_calc = Pvs_calc*mpc_A_;
    Pas_calc = Pas_calc*mpc_A_;
  }

  Eigen::MatrixXd Ppu, Pvu, Pau;
  Ppu.resize(NL_z,NL_z); Pvu.resize(NL_z,NL_z); Pau.resize(NL_z,NL_z);
  Ppu.setZero();
  Eigen::MatrixXd Pu_calc;
  Pu_calc.resize(3,NL_z); Pu_calc.setZero();
  for(int i = 0; i < NL_z; i++)
  {
    if(i == 0)
    {
      Pu_calc.block<3,1>(0,i) = mpc_B_;
    }
    else
    {
      Pu_calc.block<3,1>(0,i) = mpc_B_;
    }
    Ppu.block<1, 16>(i,0) = Czp*Pu_calc;
    Pvu.block<1, 16>(i,0) = Czv*Pu_calc;
    Pau.block<1, 16>(i,0) = Cza*Pu_calc;
    Pu_calc = mpc_A_*Pu_calc;
  }

  //MPC Control
  if(walking_tick_ == 0)
  {
    MPC_z_.setZero();
    MPC_z_(0) = com_support_init_(2);
  }
  else
  {
    //MPC_z_(0) = com_support_current_(2);
  }
  
  Eigen::MatrixXd z_ref, ref_z_calc_;
  double tick_start_ = walking_tick_ - (bool)current_step_num_*t_start_;
  ref_z_calc_.resize(320,1);
  ref_z_calc_.block<320,1>(0,0) = ref_com_z_.block<320,1>(tick_start_,0);

  z_ref.resize(NL_z, 1);
  
  for(int i = 0; i < NL_z; i++)
  {
    if(i == 0)
    {
      z_ref(i,0) = ref_z_calc_((i + 1) * calc_hz_ - 1,0);
    }
    else
    {
      z_ref(i,0) = ref_z_calc_((i + 1) * calc_hz_ - 1,0);
    }
  }

  Eigen::MatrixXd Qp, Qv, Qa, R;
  Qp.resize(NL_z, NL_z);  Qv.resize(NL_z, NL_z);  Qa.resize(NL_z, NL_z);  R.resize(NL_z, NL_z);
  Qp.setIdentity();       Qv.setIdentity();       Qa.setIdentity();       R.setIdentity();
  Qp = 1.0*Qp;            Qv = 1e-2*Qv;           Qa = 1e-5*Qa;          R = 1e-6*R;

  Eigen::MatrixXd Q_calc, g_calc;
  Q_calc.resize(NL_z, NL_z); Q_calc.setZero();
  Q_calc = Ppu.transpose()*Qp*Ppu + Pvu.transpose()*Qv*Pvu + Pau.transpose()*Qa*Pau + R;

  g_calc.resize(NL_z, 1); g_calc.setZero();
  g_calc = (Ppu.transpose()*Qp*Pps + Pvu.transpose()*Qv*Pvs + Pau.transpose()*Qa*Pas)*MPC_z_ - Ppu.transpose()*Qp*z_ref;

  Eigen::MatrixXd uOpt;
  uOpt.resize(NL_z,1);

  Eigen::MatrixXd A_ieq, b_ieq, Z_max, Z_min;
  A_ieq.resize(2*NL_z,NL_z); b_ieq.resize(2*NL_z,1); Z_max.resize(NL_z,1);    Z_min.resize(NL_z,1);
  A_ieq.setZero();           b_ieq.setZero();        Z_max.setConstant(0.78); Z_min.setConstant(0.35);
  A_ieq << Ppu, -Ppu;        b_ieq << Z_max - Pps*MPC_z_, Z_min - Pps*MPC_z_;

  //qpoases_solve(NL_z, 2*NL_z, Q_calc, g_calc, uOpt, A_ieq, b_ieq);
  qpoases_solve(NL_z, 2*NL_z, Q_calc, g_calc, uOpt);

  Eigen::Vector3d MPC_z_calc_, MPC_z_interpol_;
  MPC_z_calc_.setZero(); MPC_z_interpol_.setZero();
  MPC_z_calc_ = mpc_A_*MPC_z_ + mpc_B_*uOpt(0,0);

  MPC_z_prev_.resize(3, NL_z + 1); MPC_z_prev_.setZero();
  MPC_z_prev_.block<3,1>(0,0) = MPC_z_;
  
  for(int i = 1; i < NL_z + 1; i++)
  {
    if(i == 0)
    {
      MPC_z_prev_.block<3,1>(0,i) = mpc_A_*MPC_z_prev_.block<3,1>(0,i-1) + mpc_B_*uOpt(i - 1,0);
    }
    else
    {
      MPC_z_prev_.block<3,1>(0,i) = mpc_A_*MPC_z_prev_.block<3,1>(0,i-1) + mpc_B_*uOpt(i - 1,0);
    }
  }
  
  MPC_z_calc_ = (1/calc_hz_)*MPC_z_calc_ + (1 - 1/calc_hz_)*MPC_z_;
  MPC_z_ = MPC_z_calc_;

  com_desired_(2) = MPC_z_(0);

  if(tick_start_ == t_last_ - (bool)current_step_num_*t_start_ && current_step_num_ != total_step_num_ - 1)
  {
    MPC_z_(0) = MPC_z_(0) - foot_step_support_frame_(current_step_num_,2);
  }
  
  e_mpc_graphz << MPC_z_(0) << "," << MPC_z_(1) << "," << MPC_z_(2) << "," << z_ref(0,0) << "," << uOpt(0,0) << "," << ref_z_calc_(0,0) << endl;
}

void WalkingController::getCPTrajectory()
{
  if(walking_tick_ < t_temp_)
  {
    if(walking_tick_ == 0)
    {
      Td = 1.0;
    }

    cp_eos(0) = 0.0;
    cp_eos(1) = com_support_init_(1);

    if(walking_tick_ >= t_temp_ - 0.5*hz_)
    {
      cp_eos(1) = 0.0;
      if(walking_tick_ == t_temp_ - 0.5*hz_)
      {
        Td = 0.5;
      }
    }
  }

  if(walking_tick_ > t_temp_)
  {
    if(walking_tick_ == t_start_ + t_double1_ + t_rest_init_)
    {
      Td = t_total_/hz_;
    }

    if(current_step_num_ == total_step_num_ - 1)
    {
      cp_eos(0) = 0.0;
      cp_eos(1) = 0.5*foot_step_support_frame_(current_step_num_,1);
    }

    if(walking_tick_ >= t_start_ && walking_tick_ <= t_start_ + t_double1_ + t_rest_init_)
    {
      cp_eos(0) = 0.0;
      cp_eos(1) = 0.0;
    }
    else
    {
      cp_eos(0) = foot_step_support_frame_(current_step_num_,0);
      cp_eos(1) = foot_step_support_frame_(current_step_num_,1);
      
      if(current_step_num_ == total_step_num_ - 1)
      {
        cp_eos(1) = 0.5*foot_step_support_frame_(current_step_num_,1);
      }
    }
  }

  if(walking_tick_ == 0)
  {
    com.setZero();
    cp.setZero();

    com(0) = com_support_init_(0);
    com(1) = com_support_init_(1);

    cp(0) = com(0);
    cp(1) = com(1);
  }
  
  double w = sqrt(GRAVITY/zc_);
  double temp = 1/(1-exp(w*Td));
  double temp2 = (w/hz_)/(1 + w/hz_);

  if(walking_tick_ == t_start_ && current_step_num_ !=0)
  {
    cp(0) = cp(0) - 0.2;
    cp(1) = cp(1) + foot_step_support_frame_(current_step_num_,1);

    com(0) = com(0) - 0.2;
    com(1) = com(1) + foot_step_support_frame_(current_step_num_,1);
  }

  p = temp*cp_eos + (1 - temp)*cp;

  cp_next = p + exp(w/hz_)*(cp - p);

  com_next = temp2*cp_next + (1 - temp2)*com;

  e_cp_p_graph << Td << "," << com_next(0) << "," << p(0) << "," << cp_next(0) << "," << com_next(1) << "," << p(1) << "," << cp_next(1) << endl;

  com = com_next;

  cp = cp_next;
  cp_des_next = cp_next;

  Td = max(0.005, Td - 1/hz_);

  CPTracking();
  //com_desired_(0) = com(0);
  //com_desired_(1) = com(1);
}

void WalkingController::CPTracking()
{
  double w = sqrt(GRAVITY/zc_);

  double cp_w_dot = 0;
  double cp_w_dot_des = 0;
  double cp_lambda = (GRAVITY + com_support_current_ddot_LPF(2))/com_support_current_(2);
  double cp_lambda_des = (GRAVITY + MPC_z_(2))/MPC_z_(0);
  if(walking_tick_ == 0)
  {
    cp_w = sqrt(cp_lambda);
    cp_w_des = sqrt(cp_lambda_des);
  }
  //cp_w_dot = cp_w*cp_w - cp_lambda;
  //cp_w_dot_des = cp_w_des*cp_w_des - cp_lambda_des;

  e_tmp_graph << cp_w_des << "," << cp_w <<endl;
  //cp_w = cp_w + cp_w_dot/hz_;
  //cp_w_des = cp_w_des + cp_w_dot_des/hz_;

  cp_w = cp_w*cp_w/hz_ + cp_w - cp_lambda/hz_;
  cp_w_des = cp_w_des*cp_w_des/hz_ + cp_w_des - cp_lambda_des/hz_;


  double Td_tracking = 0.05;
  double temp = 1/(1-exp(w*Td_tracking));
  cp_measured_ = com_support_current_.segment<2>(0) + com_support_current_dot_LPF.segment<2>(0)/w;
  p_d = temp*cp_des_next + (1-temp)*cp_measured_;

  if(walking_tick_ == 0)
  {
    zmp_err_sum.setZero();
  }
  
  zmp_err_sum = zmp_err_sum + (p_d - zmp_measured_LPF)/hz_;
  e_cp_t_graph << cp_next(0) << "," << cp_measured_(0) << "," << p_d(0) << "," << cp_next(1) << "," << cp_measured_(1) << "," << p_d(1) << endl;
}

void WalkingController::getPelvTrajectory()
{
  double z_rot = foot_step_support_frame_(current_step_num_,5);
  
  pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0*(com_desired_(0) - com_support_current_(0));
  pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 1.0*(com_desired_(1) - com_support_current_(1));
  pelv_trajectory_support_.translation()(2) = pelv_support_current_.translation()(2) + 1.0*(com_desired_(2) - com_support_current_(2));
  //pelv_trajectory_support_.translation()(2) = com_desired_(2);
       
  Eigen::Vector3d Trunk_trajectory_euler;
  Trunk_trajectory_euler.setZero();

  if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_)
  { Trunk_trajectory_euler(2) = pelv_support_euler_init_(2); }
  else if(walking_tick_ >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_ < t_start_ + t_total_ - t_double2_ - t_rest_last_)
  { Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_double2_-t_rest_last_, pelv_support_euler_init_(2),z_rot/2.0,0.0,0.0); }
  else
  { Trunk_trajectory_euler(2) = z_rot/2.0; } 
  
  pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2))*DyrosMath::rotateWithY(Trunk_trajectory_euler(1))*DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void WalkingController::supportToFloatPattern()
{  
  pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*pelv_trajectory_support_;
  lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*lfoot_trajectory_support_;
  rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*rfoot_trajectory_support_;
}

void WalkingController::getFootTrajectory()
{
  kajita_alpha_zmp();
  
  Eigen::Vector6d target_swing_foot;

  //cout << foot_step_support_frame_(0,2) << "," << foot_step_support_frame_(1,2) << "," << foot_step_support_frame_(2,2) << "," << foot_step_support_frame_(3,2) << "," << foot_step_support_frame_(4,2) << "," << foot_step_support_frame_(5,2) << endl;

  for(int i=0; i<6; i++)
  { target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i); }

  if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_) 
  { 
    lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();  
    rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
    
    lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
    rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

    if(foot_step_(current_step_num_,6) == 1)//left foot support
    { 
      //lfoot_trajectory_support_.translation().setZero();
      lfoot_trajectory_support_.translation()(0) = 0.0;
      lfoot_trajectory_support_.translation()(1) = 0.0;
      lfoot_trajectory_euler_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      //rfoot_trajectory_support_.translation()(2) = 0;
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
    }     
    else if(foot_step_(current_step_num_,6) == 0)//right foot support
    { 
      //rfoot_trajectory_support_.translation().setZero();
      rfoot_trajectory_support_.translation()(0) = 0.0;
      rfoot_trajectory_support_.translation()(1) = 0.0;
      rfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      //lfoot_trajectory_support_.translation()(2) = 0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_; 
    }     

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
  }
  
  else if(walking_tick_ >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_ < t_start_ + t_total_ - t_double2_ - t_rest_last_)  
  {   
    if(foot_step_(current_step_num_,6) == 1)//left foot support
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();             
      lfoot_trajectory_euler_support_.setZero(); 

      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      
      if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0) 
      { 
        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_+ t_rest_init_ + t_double1_,t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,rfoot_support_init_.translation()(2) ,rfoot_support_init_.translation()(2) +  foot_height_,0.0,0.0);
      }  
      else
      { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,rfoot_support_init_.translation()(2) + foot_height_,target_swing_foot(2),0.0,0.0); }
      
      for(int i=0; i<2; i++) // X, Y 방향 궤적 생성, 여기서 불연속 문제를 회복해야함. 스윙발이 완벽히 따라가지 못하고 지지발이 되었을때, 현재 지지발 기준으로 봤을때 스윙발이 오차가 생긴것이기 때문에 3차곡선으로 이어줘야함. 
      { rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_ + t_double1_ , t_start_+t_total_-t_rest_last_-t_double2_, rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); } 
      
      rfoot_trajectory_euler_support_(0) = 0;
      rfoot_trajectory_euler_support_(1) = 0;
      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_ + t_rest_init_ + t_double1_,t_start_ + t_total_ - t_rest_last_ - t_double2_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if(foot_step_(current_step_num_,6) == 0)//right foot support
    { 
      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation(); 
      rfoot_trajectory_euler_support_.setZero(); 

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
 
      if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
      { 
        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,lfoot_support_init_.translation()(2) ,lfoot_support_init_.translation()(2) +  foot_height_,0.0,0.0);
      }
      else
      { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(2) + foot_height_,target_swing_foot(2),0.0,0.0); }
         
      for(int i=0; i<2; i++)
      { lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); }
      
      //econom2
      //lfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(1),target_swing_foot(1) + foot_width_tmp_,0.0,0.0);

      lfoot_trajectory_euler_support_(0) = 0;
      lfoot_trajectory_euler_support_(1) = 0;  
      lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    } 
  }
  else // 210 ~ 239 tick , 0.15초 
  { 
    if(foot_step_(current_step_num_,6) == 1)//left foot support
    {
      //lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_euler_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      
      for(int i=0; i<3; i++)
      {
        rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if (foot_step_(current_step_num_,6) == 0)//right foot support
    {
      //rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_euler_support_.setZero();
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

      for(int i=0; i<3; i++)
      {
        lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      
      lfoot_trajectory_support_.translation()(1) = target_swing_foot(1);

      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    }
  }
}

void WalkingController::computeIK_e(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des)
{  
  double offset_hip_pitch = 24.0799945102*DEG2RAD;
  double offset_knee_pitch = 14.8197729791*DEG2RAD;
  double offset_ankle_pitch = 9.2602215311*DEG2RAD;

  Eigen::Vector3d R_r, R_D, L_r, L_D ;

  L_D << 0 , +0.105, -0.1119;
  R_D << 0 , -0.105, -0.1119;
  
  L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*L_D - float_lleg_transform.translation());
  R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*R_D - float_rleg_transform.translation());
  
  double R_C = 0, L_C = 0, L_upper = 0.3713, L_lower = 0.3728 , R_alpha = 0, L_alpha = 0;

  L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
  R_C = sqrt( pow(R_r(0),2) + pow(R_r(1),2) + pow(R_r(2),2) );
  
  q_des(3) = (-acos((pow(L_upper,2) + pow(L_lower,2) - pow(L_C,2)) / (2*L_upper*L_lower))+ M_PI) ;
  q_des(9) = (-acos((pow(L_upper,2) + pow(L_lower,2) - pow(R_C,2)) / (2*L_upper*L_lower))+ M_PI) ;
  L_alpha = asin(L_upper / L_C * sin(M_PI - q_des(3)));
  R_alpha = asin(L_upper / R_C * sin(M_PI - q_des(9)));

  q_des(4)  = -atan2(L_r(0), sqrt(pow(L_r(1),2) + pow(L_r(2),2))) - L_alpha ;
  q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) - R_alpha ;
  
  Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
  Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
  Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

  L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3)-q_des(4));
  L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
  R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9)-q_des(10));
  R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11)); 
  
  L_Hip_rot_mat.setZero(); R_Hip_rot_mat.setZero();

  L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat; 
  R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

  q_des(0) = -atan2(-L_Hip_rot_mat(0,1),L_Hip_rot_mat(1,1)); // Hip yaw
  q_des(1) =  atan2(L_Hip_rot_mat(2,1), -L_Hip_rot_mat(0,1) * sin(q_des(0)) + L_Hip_rot_mat(1,1)*cos(q_des(0))); // Hip roll
  q_des(2) =  atan2(-L_Hip_rot_mat(2,0), L_Hip_rot_mat(2,2)) + offset_hip_pitch; // Hip pitch
  q_des(3) =  q_des(3) - 14.8197729791*DEG2RAD; // Knee pitch
  q_des(4) =  q_des(4) - 9.2602215311*DEG2RAD; // Ankle pitch
  q_des(5) =  atan2( L_r(1), L_r(2) ); // Ankle roll

  q_des(6) = -atan2(-R_Hip_rot_mat(0,1),R_Hip_rot_mat(1,1));
  q_des(7) =  atan2(R_Hip_rot_mat(2,1), -R_Hip_rot_mat(0,1) * sin(q_des(6)) + R_Hip_rot_mat(1,1)*cos(q_des(6)));
  q_des(8) = -atan2(-R_Hip_rot_mat(2,0), R_Hip_rot_mat(2,2)) - offset_hip_pitch;
  q_des(9) = -q_des(9) + 14.8197729791*DEG2RAD;
  q_des(10) = -q_des(10) + 9.2602215311*DEG2RAD; 
  q_des(11) =  atan2( R_r(1), R_r(2) );
  
}

void WalkingController::updateNextStepTime()
{
  if(walking_tick_ == t_last_)  
  { 
    if(current_step_num_ != total_step_num_-1)
    { 
      t_start_ = t_last_ + 1 ;  
      t_start_real_ = t_start_ + t_rest_init_;  
      t_last_ = t_start_ + t_total_ -1;    
      current_step_num_ ++;
    }
  }
  
  if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_last_ + t_total_) 
  {
    walking_enable_ = false;
  }
  walking_tick_ ++;
}

void WalkingController::slowCalc()
{
  while(true)
  {
    if(ready_for_thread_flag_)
    {
       
      if (ready_for_compute_flag_ == false)
      {
        ready_for_compute_flag_ = true;
      }
    }
    this_thread::sleep_for(chrono::milliseconds(100));
  }
}

void WalkingController::Compliant_control(Eigen::Vector12d desired_leg_q)
{
  if(walking_tick_ == 0)  
  {
    pre_motor_q_leg_ = current_motor_q_leg_; 
    DOB_IK_output_b_ = current_motor_q_leg_;
  }
      
  Eigen::Vector12d current_u; // left right order
  double del_t = 0, Kp = 0;
  del_t = 1/hz_; Kp = 30;

  for (int i = 0; i < 12; i++)
  { // (theta_m(k+1) - theta_m(k)) / dt = Kp (u - theta_m(k)) 
    current_u(i) = (current_motor_q_leg_(i) - (1 - Kp*del_t)*pre_motor_q_leg_(i)) / (Kp*del_t);
  }
  
  Eigen::Vector12d d_hat;    
  d_hat = current_u - DOB_IK_output_b_ ; //current_u -> u' + d , DOB_IK_output_b_ -> u'= IK output + d hat
  
  if(walking_tick_ == 0)
    d_hat_b = d_hat;

  d_hat = 0.7*d_hat_b + 0.3*d_hat; // 필터링

  // Mingon's LQR contorller gain (using external encoder)
  double default_gain = 0.0; // Kp가 정확하다면 시뮬레이션이나 실제 로봇이나 0.2~1의 의미는 같다.
  double compliant_gain = 1.0;
  double compliant_tick = 0.1*hz_;
  double gain_temp;
  for (int i = 0; i < 12; i ++)
  {
    if(i < 6) //왼쪽 다리 관절
    { 
      gain_temp = default_gain;
      if(walking_enable_ == true)
      {
        if (foot_step_(current_step_num_,6) == 0) // 오른발 지지 상태
        { 
          if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick) // gain_temp -> 0.2
          { // t_total_: 240tick, t_rest_init_,last: 10tick (0.05초), t_double1,2_: 20tick (0.1초), compliant_tick : 20tick (0.1초)
            gain_temp = default_gain; // 1step 240tick 동안 0~190 tick까지 계속 기본 gain.
          }
          else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_)
          { 
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_, default_gain, compliant_gain, 0.0, 0.0);
          } // 1step 240tick 동안 190~210 tick까지 기본 gain 0.2에서 1까지 올림.
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
          } // 1step 240tick 동안 210~240 tick까지 기본 gain 1에서 0.2까지 낮춤.
        } 
        else // 왼발 지지 상태
        {
          gain_temp = default_gain;
        }
      }       
      DOB_IK_output_(i) = desired_leg_q(i) + gain_temp*d_hat(i);  // LQR 위치 대신 단순 IK 기반 위치 ( u_c + d_hat = u' (논문))
    }
    else //오른 다리 관절
    {
      //double gain_temp = default_gain;
      gain_temp = default_gain;

      if(walking_enable_ == true)
      {
        if (foot_step_(current_step_num_,6) == 1) // 왼발 지지 상태
        {
          if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick) // gain_temp -> 0.2
          {
            gain_temp = default_gain;
          }
          else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_, default_gain, compliant_gain, 0.0, 0.0);
          }
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
          }
        }
        else // 오른발 지지 상태
        {
          gain_temp = default_gain;
        }
      }
      //lqr_output_(i) = lqr_output_pre_(i) + del_u_right(i, 0) - gain_temp*d_hat(i);
      DOB_IK_output_(i) = desired_leg_q(i) + gain_temp*d_hat(i);  // LQR 위치 대신 단순 IK 기반 위치
    }
  }  
  pre_motor_q_leg_ = current_motor_q_leg_; 
  d_hat_b = d_hat;
  //cout << d_hat*180/M_PI << endl;
  DOB_IK_output_b_ = DOB_IK_output_; 
}

void WalkingController::preview_Parameter_CPM(double dt, int NL, Eigen::Matrix3d& K, Eigen::Vector3d com_support_init_, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, 
  Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::MatrixXd& A_bar, Eigen::VectorXd& B_bar)
  {      
    double Kp = 100;
    double Kv = 10;
    
    A.resize(2,2);
    A(0,0) = 1.0 - Kp*dt*dt*0.5;
    A(0,1) = dt - 0.5*Kv*dt*dt;
    A(1,0) = -Kp*dt;
    A(1,1) = 1.0 - Kv*dt;
    
    B.resize(2);
    B(0) = 0.5*Kp*dt*dt;
    B(1) = Kp*dt;
    
    C.resize(1,2);
    C(0,0) = 1 + zc_/GRAVITY*Kp;
    C(0,1) = zc_/GRAVITY*Kv; 
    D.resize(1,1);
    D(0,0) = -zc_/GRAVITY*Kp; 

    B_bar.resize(3,1);
    B_bar(0,0) = D(0,0);
    B_bar(1,0) = B(0);
    B_bar(2,0) = B(1);
    
    Eigen::Matrix1x3d B_bar_tran;
    B_bar_tran = B_bar.transpose();
    
    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.resize(3,3);
    I_bar.resize(3,1);
    F_bar.resize(3,2);
    F_bar.setZero();

    F_bar(0,0) = C(0,0);
    F_bar(0,1) = C(0,1);

    F_bar(1,0) = A(0,0);
    F_bar(1,1) = A(0,1);
    F_bar(2,0) = A(1,0);
    F_bar(2,1) = A(1,1);
    
    I_bar.setZero();
    I_bar(0,0) = 1.0;

    A_bar(0,0) = I_bar(0,0);
    A_bar(1,0) = I_bar(1,0);
    A_bar(2,0) = I_bar(2,0);

    A_bar(0,1) = F_bar(0,0);
    A_bar(0,2) = F_bar(0,1);
    A_bar(1,1) = F_bar(1,0);
    A_bar(1,2) = F_bar(1,1);
    A_bar(2,1) = F_bar(2,0);
    A_bar(2,2) = F_bar(2,1);
   
    Eigen::MatrixXd Qe;
    Qe.resize(1,1);
    Qe(0,0) = 1.0;

    Eigen::MatrixXd R;
    R.resize(1,1);
    R(0,0) = 1.0;

    Eigen::MatrixXd Qx;
    Qx.resize(3,3);
    Qx.setZero();

    Eigen::MatrixXd Q_bar;
    Q_bar.resize(3,3);
    Q_bar.setZero();
    Q_bar(0,0) = Qe(0,0);
        
    K(0,0) = 110.075528194525;
    K(0,1) = 6002.773189475650;
    K(0,2) = 1620.941388698153;
    K(1,1) = 330547.378525671258;
    K(1,2) = 89255.846463209440;
    K(2,2) = 24102.882783488018;
    K(1, 0) = K(0, 1);
    K(2, 0) = K(0, 2);
    K(2, 1) = K(1, 2);

    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.resize(1,1);
    Temp_mat.setZero();
    Temp_mat_inv.resize(1,1);
    Temp_mat_inv.setZero();
    Ac_bar.setZero();
    Ac_bar.resize(3,3);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse(); 
    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;
    
    Eigen::MatrixXd Ac_bar_tran(3,3);
    Ac_bar_tran = Ac_bar.transpose();
 
    Gi.resize(1,1); Gx.resize(1,2);
    Gi = Temp_mat_inv * B_bar_tran * K * I_bar ;
    Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;   
    
    Eigen::MatrixXd X_bar;
    Eigen::Vector3d X_bar_col;
    X_bar.resize(3, NL); 
    X_bar.setZero();
    X_bar_col.setZero();
    X_bar_col = - Ac_bar_tran * K * I_bar;

    for(int i = 0; i < NL; i++)
    {
      X_bar.block<3,1>(0,i) = X_bar_col;
      X_bar_col = Ac_bar_tran*X_bar_col;
    }           

    Gd.resize(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0,0);
    
    for(int i = 0; i < NL; i++)
    {
      Gd.segment(i,1) = Gd_col;
      Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i) ;
    }     
}

void WalkingController::previewcontroller_CPM(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, 
       Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd A_bar, Eigen::VectorXd B_bar, Eigen::Vector2d &XD, Eigen::Vector2d &YD, Eigen::VectorXd& X_bar_p, Eigen::VectorXd& Y_bar_p)
{
  int zmp_size;
  zmp_size = ref_zmp_.col(1).size();  
  Eigen::VectorXd px_ref, py_ref;
  px_ref.resize(zmp_size);
  py_ref.resize(zmp_size);
  
  for(int i = 0; i < zmp_size; i++)
  {
    px_ref(i) = ref_zmp_(i,0);
    py_ref(i) = ref_zmp_(i,1);
  }
  

  Eigen::Matrix1x3d C;
  C(0,0) = 1; C(0,1) = 0; C(0,2) = -zc_/GRAVITY;
  
  Eigen::VectorXd px, py;
  px.resize(1); py.resize(1);
  
  X_bar_p.resize(3); Y_bar_p.resize(3);

  if(tick == 0 && current_step_num_ == 0)
  { 
    Preview_X_b(0) = x_i;  
    Preview_Y_b(0) = y_i;
    Preview_X(0) = x_i;
    Preview_Y(0) = y_i;
    Preview_X(1) = 0;
    Preview_Y(1) = 0;
    X_bar_p.setZero(); Y_bar_p.setZero();
  }
  else
  {       
    Preview_X(0) = xs(0); Preview_Y(0) = ys(0);
    Preview_X(1) = X_bar_p(1)/dt; Preview_Y(1) = Y_bar_p(1)/dt;     
  }  
  
  Eigen::VectorXd Temp_mat_X, Temp_mat_Y;
  Temp_mat_X.resize(3); Temp_mat_Y.resize(3);
  Temp_mat_X.setZero(); Temp_mat_Y.setZero();  
    
  Temp_mat_X(0) = Preview_X(0); Temp_mat_Y(0) = Preview_Y(0); 
  Temp_mat_X(2) = X_bar_p(2)/dt; Temp_mat_Y(2) = Y_bar_p(2)/dt;  
  
  px = C*Temp_mat_X;
  py = C*Temp_mat_Y; 

  X_bar_p(0) = px(0) - px_ref(tick); 
  Y_bar_p(0) = py(0) - py_ref(tick);
   
  double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

  for(int i = 0; i < NL; i++) 
  {
    sum_Gd_px_ref = sum_Gd_px_ref + Gd(i)*(px_ref(tick + 1 + i) - px_ref(tick + i));
    sum_Gd_py_ref = sum_Gd_py_ref + Gd(i)*(py_ref(tick + 1 + i) - py_ref(tick + i));
  }

  Eigen::MatrixXd temp; temp.resize(2, 1);
  Eigen::VectorXd GxX(1); Eigen::VectorXd GxY(1);

  temp(0, 0) = X_bar_p(1);  
  temp(1, 0) = X_bar_p(2);    
  GxX = Gx*temp;

  temp(0, 0) = Y_bar_p(1); 
  temp(1, 0) = Y_bar_p(2); 
  GxY = Gx*temp;

  Eigen::MatrixXd del_ux(1,1);
  Eigen::MatrixXd del_uy(1,1);
  del_ux.setZero();
  del_uy.setZero();
 
  del_ux(0,0) = -(X_bar_p(0) * Gi(0,0)) - GxX(0) - sum_Gd_px_ref;
  del_uy(0,0) = -(Y_bar_p(0) * Gi(0,0)) - GxY(0) - sum_Gd_py_ref;
  
  X_bar_p = A_bar*X_bar_p + B_bar*del_ux;
  Y_bar_p = A_bar*Y_bar_p + B_bar*del_uy;  

  UX = UX + del_ux(0,0);
  UY = UY + del_uy(0,0);    
  
  XD = A*Preview_X + B*UX;
  YD = A*Preview_Y + B*UY;
         
}

void WalkingController::getCom_preview_control_CPM()
{
  if(walking_tick_ == 0)  
  { 
    preview_Parameter_CPM(1.0/hz_, 16*hz_/10, K_ ,com_support_init_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_); 
    UX_ = com_support_init_(0);
    UY_ = com_support_init_(1);
    xs_(0) = xi_; xs_(1) = 0; xs_(2) = 0;
    ys_(0) = yi_; ys_(1) = 0; xs_(2) = 0;
  }

  if(current_step_num_ == 0)
  { zmp_start_time_ = 0.0; }
  else
  { zmp_start_time_ = t_start_; }
          
  previewcontroller_CPM(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, A_bar_, B_bar_, XD_, YD_, X_bar_p_, Y_bar_p_);
  
  xs_(0) = com_support_current_(0); ys_(0) = com_support_current_(1); 
  xs_(1) = XD_(1); ys_(1) = YD_(1); 

  com_desired_(0) = UX_;
  com_desired_(1) = UY_;
  com_desired_(2) = pelv_support_start_.translation()(2);

  if (walking_tick_ == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1)  
  { 
    Eigen::Vector3d com_pos_prev;
    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel_prev;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d com_acc_prev;
    Eigen::Vector3d com_acc;
    //
    Eigen::Vector3d com_u_prev;
    Eigen::Vector3d com_u;
    // 
    Eigen::Matrix3d temp_rot;
    Eigen::Vector3d temp_pos;
    
    temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_,5)); 
    for(int i=0; i<3; i++)
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);     
    
    com_pos_prev(0) = xs_(0);
    com_pos_prev(1) = ys_(0);
    com_pos = temp_rot*(com_pos_prev - temp_pos);
     
    com_vel_prev(0) = xs_(1);
    com_vel_prev(1) = ys_(1);
    com_vel_prev(2) = 0.0;
    com_vel = temp_rot*com_vel_prev;

    com_acc_prev(0) = xs_(2);
    com_acc_prev(1) = ys_(2);
    com_acc_prev(2) = 0.0;
    com_acc = temp_rot*com_acc_prev;

    com_u_prev(0) = UX_;
    com_u_prev(1) = UY_;
    com_u = temp_rot*(com_u_prev - temp_pos);

    xs_(0) = com_pos(0);
    ys_(0) = com_pos(1);
    xs_(1) = com_vel(0);
    ys_(1) = com_vel(1);
    xs_(2) = com_acc(0);
    ys_(2) = com_acc(1); 

    UX_ = com_u(0);
    UY_ = com_u(1);  
  } 
}

}