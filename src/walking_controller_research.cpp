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
  ofstream e_tmp_graph("/home/econom2/data/e_tmp_graph.txt");
  ofstream e_tmp_2_graph("/home/econom2/data/e_tmp_2_graph.txt");

  ofstream e_any_graph("/home/econom2/data/e_any_graph.txt");
  ofstream e_any_graph1("/home/econom2/data/e_any_graph1.txt");

  ofstream e_zmp_x_graph("/home/econom2/data/e_zmp_x_graph.txt");
  ofstream e_zmp_y_graph("/home/econom2/data/e_zmp_y_graph.txt");
  
  ofstream e_com_x_graph("/home/econom2/data/e_com_x_graph.txt");
  ofstream e_com_y_graph("/home/econom2/data/e_com_y_graph.txt");
  ofstream e_com_z_graph("/home/econom2/data/e_com_z_graph.txt");
  
  ofstream e_dis_1_graph("/home/econom2/data/e_dis_1_graph.txt");
  ofstream e_dis_2_graph("/home/econom2/data/e_dis_2_graph.txt");
  ofstream e_dis_tmp_graph("/home/econom2/data/e_dis_tmp_graph.txt");

  ofstream e_lfoot_z_graph("/home/econom2/data/e_lfoot_z_graph.txt");

  ofstream e_cp_graph1("/home/econom2/data/e_cp_graph1.txt");
  ofstream e_cp_graph2("/home/econom2/data/e_cp_graph2.txt");

  ofstream e_jump_graph("/home/econom2/data/e_jump_graph.txt");

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
        if(Com_DOB_DSP_mode == 0)
        {
          getComTrajectory();
        }
        else if(Com_DOB_DSP_mode == 1)
        {
          getComTrajectory_ori();
          //Com_DoB1_COM();
          Com_DoB2_ESO();
        }
        else if(Com_DOB_DSP_mode == 2)
        {
          getComTrajectory_ori();
          //Com_DoB1_COM();
          e_jump_trajectory();
          e_jump_zmp_control();
        }
        
        getFootTrajectory();
        getPelvTrajectory();

        supportToFloatPattern();
        computeIK_e(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des);

        Eigen::Vector12d d_q;  
        for(int i=0; i<12; i++)
        { desired_q_(i) = q_des(i); } 
        desired_q_not_compensated_ = desired_q_ ;  
        
        if(hip_compensator_mode_ == true)
        { 
          hip_compensator(); 
          for(int i = 0; i < 12; i++)
          { d_q(i) = desired_q_(i); }
          Compliant_control(d_q); 
        }

        if(Com_DOB_DSP_mode == 2 || Com_DOB_DSP_mode == 1)
        {
          for(int i = 0; i < 12; i++)
          { d_q(i) = desired_q_(i); }
          Com_DoB_js(d_q);
        }

        updateNextStepTime();
        //com dob dsp
        cout << current_step_num_ << "," << walking_tick_ << "," << tick_index << endl;
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
        if(walking_tick_ == 0)
        { desired_q(i) = desired_q_(i); }
        else
        {
          desired_q(i) = DOB_IK_output_b_(i);
          //desired_q(i) = desired_q_(i);
        }
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

  foot_height_ = 0.0; 
  Com_DOB_DSP_mode = 1;
  //0 normal walking mode with preview control
  //1 com dob mode
  //2 jump
  zmp_offset_ = 0.0;
  foot_step_tmp_ = 6;
  foot_width_tmp_ = 0.0;

  gyro_frame_flag_ = false;
  com_control_mode_ = true;
  estimator_flag_ = false; 
}

void WalkingController::getRobotState()
{
  Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
  q_temp.setZero();
  qdot_temp;  
  q_temp.segment<28>(6) = current_q_.segment<28>(0);   
  qdot_temp.segment<28>(6)= current_qdot_.segment<28>(0);
 
  model_.updateKinematics(q_temp, qdot_temp);
  com_float_current_ = model_.getCurrentCom();
  com_float_current_dot_= model_.getCurrentComDot();
  lfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)0); 
  rfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)1);
    
  ////////////////////////////////////////
  q_sim_virtual_ = model_.getMujocoCom();
  double w = sqrt(1-((q_sim_virtual_(3))*(q_sim_virtual_(3)))-((q_sim_virtual_(4))*(q_sim_virtual_(4)))-((q_sim_virtual_(5))*(q_sim_virtual_(5))));
  tf::Quaternion q_mjc(q_sim_virtual_(3),q_sim_virtual_(4),q_sim_virtual_(5),w);

  Eigen::Quaterniond q;
  if(walking_tick_ == 0)
  { q_b_ = q; }

  q.x() = q_sim_virtual_(3);
  q.y() = q_sim_virtual_(4);
  q.z() = q_sim_virtual_(5);
  q.w() = w;

  q.x() = 0.9*q_b_.x() + 0.1*q.x();
  q.y() = 0.9*q_b_.y() + 0.1*q.y();
  q.z() = 0.9*q_b_.z() + 0.1*q.z();
  q.w() = 0.9*q_b_.w() + 0.1*q.w();
  q_b_ = q;

  R = q.normalized().toRotationMatrix();   
  R_.setIdentity(); 
  R_(0,0) = R(0,0); R_(0,1) = R(0,1); R_(0,2) = R(0,2);
  R_(1,0) = R(1,0); R_(1,1) = R(1,1); R_(1,2) = R(1,2);
  R_(2,0) = R(2,0); R_(2,1) = R(2,1); R_(2,2) = R(2,2);

  double r_mjc,p_mjc,y_mjc;
  tf::Matrix3x3 m(q_mjc);
  m.getRPY(r_mjc,p_mjc,y_mjc);  
  R_angle = r_mjc; P_angle = p_mjc;
  double del_t = 1/hz_;
  R_angle_i = R_angle_i + R_angle * del_t;
  P_angle_i = P_angle_i + P_angle * del_t;
  
  com_global_current_ = R*com_float_current_;
  ////////////////////////////////////////////

  if(foot_step_(current_step_num_, 6) == 0) 
  { supportfoot_float_current_ = rfoot_float_current_;}
  else if(foot_step_(current_step_num_, 6) == 1)
  { supportfoot_float_current_ = lfoot_float_current_; }

  if(current_step_num_ != 0 && walking_tick_ == t_start_) // step change
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
  com_support_current_bb = com_support_current_b;
  com_support_current_b = com_support_current_;
  com_support_current_ =  DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);
  com_support_current_imu_b_ = com_support_current_imu;
  com_support_current_imu = R_.rotation()*com_support_current_;

  if(foot_step_(current_step_num_,6) == 0)//right foot support
  {
    com_swing_current_ = com_support_current_ - lfoot_support_current_.translation();
    com_trajectory_swing_ = com_desired_ - lfoot_trajectory_support_.translation();
  }
  else if(foot_step_(current_step_num_,6) == 1)
  {
    com_swing_current_ = com_support_current_ - rfoot_support_current_.translation();
    com_trajectory_swing_ = com_desired_ - rfoot_trajectory_support_.translation();
  }

  if(walking_tick_ != t_start_)
  {
    com_support_current_dot = (com_support_current_ - com_support_current_b)*hz_;
    com_support_current_ddot = (com_support_current_ - 2*com_support_current_b + com_support_current_bb)*hz_*hz_;
  }   

  if(walking_tick_ == 0)
  {
    com_support_current_dot.setZero();
    com_support_current_dot_LPF = com_support_current_dot;

    com_support_current_ddot.setZero();
    com_support_current_ddot_LPF = com_support_current_ddot;
  }

  double com_dot_lpf_fre = 5;
  double com_dot_lpf_w = 2*M_PI*com_dot_lpf_fre;

  com_support_current_dot_LPF_b_ = com_support_current_dot_LPF;
  com_support_current_dot_LPF = (com_dot_lpf_w*del_t*com_support_current_dot + com_support_current_dot_LPF)/(1+com_dot_lpf_w*del_t);
  com_support_current_ddot_LPF_b_ = com_support_current_ddot_LPF;
  com_support_current_ddot_LPF = (com_dot_lpf_w*del_t*com_support_current_ddot + com_support_current_ddot_LPF)/(1+com_dot_lpf_w*del_t);

  com_support_current_dot_imu_b_ = com_support_current_dot_imu;
  com_support_current_dot_imu = R_*com_support_current_dot_LPF;

  com_support_current_ddot_imu_b_ = com_support_current_ddot_imu;
  com_support_current_ddot_imu = R_*com_support_current_ddot_LPF;

  r_ft_ = model_.getRightFootForce();
  l_ft_ = model_.getLeftFootForce();
  collide_ft_b = collide_ft_;
  collide_ft_ = model_.getCollideForce();
  collide_ft_ = pelv_support_current_.linear().transpose()*collide_ft_;
  collide_ft_ = 0.7*collide_ft_b + 0.3*collide_ft_;
  grav_vec_ = model_.getImuAngvel();

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
  unsigned int planning_step_number = 3;
  unsigned int norm_size = 0;

  unsigned int com_dob_dsp_temp = 0;
  
  if(current_step_num_ >= total_step_num_ - planning_step_number)
    norm_size = (t_last_ - t_start_ + 1)*(total_step_num_ - current_step_num_) + 20*hz_;
  else
    norm_size = (t_last_ - t_start_ + 1)*(planning_step_number); 
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_ + 1;
    
  zmpGenerator(norm_size, planning_step_number);
  zmpCalc();
  
  Eigen::Vector3d tmp;
  tmp << zmp_measured_LPF, -0.5*rfoot_support_current_.translation()(1)/R_.rotation()(0,0);
  zmp_measured_imu = (R_.rotation()*tmp).head(2);

  if(current_step_num_ == 0)
  {
    zmp_desired_(0) = ref_zmp_(walking_tick_,0);
    zmp_desired_(1) = ref_zmp_(walking_tick_,1);
    
    e_zmp_x_graph << ref_zmp_(walking_tick_,0) << "," << zmp_measured_LPF(0) << endl;
    e_zmp_y_graph << ref_zmp_(walking_tick_,1) << "," << zmp_measured_LPF(1) << "," << zmp_measured_imu(1) << endl;
    e_any_graph1 << r_ft_(2) << "," << l_ft_(2) << endl;
  }
  else
  {
    zmp_desired_(0) = ref_zmp_(walking_tick_ - t_start_,0);
    zmp_desired_(1) = ref_zmp_(walking_tick_ - t_start_,1);

    e_zmp_x_graph << ref_zmp_(walking_tick_ - t_start_,0) << "," << zmp_measured_LPF(0) << endl;
    e_zmp_y_graph << ref_zmp_(walking_tick_ - t_start_,1) << "," << zmp_measured_LPF(1) << "," << zmp_measured_imu(1) << endl;
    e_any_graph1 << r_ft_(2) << "," << l_ft_(2) << endl;
  }
}

void WalkingController::zmpCalc()
{
  //0.0062
  zmp_r_(0) = (-r_ft_(4) - r_ft_(0)*0.096) / r_ft_(2);
  zmp_r_(1) = (r_ft_(3) - r_ft_(1)*0.096) / r_ft_(2);
  zmp_l_(0) = (-l_ft_(4) - l_ft_(0)*0.096) / l_ft_(2);
  zmp_l_(1) = (l_ft_(3) - l_ft_(1)*0.096) / l_ft_(2);

  zmp_measured_(0) = ((((lfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_l_)(0) + (lfoot_support_current_.translation())(0))*l_ft_(2) + zmp_r_(0)*r_ft_(2)) / (r_ft_(2) + l_ft_(2));
  zmp_measured_(1) = ((((lfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_l_)(1) + (lfoot_support_current_.translation())(1))*l_ft_(2) + zmp_r_(1)*r_ft_(2)) / (r_ft_(2) + l_ft_(2));
  zmp_measured_(1) = 1.0*zmp_measured_(1) + com_support_init_(1);

  if(walking_tick_ == 0)
  {
    zmp_measured_LPF = zmp_measured_;
    zmp_measured_LPF_b = zmp_measured_LPF;

    l_ft_LPF_ = l_ft_;
    r_ft_LPF_ = r_ft_;

    l_ft_LPF_b_ = l_ft_LPF_;
    r_ft_LPF_b_ = r_ft_LPF_;
  }

  zmp_measured_LPF_b = zmp_measured_LPF;
  l_ft_LPF_b_ = l_ft_LPF_;
  r_ft_LPF_b_ = r_ft_LPF_;

  //l_ft_(2) = l_ft_(2) - 8.5;
  //r_ft_(2) = r_ft_(2) + 8.5;

  zmp_measured_LPF = 0.9*zmp_measured_LPF_b + 0.1*zmp_measured_;
  zmp_measured_LPF(1) = zmp_measured_LPF(1) + 0.0004;
  l_ft_LPF_ = 0.9*l_ft_LPF_b_ + 0.1*l_ft_;
  r_ft_LPF_ = 0.9*r_ft_LPF_b_ + 0.1*r_ft_;

  zmp_e_calc_(0) = -0.92*(l_ft_LPF_(4) + r_ft_LPF_(4) + ( l_ft_LPF_(0) + r_ft_LPF_(0))*0.96)/(l_ft_LPF_(2) + r_ft_LPF_(2)) + 0.007;
  zmp_e_calc_(1) =  0.92*(l_ft_LPF_(3) + r_ft_LPF_(3) + (-l_ft_LPF_(2) + r_ft_LPF_(2))*com_support_init_(1))/(l_ft_LPF_(2) + r_ft_LPF_(2)) + com_support_init_(1);
  
  if(l_ft_(2) > 0)
  { zmp_e_calc_(1) = (r_ft_LPF_(3) + r_ft_LPF_(2)*com_support_init_(1))/r_ft_LPF_(2) + com_support_init_(1); }
  if(r_ft_(2) > 0)
  { zmp_e_calc_(1) = (l_ft_LPF_(3) - l_ft_LPF_(2)*com_support_init_(1))/l_ft_LPF_(2) + com_support_init_(1); }

  e_tmp_graph << zmp_e_calc_(0) << "," << zmp_e_calc_(1) << endl;
}

void WalkingController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{ 
  ref_zmp_.resize(norm_size, 2); 
  Eigen::VectorXd temp_px;
  Eigen::VectorXd temp_py;
  
  unsigned int index = 0; 

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
        double del_x = i - 0.5*hz_;
        ref_zmp_(i,0) = com_support_init_(0) - del_x * com_support_init_(0)/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else 
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }      
      index++;
    }    
  }
  if(current_step_num_ >= total_step_num_ - planning_step_num)
  {  
    for(unsigned int i = current_step_num_; i < total_step_num_; i++)
    {
      if(Com_DOB_DSP_mode == 1)
        ComDOB_onestepZmp(i,temp_px,temp_py);
      else
        onestepZmp(i, temp_px, temp_py);
     
      for(unsigned int j = 0; j < t_total_; j++)
      {
        ref_zmp_(index + j, 0) = temp_px(j);
        ref_zmp_(index + j, 1) = temp_py(j);    
      }
      index = index + t_total_;
    }
    
    for(unsigned int j = 0; j < 20*hz_; j++)
    {
      ref_zmp_(index + j, 0) = ref_zmp_(index -1, 0);
      ref_zmp_(index + j, 1) = ref_zmp_(index -1, 1);
    }
    index = index + 20*hz_;      
  }
  else  
  { 
    for(unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)  
    {
      if(Com_DOB_DSP_mode == 1)
        ComDOB_onestepZmp(i,temp_px,temp_py);
      else
        onestepZmp(i, temp_px, temp_py);

      for (unsigned int j = 0; j < t_total_; j++)  
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
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
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_ ) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = supportfoot_support_init_(0);
        temp_py(i) = supportfoot_support_init_(1) - zmp_offset_;
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick 
      {
        temp_px(i) = B - Kx + Kx / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
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
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_(current_step_number-1, 0);
        temp_py(i) = foot_step_support_frame_(current_step_number-1, 1) + zmp_offset_;
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
      {               
        temp_px(i) = (foot_step_support_frame_(current_step_number-1, 0) + foot_step_support_frame_(current_step_number, 0))/2 - Kx + Kx /(t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
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
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_)) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_(current_step_number-1, 0) ;
        temp_py(i) = foot_step_support_frame_(current_step_number-1, 1) + pow(-1,current_step_number-1)*zmp_offset_;
      }
      else if( i >= (t_total_ - t_rest_last_ - t_double2_) && (i < t_total_) && (current_step_num_ == total_step_num_ - 1))
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number-1, 0))/2;
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }       
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
      { 
        temp_px(i) = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    } 
  }  
}

void WalkingController::ComDOB_onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_);  
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  if(current_step_number == 0)
  {    
    temp = com_support_init_(0);
    for(int i = 0; i < t_total_; i++)
    {
      temp_px(i) = 0.0;
      temp_py(i) = (com_offset_(1) + com_support_init_(1));
    }   
  }
  else if(current_step_number == 1)
  {
    for(int i = 0; i < t_total_; i++)
    {
      temp_px(i) = 0.0;
      temp_py(i) = (foot_step_support_frame_(current_step_number-1, 1) + supportfoot_support_init_(1))/2;
    }
  }
  else
  {
    for(int i = 0; i < t_total_; i++)
    {
      temp_px(i) = 0.0;
      temp_py(i) = (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2;
    }
  }
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

void WalkingController::getCPTrajectory_M()
{
  if(walking_tick_ == 0)
  {
    dT(0) = t_total_/hz_;
    dT(1) = t_total_/hz_;
    cp(0) = com_support_init_(0); 
    cp(1) = com_support_init_(1);
    com(0) = com_support_init_(0);
    com(1) = com_support_init_(1);
    com_pre(0) = com_support_init_(0);
    com_pre(1) = com_support_init_(1);
   
    cp_eos(0) = 0.0;
    cp_eos(1) = com_support_init_(1);
  }
  // CP_eos 설정 /////////////////////////////////////////////////////////////////////
  // Step change 하면, 현재 발 기준으로 다음스텝에 CP_eos를 찍음. (보행 중)
  if( walking_tick_ >= t_start_ && walking_tick_ < t_start_ + t_rest_init_ + t_double1_ ) // First DSP까지는 CP eos를 바꾸면 안됨.
  {
    cp_eos(0) = foot_step_support_frame_(current_step_num_-1,0) ;    
    cp_eos(1) = foot_step_support_frame_(current_step_num_-1,1) ;
  }
  else if(walking_tick_ >= t_start_ + t_rest_init_ + t_double1_ ) // SSP부터 CP eos를 바꾸면 됨.
  {    
    cp_eos(0) = foot_step_support_frame_(current_step_num_,0) ;
    cp_eos(1) = foot_step_support_frame_(current_step_num_,1) ; 
  }
  // 첫번째 step의 eos 설정.
  if(current_step_num_ == 0)
  {
    if(walking_tick_ >= t_start_ && walking_tick_ < t_start_ +  t_double1_ + t_rest_init_) // First DSP까지 X방향은 0
    { cp_eos(0) = 0; } 
    else if(walking_tick_ >= t_start_ +  t_double1_ + t_rest_init_ && walking_tick_ < t_start_ + t_total_) // SSP 부터 X방향은 다음 발로 
    { cp_eos(0) = 0.2; } 

    if(walking_tick_ >= t_start_ - 0.5*hz_ && walking_tick_ < t_start_ +  t_double1_ + t_rest_init_) // 0.5초전부터 CoM을 미리 움직이기위함
    { cp_eos(1) = 0.0; }
    else if(walking_tick_ >= t_start_ +  t_double1_ + t_rest_init_ && walking_tick_ < t_start_ + t_total_)
    { cp_eos(1) = - 0.25562; } 
  }
  // 마지막 step의 eos 설정.
  if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_start_ && walking_tick_ < t_start_ + t_rest_init_ + t_double1_)
  {
    cp_eos(0) = 0;
    cp_eos(1) = 0;
  }
  else if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_start_ + t_rest_init_ + t_double1_)
  {
    cp_eos(0) = 0;
    cp_eos(1) = 0.12779; 
  }

  // dT 설정 
  if(walking_tick_ == t_start_ + t_rest_init_ + t_double1_)
  {
    dT(0) = t_total_/hz_;
    dT(1) = t_total_/hz_;
  }
  else if(current_step_num_ == 0 && walking_tick_ == 0)
  { dT(0) = 1.0; }
  else if(current_step_num_ == 0 && walking_tick_ == t_start_ - 0.5*hz_)
  { dT(1) = 0.5 + t_rest_init_/hz_; }

  // 좌표 변환
  if(walking_tick_ == t_start_ && current_step_num_ > 0 && current_step_num_ != total_step_num_ -1)  
  {
    Eigen::Vector2d temp_pos;
    for(int i=0; i<3; i++)
    {
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);
    }
    cp(0) = cp(0) - temp_pos(0);
    cp(1) = cp(1) + temp_pos(1); 
    com_pre(0) = com_pre(0) - temp_pos(0);
    com_pre(1) = com_pre(1) + temp_pos(1);   
  }
  else if(walking_tick_ == t_start_ && current_step_num_ == total_step_num_-1)
  {
    cp(0) = cp(0) - 0.2;
    cp(1) = cp(1) + 0.127794*2;
    com_pre(0) = com_pre(0) - 0.2;
    com_pre(1) = com_pre(1) + 0.127794*2;
  }
  
  // CP Control ////////////////////////////////////////////////////////////////////////////
  b(0)= exp(3.7*dT(0)); 
  p(0) = 1/(1-b(0)) * cp_eos(0) - b(0)/(1-b(0)) * cp(0); // X ZMP calculation
  
  cp_pre(0) = cp(0); 
  cp(0) = p(0) + exp(3.7*0.005)*(cp_pre(0)-p(0)); // X CP update
  com(0) = 1/(1+3.7*0.005) * com_pre(0) + cp(0) * (3.7*0.005)/(1+3.7*0.005); // X CoM update
  com_pre(0) = com(0);

  dT(0) = dT(0) - del_t; // X dT update
  if(dT(0) < 0.01)
  { dT(0) = 0.01; } // X dT Limit
 
  if(walking_tick_ >= 500)
  {
    b(1)= exp(3.7*dT(1));
    p(1) = 1/(1-b(1)) * cp_eos(1) - b(1)/(1-b(1)) * cp(1); // Y ZMP calculation
    
    cp_pre(1) = cp(1);  
    cp(1) = p(1) + exp(3.7*0.005)*(cp_pre(1)-p(1)); // Y CP update
    com(1) = 1/(1+3.7*0.005) * com_pre(1) + cp(1) * (3.7*0.005)/(1+3.7*0.005); // Y CoM update
    com_pre(1) = com(1);

    dT(1) = dT(1) - del_t;
    if(dT(1) < 0.01)
    { dT(1) = 0.01; } // Y dT Limit
  }
  cp_measured_(0) = com_support_current_(0) + com_support_current_dot_LPF(0)/3.7;
  cp_measured_(1) = com_support_current_(1) + com_support_current_dot_LPF(1)/3.7;

  p_d(0) = 1/(1 - exp(3.7*0.05)) * cp(0) - (exp(3.7*0.1))/(1- exp(3.7*0.05))*cp_measured_(0);
  p_d(1) = 1/(1 - exp(3.7*0.05)) * cp(1) - (exp(3.7*0.1))/(1- exp(3.7*0.05))*cp_measured_(1);
    
  e_cp_graph1 << com(0) << "," << cp(0) << "," << p_d(0) << "," << p(0) << "," << cp_eos(0) << "," << cp_measured_(0) << endl;
  e_cp_graph2 << com(1) << "," << cp(1) << "," << p_d(1) << "," << p(1) << "," << cp_eos(1) << "," << cp_measured_(1) << endl;
}

void WalkingController::getComTrajectory()
{
  if(walking_tick_ == 0)  
  { 
    preview_Parameter_CPM(1.0/hz_, 16*hz_/10, K_ ,com_support_init_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_); 
    UX_ = com_support_init_(0);
    UY_ = com_support_init_(1);
    xs_(0) = xi_; xs_(1) = 0; xs_(2) = 0;
    ys_(0) = yi_; ys_(1) = 0; ys_(2) = 0;
  }

  if(current_step_num_ == 0)
  { zmp_start_time_ = 0.0; }
  else
  { zmp_start_time_ = t_start_; }
          
  previewcontroller_CPM(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, A_bar_, B_bar_, XD_, YD_, X_bar_p_, Y_bar_p_);
  
  xs_(0) = com_support_current_(0); ys_(0) = com_support_current_(1);  

  com_desired_b(1) = com_desired_(1) - 0.5*foot_step_support_frame_(current_step_num_,1);
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

void WalkingController::getComTrajectory_ori()
{
  unsigned int planning_step_number = 3;
  unsigned int norm_size = 0;
  
  if(current_step_num_ >= total_step_num_ - planning_step_number)
    norm_size = (t_last_ - t_start_ + 1)*(total_step_num_ - current_step_num_) + 20*hz_;
  else
    norm_size = (t_last_ - t_start_ + 1)*(planning_step_number); 
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_ + 1;
 
  comGenerator(norm_size, planning_step_number);

  com_desired_bb = com_desired_b;
  com_desired_b = com_desired_; 

  if(current_step_num_ == 0)
  {
    com_desired_(0) = ref_com_(walking_tick_,0);
    com_desired_(1) = ref_com_(walking_tick_,1);
  }
  else
  {
    com_desired_(0) = ref_com_(walking_tick_ - t_start_, 0);
    com_desired_(1) = ref_com_(walking_tick_ - t_start_, 1);   
  }  
  if(Com_DOB_DSP_mode == 2)
    com_desired_(2) = pelv_support_start_.translation()(2);

  
  if(Com_DOB_DSP_mode == 0||Com_DOB_DSP_mode == 1)
    com_desired_(2) = pelv_support_start_.translation()(2);
    //if(tick_index >= 3*hz_)
      //com_desired_(1) = DyrosMath::cubic(tick_index, 3*hz_, 5*hz_, -0.129785, -0.129785 + 0.1, 0, 0);
  
  if(walking_tick_ == 0)
  {
    com_desired_b = com_desired_;
    com_desired_bb = com_desired_b;
  }
}

void WalkingController::comGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{ 
  ref_com_.resize(norm_size, 2); 
  Eigen::VectorXd temp_cx;
  Eigen::VectorXd temp_cy;
  
  unsigned int index = 0; 

  if(current_step_num_ == 0)  
  {
    for (int i = 0; i <= t_temp_; i++) 
    { 
      //econom2
      //ref_com_(i,0) = DyrosMath::cubic(i, 0, 2*hz_,com_support_init_(0) + com_offset_(0), 0, 0, 0);
      ref_com_(i,0) = DyrosMath::cubic(i, 0, 2*hz_,com_support_init_(0) + com_offset_(0), com_support_init_(0) + com_offset_(0), 0, 0);
      ref_com_(i,1) = com_support_init_(1);
      if(i >= 2*hz_)
      { 
        Eigen::Vector3d ref_y_com ;
        if(Com_DOB_DSP_mode == 0)
          ref_y_com = (DyrosMath::QuinticSpline(i, 2*hz_, 3*hz_,com_support_init_(1), 0, 0, com_support_init_(1), 0.289384/hz_,0));
        else if(Com_DOB_DSP_mode == 1)
          ref_y_com = (DyrosMath::QuinticSpline(i, 2*hz_, 3*hz_,com_support_init_(1), 0, 0, com_support_init_(1), 0.289384/hz_,0));
          //ref_y_com(0) = com_support_init_(1);
        ref_com_(i,1) = ref_y_com(0);
      }
      index++;
    }    
  }
  if(current_step_num_ >= total_step_num_ - planning_step_num)
  {  
    for(unsigned int i = current_step_num_; i < total_step_num_; i++)
    {
      if(Com_DOB_DSP_mode == 1)
        onestepCom(i, temp_cx, temp_cy);
        //ComDOB_onestepCom(i, temp_cx, temp_cy);
      else
        onestepCom(i, temp_cx, temp_cy);

      for(unsigned int j = 0; j < t_total_; j++)
      {
        ref_com_(index + j, 0) = temp_cx(j);
        ref_com_(index + j, 1) = temp_cy(j);    
      }
      index = index + t_total_;
    }
    
    for(unsigned int j = 0; j < 20*hz_; j++)
    {
      ref_com_(index + j, 0) = ref_com_(index -1, 0);
      ref_com_(index + j, 1) = ref_com_(index -1, 1);
    }

    if((current_step_num_ == total_step_num_ - 1)) 
    { Eigen::Vector3d ref_y_com ;
            
      for(int i = 0; i < 240; i++)
      {
        //econom2
        //ref_y_com = DyrosMath::QuinticSpline(i+239, 120, 479, 0.031081, -2.60209e-18, 1.05331e-05, 0.153963, 0, 0);
        ref_y_com = DyrosMath::QuinticSpline(i+239, 120, 479, 0.031081, -2.60209e-18, 1.05331e-05, 0.12779, 0, 0);
        ref_com_(index + i, 1) = ref_y_com(0) ;
      }
    }

    index = index + 20*hz_;      
  }
  else 
  { 
    for(unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)  
    {
      if(Com_DOB_DSP_mode == 1)
        onestepCom(i, temp_cx, temp_cy);
        //ComDOB_onestepCom(i, temp_cx, temp_cy);
      else
        onestepCom(i, temp_cx, temp_cy);

      for (unsigned int j = 0; j < t_total_; j++) 
      {
        ref_com_(index+j,0) = temp_cx(j);
        ref_com_(index+j,1) = temp_cy(j);
      }      
      index = index + t_total_; 
    }   
  }   
}

void WalkingController::ComDOB_onestepCom(unsigned int current_step_number, Eigen::VectorXd& temp_cx, Eigen::VectorXd& temp_cy)
{
  temp_cx.resize(t_total_);
  temp_cy.resize(t_total_);
  temp_cx.setZero();
  temp_cy.setZero();

  if(current_step_number == 0)
  {
    for(int i = 0;i < 0.3*t_total_; i++)
    {
      //temp_cx(i) = DyrosMath::cubic(i, 0, 0.5*t_total_-1, 0, 0.04, 0, 0);
      temp_cx(i) = 0.0;
      temp_cy(i) = com_support_init_(1);
    }
    for(int i = 0.3*t_total_;i < 0.6*t_total_; i++)
    {
      //temp_cx(i) = DyrosMath::cubic(i, 0, 0.5*t_total_-1, 0, 0.04, 0, 0);
      temp_cx(i) = 0.0;
      temp_cy(i) = com_support_init_(1);
    }
    for(int i = 0.6*t_total_;i < t_total_; i++)
    {
      //temp_cx(i) = DyrosMath::cubic(i, 0, 0.5*t_total_-1, 0, 0.04, 0, 0);
      temp_cx(i) = 0.0;
      temp_cy(i) = com_support_init_(1);
    }
  }
  else if (current_step_number == 1)
  {
    for(int i = 0;i < t_total_; i++)
    {
      temp_cx(i) = 0.0;
      temp_cy(i) = (foot_step_support_frame_(current_step_number-1, 1) + supportfoot_support_init_(1))/2;
    }
  }
  else
  {
    for(int i = 0; i < t_total_; i++)
    {
      temp_cx(i) = 0.0;
      temp_cy(i) = (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2;
    }
  }
}

void WalkingController::onestepCom(unsigned int current_step_number, Eigen::VectorXd& temp_cx, Eigen::VectorXd& temp_cy)
{
  temp_cx.resize(t_total_); 
  temp_cy.resize(t_total_);
  temp_cx.setZero();
  temp_cy.setZero();

  double A = 0, B = 0, Cx1 = 0, Cx2 = 0, Cy1 = 0, Cy2 = 0, Kx = 0, Ky = 0, wn = 0 ;
  if(current_step_number == 0)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = -(foot_step_support_frame_(current_step_number, 1) )/2 ;
    B =  (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    Cx1 = Kx - B;
    Cx2 = Kx/(wn*0.15);
    Cy1 = Ky - A;
    Cy2 = Ky/(wn*0.15);
        
    for(int i = 0; i < t_total_; i++)
    {
      double temp;
      if(total_length_input == 0)
        temp = 0.0;
      else
        temp = 0.10154;
      
      temp_cx(i) = DyrosMath::cubic(i, 0, t_total_-1,0 ,temp , 0, Kx/(t_rest_init_ + t_double1_));

      if(i >= 0 && i < (t_rest_init_ + t_double1_))
      {  
        temp_cy(i) = com_offset_(1) + com_support_init_(1) + Ky / (t_rest_init_ + t_double1_) * (i+1);
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < t_total_ - t_rest_last_ - t_double2_ )
      {
        temp_cy(i) = A + com_offset_(1) + com_support_init_(1) + Cy1 *cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_-0.15)) ;
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_) 
      {
        temp_cy(i) = Ky + (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }     
    }    
  }
  else if(current_step_number == 1)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - supportfoot_support_init_(1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*0.45));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    Cx1 = Kx - B;
    Cx2 = Kx/(wn*0.15);
    Cy1 = Ky - A;
    Cy2 = Ky/(wn*0.15);    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < (t_rest_init_ + t_double1_))
      { 
        temp_cx(i) = (foot_step_support_frame_(current_step_number-1, 0) + supportfoot_support_init_(0))/2 + Kx / (t_rest_init_+ t_double1_) * (i+1);
        temp_cy(i) = (foot_step_support_frame_(current_step_number-1, 1) + supportfoot_support_init_(1))/2 + Ky / (t_rest_init_+ t_double1_) * (i+1);
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_)) 
      {
        temp_cx(i) = (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Cx1 *cosh(wn*(i/hz_ - 0.15)) + Cx2*sinh(wn*(i/hz_-0.15)) + B;
        temp_cy(i) = A + (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Cy1 *cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_-0.15)) ;
      }
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_) 
      { 
        temp_cx(i) = (foot_step_support_frame_(current_step_number, 0)+ foot_step_support_frame_(current_step_number-1, 0)) /2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_cy(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }
  }
  else
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - foot_step_support_frame_(current_step_number-2, 1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*0.45));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    Cx1 = Kx - B;
    Cx2 = Kx/(wn*0.15);
    Cy1 = Ky - A;
    Cy2 = Ky/(wn*0.15);
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < (t_rest_init_ + t_double1_))
      {
        temp_cx(i) = (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Kx/(t_rest_init_ + t_double1_)*(i);
        temp_cy(i) = (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Ky/(t_rest_init_ + t_double1_)*(i);
      }            
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_)) 
      {
        temp_cx(i) = (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Cx1 *cosh(wn*(i/hz_ - 0.15)) + Cx2*sinh(wn*(i/hz_-0.15)) + B;
        temp_cy(i) = A + (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Cy1 *cosh(wn*(i/hz_ - 0.15)) + Cy2*sinh(wn*(i/hz_-0.15)) ;
         
      }
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_) 
      {
        temp_cx(i) = (foot_step_support_frame_(current_step_number, 0)+ foot_step_support_frame_(current_step_number-1, 0)) /2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_cy(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 - Ky/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
      
      if(i >= (t_rest_init_ + t_double1_) && (current_step_num_ == total_step_num_ - 1) && i < t_total_ ) 
      { 
        Eigen::Vector3d ref_x_com ;
        ref_x_com = DyrosMath::QuinticSpline(i, 30, 239, (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Kx, 0.289384/hz_, 0, (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + B, 0, 0);//com_support_init_(1)+com_offset_(1), 0.289384/hz_,0));
        //temp_cx(i) = ref_x_com(0);
        temp_cx(i) = (foot_step_support_frame_(current_step_number, 0)+ foot_step_support_frame_(current_step_number-1, 0)) /2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
      if( i >= 120 && i < t_total_ && (current_step_num_ == total_step_num_ - 1)) 
      { 
        Eigen::Vector3d ref_y_com ; 
        ref_y_com = DyrosMath::QuinticSpline(i, 120, 479, 0.031081, (Cy1 *sinh(wn*(120/hz_ - 0.15))*wn/hz_ + Cy2*cosh(wn*(120/hz_-0.15))*wn/hz_), (Cy1 *cosh(wn*(120/hz_ - 0.15))*wn/hz_*wn/hz_ + Cy2*sinh(wn*(120/hz_-0.15))*wn/hz_*wn/hz_), 0.12779, 0, 0);
        temp_cy(i) = ref_y_com(0);
      }      
    } 
  }
    
}

void WalkingController::getPelvTrajectory()
{
  if(Com_DOB_DSP_mode == 0)
  {
    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0*(com_desired_(0) - com_support_current_(0));// - 0.15*zmp_err_(0);
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 1.0*(com_desired_(1) - com_support_current_(1));// - 0.8*zmp_err_(1);
    //pelv_trajectory_support_.translation()(2) = pelv_support_current_.translation()(2) + 1.0*(com_desired_(2) - com_support_current_(2));
    pelv_trajectory_support_.translation()(2) = com_desired_(2); 
  }
  else if(Com_DOB_DSP_mode == 1)
  { 
    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0*(com_dob_u(0) - com_support_current_(0));
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 1.0*(com_dob_u(1) - com_support_current_(1));
    pelv_trajectory_support_.translation()(2) = com_dob_u(2);
  }
  else if(Com_DOB_DSP_mode == 2)
  { 
    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0*(com_dob_u(0) - com_support_current_(0));
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 1.0*(com_dob_u(1) - com_support_current_(1));
    pelv_trajectory_support_.translation()(2) = pelv_support_current_.translation()(2) + 1.0*(com_dob_u(2) - com_support_current_(2));
    //pelv_trajectory_support_.translation()(2) = com_dob_u(2);
  }

  Eigen::Vector3d Trunk_trajectory_euler;
  Trunk_trajectory_euler.setZero();
  
  pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2))*DyrosMath::rotateWithY(Trunk_trajectory_euler(1))*DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
  
  Eigen::Vector3d com_desired_imu = pelv_support_current_.rotation()*com_desired_;
  pelv_support_current_imu_.translation() = pelv_support_current_.rotation()*pelv_support_current_.translation();
  pelv_support_current_imu_.linear() = pelv_trajectory_support_.linear();

  e_com_x_graph << com_desired_(0) << "," << com_support_current_(0) << "," << com_support_current_dot_LPF(0) << "," << com_support_current_ddot_LPF(0) << "," << com_dob_u(0) << "," << com_support_current_imu(0) << "," << com_desired_imu(0) << endl;
  e_com_y_graph << com_desired_(1) << "," << com_support_current_(1) << "," << com_support_current_dot_LPF(1) << "," << com_support_current_ddot_LPF(1) << "," << com_dob_u(1) << "," << com_support_current_imu(1) << "," << pelv_support_current_.rotation()(0,0) << "," << R_.rotation()(0,0) << "," << q_sim_virtual_(3) << endl;
  q_sim_virtual_ = model_.getMujocoCom();
  e_com_z_graph << com_desired_(2) << "," << com_support_current_(2) << "," << com_support_current_dot_LPF(2) << "," << com_support_current_ddot_LPF(2) << "," << com_dob_u(2) << "," << com_support_current_imu(2) << "," << q_sim_virtual_(2) << endl;
}

void WalkingController::supportToFloatPattern()
{  
  pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*pelv_trajectory_support_;
  pelv_support_current_imu_calc_ = DyrosMath::inverseIsometry3d(pelv_support_current_imu_)*pelv_support_current_imu_;
  lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*lfoot_trajectory_support_;
  rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*rfoot_trajectory_support_;
}

void WalkingController::getFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  for(int i=0; i<6; i++)
  { target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i); }
 
  if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_) 
  { 
    lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();  
    rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
    
    lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
    rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

    if(foot_step_(current_step_num_,6) == 1)  
    { 
      lfoot_trajectory_support_.translation().setZero();
      lfoot_trajectory_euler_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation()(2) = 0;
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
    }     
    else if(foot_step_(current_step_num_,6) == 0)  
    { 
      rfoot_trajectory_support_.translation().setZero();
      rfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.translation()(2) = 0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_; 
    }     

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
  }
  
  else if(walking_tick_ >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_ < t_start_ + t_total_ - t_double2_ - t_rest_last_)  
  {   
    if(foot_step_(current_step_num_,6) == 1) 
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();             
      lfoot_trajectory_euler_support_.setZero(); 

      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      
      if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0) 
      { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_+ t_rest_init_ + t_double1_,t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,0,foot_height_,0.0,0.0); }  
      else
      { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0); }
      
      for(int i=0; i<2; i++) // X, Y 방향 궤적 생성, 여기서 불연속 문제를 회복해야함. 스윙발이 완벽히 따라가지 못하고 지지발이 되었을때, 현재 지지발 기준으로 봤을때 스윙발이 오차가 생긴것이기 때문에 3차곡선으로 이어줘야함. 
      { rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_ + t_double1_ , t_start_+t_total_-t_rest_last_-t_double2_, rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); } 
      
      rfoot_trajectory_euler_support_(0) = 0;
      rfoot_trajectory_euler_support_(1) = 0;
      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_ + t_rest_init_ + t_double1_,t_start_ + t_total_ - t_rest_last_ - t_double2_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발이 지지발일때, 지지발은 고정, 왼발은 목표 위치로 스윙
    { 
      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation(); 
      rfoot_trajectory_euler_support_.setZero(); 

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
 
      if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
      { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0); }
      else
      { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0); }
         
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
    if(foot_step_(current_step_num_,6) == 1) // 왼쪽 발 지지
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
    else if (foot_step_(current_step_num_,6) == 0) 
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
  e_lfoot_z_graph << rfoot_support_current_.translation()(2) << endl;
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

void WalkingController::computeIK_e_jaco(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des)
{
  Eigen::Isometry3d trunk_lleg_transform, trunk_rleg_transform;
  
  trunk_lleg_transform.translation() = float_lleg_transform.translation() - float_trunk_transform.translation();
  trunk_rleg_transform.translation() = float_rleg_transform.translation() - float_trunk_transform.translation();
  
  Eigen::Isometry3d global_transform;
  global_transform.setIdentity();
  global_transform.linear() = R_.linear();

  trunk_lleg_transform = global_transform*trunk_lleg_transform;
  trunk_rleg_transform = global_transform*trunk_rleg_transform;

  Eigen::Matrix<double, 6, 34> tmp_l_leg_full_jaco_;
  Eigen::Matrix<double, 6, 34> tmp_r_leg_full_jaco_;

  model_.getJacobianMatrix34DoF((DyrosJetModel::EndEffector)0, &tmp_l_leg_full_jaco_);
  model_.getJacobianMatrix34DoF((DyrosJetModel::EndEffector)1, &tmp_r_leg_full_jaco_);
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
   
  if(walking_tick_ == 0)
    tick_index =0;
  tick_index ++;

  if(Com_DOB_DSP_mode == 2)
    if (walking_tick_ == 2*hz_)
      walking_tick_ --;

  if(Com_DOB_DSP_mode == 1)
    if (walking_tick_ == 2*hz_)
      walking_tick_ --;

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

void WalkingController::hip_compensator()
{
  double left_hip_angle = 3.0*DEG2RAD, right_hip_angle = 3.0*DEG2RAD, left_hip_angle_first_step = 3.0*DEG2RAD, right_hip_angle_first_step = 3.0*DEG2RAD,
      left_hip_angle_temp = 0.0, right_hip_angle_temp = 0.0, temp_time = 0.1*hz_;

  if(current_step_num_ == 0)
  {
    if(foot_step_(current_step_num_, 6) == 1) //left support foot
    {
      if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_angle_first_step, 0.0, 0.0);
      else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_angle_first_step, 0.0, 0.0, 0.0);
      else
        left_hip_angle_temp = 0;
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_angle_first_step, 0.0, 0.0);
      else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_angle_first_step, 0.0, 0.0, 0.0);
      else
        right_hip_angle_temp = 0;
    }
  }
  else
  {
    if(foot_step_(current_step_num_, 6) == 1) //left support foot
    {
      if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_angle, 0.0, 0.0);
      else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_angle, 0.0, 0.0, 0.0);
      else
        left_hip_angle_temp = 0;
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_angle, 0.0, 0.0);
      else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_angle, 0.0, 0.0, 0.0);
      else
        right_hip_angle_temp = 0;
    }
  }
  desired_q_(1) = desired_q_(1) + left_hip_angle_temp;
  desired_q_(7) = desired_q_(7) - right_hip_angle_temp;
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
  {  
    current_u(i) = (current_motor_q_leg_(i) - (1 - Kp*del_t)*pre_motor_q_leg_(i)) / (Kp*del_t);
  }
  
  Eigen::Vector12d d_hat;    
  d_hat = current_u - DOB_IK_output_b_ ;  
  
  if(walking_tick_ == 0)
    d_hat_b = d_hat;

  d_hat = 0.7*d_hat_b + 0.3*d_hat; // 필터링
 
  double default_gain = 0.0;  
  double compliant_gain = 0.0;
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
          if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)  
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
        else // 왼발 지지 상태
        {
          gain_temp = default_gain;
        }
      }       
      DOB_IK_output_(i) = desired_leg_q(i) + gain_temp*d_hat(i);   
    }
    else //오른 다리 관절
    { 
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
      DOB_IK_output_(i) = desired_leg_q(i) + gain_temp*d_hat(i);   
    }
  }  
  pre_motor_q_leg_ = current_motor_q_leg_; 
  d_hat_b = d_hat; 
  DOB_IK_output_b_ = DOB_IK_output_; 
}

//econom2 research
//com dob
void WalkingController::Com_DoB_js(Eigen::Vector12d desired_leg_q)
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
  {  
    current_u(i) = (current_motor_q_leg_(i) - (1 - Kp*del_t)*pre_motor_q_leg_(i)) / (Kp*del_t);
  }
  
  Eigen::Vector12d d_hat;    
  d_hat = current_u - DOB_IK_output_b_ ;  
  
  if(walking_tick_ == 0)
    d_hat_b = d_hat;

  d_hat = 0.7*d_hat_b + 0.3*d_hat; // 필터링
 
  Eigen::Vector12d compliant_gain;
  for (int i = 0; i < 12; i++)
  {
    //compliant_gain(i) = DyrosMath::cubic(tick_index, 3.0*hz_, 4.0*hz_, 0.0, 0.0, 0.0, 0.0);
    compliant_gain(i) = DyrosMath::cubic(tick_index, 3.0*hz_, 4.0*hz_, 0.0, 5.0, 0.0, 0.0);
  }

  d_hat(2) = d_hat(2) - 0.001; d_hat(8) = d_hat(8) + 0.001; 
  d_hat(3) = d_hat(3) - 0.0025; d_hat(9) = d_hat(9) + 0.0025; 
  //d_hat(4) = 0; d_hat(10)= 0;
  for (int i = 0; i < 12; i ++)
  {
    if(i==2||i==3||i==4||i==8||i==9||i==10)
    {
      compliant_gain(i) = compliant_gain(i)*0.2;
    }
    DOB_IK_output_(i) = desired_leg_q(i) + compliant_gain(i)*d_hat(i); 
  }
  pre_motor_q_leg_ = current_motor_q_leg_; 
  d_hat_b = d_hat;
  
  e_tmp_2_graph << d_hat(0) << "," << DOB_IK_output_(0) << "," << current_motor_q_leg_(0) << "," 
                << d_hat(1) << "," << DOB_IK_output_(1) << "," << current_motor_q_leg_(1) << "," 
                << d_hat(2) << "," << DOB_IK_output_(2) << "," << current_motor_q_leg_(2) << "," 
                << d_hat(3) << "," << DOB_IK_output_(3) << "," << current_motor_q_leg_(3) << "," 
                << d_hat(4) << "," << DOB_IK_output_(4) << "," << current_motor_q_leg_(4) << "," 
                << d_hat(5) << "," << DOB_IK_output_(5) << "," << current_motor_q_leg_(5) << "," << collide_ft_(0) << "," << collide_ft_(1) << "," << collide_ft_(2) << endl;
  DOB_IK_output_b_ = DOB_IK_output_; 
}

void WalkingController::Com_DoB1_COM()
{
  double del_t = 1/hz_;
  double kp_y, kv_y;
  del_t = 1/hz_;

  kp_y = 225.52;
  kv_y = 6.161;

  double cut_off_freq = 6.0;
  double w = 2*M_PI*cut_off_freq;
  double zeta = 1.0;

  com_dob_u = com_desired_;

  Eigen::Vector3d com_dob_gain;
  com_dob_gain.setZero();
  com_dob_gain(1) = DyrosMath::cubic(tick_index, 3.0*hz_, 4.0*hz_, 0.0, 0.0, 0.0, 0.0);
  if(tick_index >=1000)
  {
    cout << "Ready" << endl;
  }

  com_dob_u(1) = com_desired_(1) + com_dob_gain(1) * (com_dob_d_hat(1)+0.0003);

  if(tick_index >= 600)
  {
    com_dob_u(1) = com_dob_u(1) + 0.0;
  }

  if(walking_tick_ == 0)
  {
    com_dob_d.setZero();
  }
  else
  {    
    com_dob_d(1) = (com_support_current_ddot_imu(1) + kv_y*com_support_current_dot_imu(1) + kp_y*com_support_current_imu(1))/kp_y - com_dob_u(1);
  }
  
  if(walking_tick_ == 0)
  {
    com_dob_d_hat_b = com_dob_d_hat;
    com_dob_d_hat_bb = com_dob_d_hat_b;
  }
  
  com_dob_d_hat_bb = com_dob_d_hat_b;
  com_dob_d_hat_b = com_dob_d_hat;
  com_dob_d_hat = (2*(1 + zeta*w*del_t)*com_dob_d_hat_b - com_dob_d_hat_bb + w*w*del_t*del_t*com_dob_d)/(1 + w*w*del_t*del_t + 2*zeta*w*del_t);

  e_dis_1_graph << com_dob_d_hat(1) << "," << sim_time_ << "," << pelv_support_current_.rotation()(0,0) << endl;
}

void WalkingController::Com_DoB2_ESO()
{
  double uff = com_desired_(1) + DyrosMath::cubic(tick_index, 5.0*hz_, 6.0*hz_, 0.0, 0.0, 0.0, 0.0);

  //double kp = 125, kv = 10;
  double kp = 225.52, kv = 6.161;

  Eigen::MatrixXd A, B, C, C_cp, D, Leso, Keso;

  double y_hat, y_hat_cp;
  double y_sensor, y_model, y_dynamics, y_cp;

  Leso.resize(1,3); Keso.resize(1,2);
  A.resize(3,3);
  A << 0,1,0, -kp,-kv,kp, 0,0,0;
  B.resize(3,1);
  B << 0,kp,0;

  if(walking_tick_ == 0)
  {
    ob_x_hat_y.setZero();
    ob_x_hat_y(0) = com_support_init_(1);
    ob_x_hat_y_b = ob_x_hat_y;
    ob_x_hat_dot_y.setZero();
    uy_b = (Keso(0) + 1)*uff - Keso(0)*ob_x_hat_y(0) - Keso(1)*ob_x_hat_y(1);
  }

  double zmp_com_cp = 1;//0 zmp 1 com 2 cp
  if(zmp_com_cp == 0)
  {
    //Leso << 5.4206, -0.0986, 4.7955;//-10
    Leso << 10.1302, 7.0109, 9.5910;//-20
    //Leso << 24.2591, 28.3394, 23.9774;//-50

    A << 0,1,0, -kp,-kv,kp, 0,0,0;
    C.resize(1,3);
    C << 1+zc_*kp/GRAVITY,zc_*kv/GRAVITY,0;
    D.resize(1,1);
    D << -zc_*kp/GRAVITY;

    y_hat = C(0)*ob_x_hat_y_b(0) + C(1)*ob_x_hat_y_b(1) + C(2)*ob_x_hat_y_b(2) + D(0)*uy_b;
    y_sensor = zmp_e_calc_(1);
    y_model = C(0)*com_support_current_imu(1) + C(1)*com_support_current_dot_imu(1) + C(2)*ob_x_hat_y_b(2) + D(0)*uy_b;
    y_dynamics = com_support_current_imu(1) - zc_*com_support_current_ddot_imu(1)/GRAVITY;

    cout << "zmp" << endl;
  }
  else if(zmp_com_cp == 1)
  {
    double Keso_real = -10;
    double Keso_im = 2;

    Keso << (Keso_real*Keso_real - Keso_im*Keso_im)/kp - 1, -(2*Keso_real + kv)/kp;

    double Leso_r = -30;
    double Leso_im = 2;
    double Ll1pLl2 = 2*Leso_r;
    double Ll1mLl2 = Leso_r*Leso_r - Leso_im*Leso_im;
    double Ll3 = -20;

    Leso(0) = -kv - Ll1pLl2 - Ll3;
    Leso(1) = Ll1mLl2 + Ll3*(Ll1pLl2) - kp - Leso(0)*kv;
    Leso(2) = -Ll3*Ll1mLl2/kp;

    C.resize(1,3);
    C << 1,0,0;
    C_cp.resize(1,3);
    C_cp << 1,sqrt(zc_/GRAVITY),0;
    D.resize(1,1);
    D << 0;
    y_hat = C(0)*ob_x_hat_y_b(0) + C(1)*ob_x_hat_y_b(1) + C(2)*ob_x_hat_y_b(2) + D(0)*uy_b;
    y_hat_cp = C_cp(0)*ob_x_hat_y_b(0) + C_cp(1)*ob_x_hat_y_b(1) + C_cp(2)*ob_x_hat_y_b(2) + D(0)*uy_b;
    
    y_sensor = com_support_current_imu(1);
    y_cp = C_cp(0)*com_support_current_imu(1) + C_cp(1)*com_support_current_dot_imu(1) + C_cp(2)*com_support_current_ddot_imu(1) + D(0)*uy_b;

    cout << "com" << endl;
  }
  else if(zmp_com_cp == 2)
  {
    double Keso_real = -10;
    double Keso_im = 1;

    Keso << (Keso_real*Keso_real - Keso_im*Keso_im)/kp - 1, -(2*Keso_real + kv)/kp;
    
    double Leso_r = -30;
    double Leso_im = 2;
    double Ll1pLl2 = 2*Leso_r;
    double Ll1mLl2 = Leso_r*Leso_r - Leso_im*Leso_im;
    double Ll3 = -20;

    Leso(2) = -Ll3*Ll1mLl2/kp;

    Eigen::Matrix2d calc_tmp_mat;
    calc_tmp_mat.setZero();
    calc_tmp_mat << 1, sqrt(zc_/GRAVITY), kv - sqrt(zc_/GRAVITY)*kp, 1;

    Eigen::MatrixXd calc_tmp_vec;
    calc_tmp_vec.resize(2,1);
    calc_tmp_vec << -kv - Ll1pLl2 - Ll3, Ll1mLl2 + Ll3*(Ll1pLl2) - kp - kp*sqrt(zc_/GRAVITY)*Leso(2);

    calc_tmp_vec = calc_tmp_mat.inverse()*calc_tmp_vec;

    Leso(0) = calc_tmp_vec(0);
    Leso(1) = calc_tmp_vec(1);

    C_cp.resize(1,3);
    C_cp << 1,sqrt(zc_/GRAVITY),0;
    D.resize(1,1);
    D << 0;

    y_hat = C_cp(0)*ob_x_hat_y_b(0) + C_cp(1)*ob_x_hat_y_b(1) + C_cp(2)*ob_x_hat_y_b(2) + D(0)*uy_b;
    y_sensor = C_cp(0)*com_support_current_imu(1) + C_cp(1)*com_support_current_dot_imu(1) + C_cp(2)*com_support_current_ddot_imu(1) + D(0)*uy_b;

    cout << "cp" << endl;
  }

  double dob_gain = DyrosMath::cubic(tick_index, 3.0*hz_, 4.0*hz_,0.0, 1.0, 0.0, 0.0);

  ob_x_hat_dot_y = A*ob_x_hat_y_b + B*uy_b + Leso*(y_sensor - y_hat);

  e_dis_2_graph << ob_x_hat_y(0) << "," << ob_x_hat_y(2);
  
  ob_x_hat_y = ob_x_hat_y + ob_x_hat_dot_y*1/hz_;

  double u_ref_b = uff + dob_gain*ob_x_hat_y_b(2);

  ob_x_hat_y_b = ob_x_hat_y;
  
  double u_ref = uff + dob_gain*ob_x_hat_y_b(2);
  uy_b = u_ref;

  if(tick_index >= 4.0*hz_)
  {
    cout << "Ready" << endl;
    uy_b = (Keso(0) + 1)*u_ref - Keso(0)*ob_x_hat_y(0) - Keso(1)*ob_x_hat_y(1);
  }
  
  com_dob_u = com_desired_;
  //com_dob_u(1) = uy_b;
  com_dob_u(1) = uff;

  e_dis_2_graph << "," << u_ref_b << "," << u_ref << "," << R_angle << "," << uff << endl;
}

void WalkingController::Com_SO()
{
  double uff1 = com_desired_(1);
  double uff2 = com_desired_(1) - 0.5*foot_step_support_frame_(current_step_num_,1);
  double kp = 125, kv = 10;

  Eigen::MatrixXd A, B, C, D, Leso, Keso;

  double y_hat1, y_hat2;
  double y_sensor1, y_sensor2;

  Leso.resize(1,3); Keso.resize(1,2);
  A.resize(3,3);
  A << 0,1,0, -kp,-kv,kp, 0,0,0;
  B.resize(3,1);
  B << 0,kp,0;

  double Keso_real = -10;
  double Keso_im = 2;

  Keso << (Keso_real*Keso_real - Keso_im*Keso_im)/kp - 1, -(2*Keso_real + kv)/kp;

  if(walking_tick_ == 0)
  {
    ob_x_hat_y.setZero();
    ob_x_hat_y2.setZero();
    ob_x_hat_y(0) = com_support_init_(1);
    ob_x_hat_y2(0) = com_support_init_(1);
    ob_x_hat_y_b = ob_x_hat_y;
    ob_x_hat_y_b2 = ob_x_hat_y2;
    ob_x_hat_dot_y.setZero();
    ob_x_hat_dot_y2.setZero();
    uy_b = (Keso(0) + 1)*uff1 - Keso(0)*ob_x_hat_y(0) - Keso(1)*ob_x_hat_y(1);
  }

  double Leso_r = -30;
  double Leso_im = 2;
  double Ll1pLl2 = 2*Leso_r;
  double Ll1mLl2 = Leso_r*Leso_r - Leso_im*Leso_im;
  double Ll3 = -20;
  
  Leso(0) = -kv - Ll1pLl2 - Ll3;
  Leso(1) = Ll1mLl2 + Ll3*(Ll1pLl2) - kp - Leso(0)*kv;
  Leso(2) = -Ll3*Ll1mLl2/kp;
  
  C.resize(1,3);
  C << 1,0,0;
  D.resize(1,1);
  D << 0;
  
  y_hat1 = C(0)*ob_x_hat_y_b(0) + C(1)*ob_x_hat_y_b(1) + C(2)*ob_x_hat_y_b(2) + D(0)*uy_b;
  y_hat2 = C(0)*ob_x_hat_y_b2(0) + C(1)*ob_x_hat_y_b2(1) + C(2)*ob_x_hat_y_b2(2) + D(0)*uff2;

  y_sensor1 = com_support_current_imu(1);
  y_sensor2 = com_support_current_imu(1) - 0.5*foot_step_support_frame_(current_step_num_,1);
  
  com_dob_u = com_desired_;
  com_dob_u(1) = uy_b;

  ob_x_hat_dot_y = A*ob_x_hat_y_b + B*uy_b + Leso*(y_sensor1 - y_hat1);
  ob_x_hat_dot_y2 = A*ob_x_hat_y_b2 + B*uff2 + Leso*(y_sensor2 - y_hat2);
  
  double theta = abs(atan2(uff2, zc_));
  com_dot_desired_(1) = (uff2 - com_desired_b(1))*hz_;

  e_dis_2_graph << ob_x_hat_y2(0) << "," << ob_x_hat_y2(1) << "," << ob_x_hat_y2(2) << "," << uff2 << "," << ob_x_hat_y2(0) + 0.5*foot_step_support_frame_(current_step_num_,1) << "," << theta << endl;
  
  ob_x_hat_y = ob_x_hat_y + ob_x_hat_dot_y*1/hz_;
  ob_x_hat_y2 = ob_x_hat_y2 + ob_x_hat_dot_y2*1/hz_;
  ob_x_hat_y_b = ob_x_hat_y;
  ob_x_hat_y_b2 = ob_x_hat_y2;

  double u_ref = uff1;
  uy_b = u_ref;
  
  if(tick_index >= 2.0*hz_)
  {
    uy_b = (Keso(0) + 1)*u_ref - Keso(0)*ob_x_hat_y(0) - Keso(1)*ob_x_hat_y(1);
  }

  com_dob_u = com_desired_;
  com_dob_u(1) = uy_b;
  double collide_tmp = 0.1 *hz_;
  if(walking_tick_ > t_start_ + t_total_ - t_rest_last_ - t_double2_ - collide_tmp)
  {
    if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ + collide_tmp)
    {
      com_dob_u(2) = com_dob_u(2) - 10*abs(ob_x_hat_y2(1)*sin(theta)*cos(theta)/kp);
    }
  }
}

//jump
void WalkingController::e_jump_trajectory()
{
  double jump_ready_tick = 3.0*hz_;
  double jump_start_tick = 5.0*hz_;
  double jump_motion_time = 0.7;
  double jump_motion_tick = 0.7*hz_;

  double jump_ready_height = 0.7;
  double jump_start_height = 0.75;
  double jump_target_height = 0.5;

  double jump_target_velocity = sqrt(2*GRAVITY*jump_target_height);
  double jump_target_acceleration = 0.0;

  //ready phase
  com_dob_u(0) = com_desired_(0);
  com_dob_u(1) = com_desired_(1);
  com_dob_u(2) = DyrosMath::cubic(tick_index, 0.0, jump_ready_tick, zc_, jump_ready_height, 0.0, 0.0);

  //5th order polynomial
  Eigen::Matrix3d calc_poly_coeff_mat_;
  calc_poly_coeff_mat_.setZero();
  calc_poly_coeff_mat_ <<   pow(jump_motion_time,3),    pow(jump_motion_time,4),    pow(jump_motion_time,5),
                          3*pow(jump_motion_time,2),  4*pow(jump_motion_time,3),  5*pow(jump_motion_time,4),
                          6*pow(jump_motion_time,1), 12*pow(jump_motion_time,2), 20*pow(jump_motion_time,3);
  
  Eigen::Vector3d calc_poly_coeff_vec_;
  calc_poly_coeff_vec_ << jump_start_height - jump_ready_height, jump_target_velocity, jump_target_acceleration;

  Eigen::Vector3d poly_coeff_ = calc_poly_coeff_mat_.inverse()*calc_poly_coeff_vec_;

  if (tick_index >= jump_start_tick)
  {
    double jump_time = (tick_index - jump_start_tick)/hz_;
    if (jump_time <= jump_motion_time)
    {
      double tra_tmp = jump_ready_height + poly_coeff_(0)*pow(jump_time,3) + poly_coeff_(1)*pow(jump_time,4) + poly_coeff_(2)*pow(jump_time,5);
      com_dob_u(2) = tra_tmp;
    }
    else
    {
      //com_dob_u(2) = jump_start_height + DyrosMath::cubic(tick_index, 1141, 1200, 0.0, -0.05, jump_target_velocity/200, 0.0);
      com_dob_u(2) = jump_start_height + DyrosMath::cubic(tick_index, 1141, 1200, 0.0, -0.05, 0.0, 0.0);      
    }
  }

  e_jump_impedance();
}

void WalkingController::e_jump_impedance()
{
  double mass_gravity = 0;
  if (tick_index == 0)
  {
    mass_gravity = 443;
    z_pos = 0;
    z_vel = 0;
  }

  double k_coeff = 100000;
  double z_acc = 0.0;
  if(tick_index >= 1150)
  {
    if(l_ft_(2) + r_ft_(2) < 0)
      {
        z_acc = (- l_ft_(2) - r_ft_(2) - mass_gravity)/k_coeff;
        z_vel = z_vel + z_acc*1/hz_;
        z_pos = z_pos + z_vel*1/hz_ + 0.5*z_acc*1/hz_*1/hz_;
      }
  }
    
  e_jump_graph << l_ft_(2) + r_ft_(2) << "," << z_acc << "," << z_vel << "," << z_pos << endl;
  //com_dob_u(2) = com_dob_u(2) - z_pos;
}

void WalkingController::e_jump_zmp_control()
{
  double prev_time = 1.6;
  double prev_tick = prev_time*hz_;

  Eigen::VectorXd zmp_x_ref, zmp_y_ref;
  zmp_x_ref.resize(prev_tick);
  zmp_y_ref.resize(prev_tick);

  for(int i = 0; i < prev_tick; i++)
  {
    zmp_x_ref(i) = ref_zmp_(i,0);
    zmp_y_ref(i) = ref_zmp_(i,1);
  }

  Eigen::MatrixXd A,B,C,Q,R;
  A.resize(3,3);
  A.setZero();
  A(0,0) = 1; A(0,1) = del_t; A(0,2) = 0.5*del_t*del_t;
              A(1,1) = 1;     A(1,2) = del_t;
                              A(2,2) = 1;

  B.resize(3,1);
  B.setZero();
  B(0,0) = del_t*del_t*del_t/6;
  B(1,0) = del_t*del_t/2;
  B(2,0) = del_t;

  C.resize(1,3);
  C.setZero();
  C(0,0) = 1; C(0,1) = 0; C(0,2) = -zc_/GRAVITY;

  Q.resize(4,4);
  Q.setZero();
  Q(0,0) = 1;

  R.resize(1,1);
  R.setZero();
  R(0,0) = 1e-6;

  Eigen::MatrixXd A_t, B_t, C_t, I_t, F_t, K_t;
  B_t.resize(4,1);
  B_t.setZero();
  B_t.block<1,1>(0,0) = C*B;
  B_t.block<3,1>(1,0) = B;
  
  I_t.resize(4,1);
  I_t.setZero();
  I_t(0,0) = 1;

  F_t.resize(4,3);
  F_t.setZero();
  F_t.block<1,3>(0,0) = C*A;
  F_t.block<3,3>(1,0) = A;

  A_t.resize(4,4);
  A_t.setZero();
  A_t.block<4,1>(0,0) = I_t;
  A_t.block<4,3>(0,1) = F_t;

  K_t.resize(4,4);
  K_t.setZero();
  K_t(0,0) =    110.8102;
  K_t(0,1) =   6084.0397;
  K_t(0,2) =   1663.9597;
  K_t(0,3) =      4.2617;
  
  K_t(1,0) = K_t(0,1);
  K_t(1,1) = 341381.4154;
  K_t(1,2) =  93392.3017;
  K_t(1,3) =    246.1489;
  
  K_t(2,0) = K_t(0,2);
  K_t(2,1) = K_t(1,2);
  K_t(2,2) =  25549.8003;
  K_t(2,3) =     67.4240;

  K_t(3,0) = K_t(0,3);
  K_t(3,1) = K_t(1,3);
  K_t(3,2) = K_t(2,3);
  K_t(3,3) =      0.2012;
  
  Eigen::MatrixXd Gi, Gx, Gd, Ac_t, X_t;
  Gi.resize(1,1);
  Gi.setZero();
  Gi = B_t.transpose()*K_t*I_t;
  Gi = Gi/(R + B_t.transpose()*K_t*B_t)(0,0);
  
  Gx.resize(1,3);
  Gx.setZero();
  Gx = B_t.transpose()*K_t*F_t;

  Ac_t.resize(4,4);
  Ac_t.setZero();
  Ac_t = A_t - B_t.transpose()*B_t*K_t*A_t/(R + B_t.transpose()*K_t*B_t)(0,0);

  X_t.resize(4,prev_tick);
  X_t.setZero();
  X_t.block<4,1>(0,0) = -Ac_t.transpose()*K_t*I_t;

  cout << Gi << endl;
  cout << Gx << endl;
  cout << Ac_t << endl;
  cout << X_t.block<4,1>(0,0) << endl;

  Gd.resize(1,prev_tick);
  Gd.setZero();
  Gd(0,0) = -Gi(0,0);

  for(int i = 1; i < prev_tick; i++)
  {
    X_t.block<4,1>(0,i) = Ac_t.transpose()*X_t.block<4,1>(0,i-1);
    Gd.block<1,1>(0,i) = B_t.transpose()*X_t.block<4,1>(0,i-1)/(R + B_t.transpose()*K_t*B_t)(0,0);
  }
  if(tick_index == 0)
    e_jump_graph << Gd << endl;
}

void WalkingController::econom2_task_compliant()
{
  Eigen::Matrix<double, 6, 34> tmp_l_leg_full_jaco_;
  Eigen::Matrix<double, 6, 34> tmp_r_leg_full_jaco_;

  model_.getJacobianMatrix34DoF((DyrosJetModel::EndEffector)0, &tmp_l_leg_full_jaco_);
  model_.getJacobianMatrix34DoF((DyrosJetModel::EndEffector)1, &tmp_r_leg_full_jaco_);

  Eigen::Matrix<double, 12, 34> tmp_contact_full_jaco_;
  tmp_contact_full_jaco_ << tmp_l_leg_full_jaco_, tmp_r_leg_full_jaco_ ;

  Eigen::Matrix<double, 12, 1> tmp_contact_force_;
  tmp_contact_force_ << l_ft_ , r_ft_;

  Eigen::Matrix<double, 34, 34> tmp_full_inertia;
  tmp_full_inertia = model_.getFullInertia();

  Eigen::Matrix<double, 6, 34> tmp_pelvis_full_jaco_;
  model_.getpelvJacobianMatrix34DoF(true, &tmp_pelvis_full_jaco_);

  Eigen::Matrix<double, 6, 6> tmp_pelvis_lambda_inv_;
  tmp_pelvis_lambda_inv_ = tmp_pelvis_full_jaco_*tmp_full_inertia.inverse()*tmp_pelvis_full_jaco_.transpose();

  Eigen::Matrix<double, 6, 6> tmp_pelvis_lambda_;
  tmp_pelvis_lambda_ = tmp_pelvis_lambda_inv_.inverse();

  Eigen::Matrix<double, 12, 1> tmp_contact_force0_;
  tmp_contact_force0_ << -0.14, 14.58, -216.79, 0.76, 8.77, -2.18, 0.33, -14.61, -225.74, -0.14, 9.63, 2.11;

  Eigen::Matrix<double, 6, 1> tmp_F0_;
  tmp_F0_ = tmp_pelvis_lambda_*tmp_pelvis_full_jaco_*tmp_full_inertia.inverse()*tmp_contact_full_jaco_.transpose()*tmp_contact_force0_;

  Eigen::Matrix<double, 6, 1> tmp_F_;
  tmp_F_ = tmp_pelvis_lambda_*tmp_pelvis_full_jaco_*tmp_full_inertia.inverse()*tmp_contact_full_jaco_.transpose()*tmp_contact_force_;
}

void WalkingController::apply_ext_force()
{
  const double ext_force_time = 5.0*hz_;
  const double ext_force_duration = 1.0*hz_;
  
  //mujoco_ext_force_apply_pub = nh_ext_force_.advertise<std_msgs::Float32MultiArray>("/dyros_jet/applied_ext_force",10);
  mujoco_applied_ext_force.data.resize(7);

  const double max_force = 0.0;
  const double z_axis_const = 1;

  mujoco_applied_ext_force.data[0] = 0.0;
  mujoco_applied_ext_force.data[1] = DyrosMath::cubic(tick_index, ext_force_time, ext_force_time + ext_force_duration, 0.0,   max_force, 0.0, 0.0);
  mujoco_applied_ext_force.data[2] = DyrosMath::cubic(tick_index, ext_force_time, ext_force_time + ext_force_duration, 0.0, - 2*max_force, 0.0, 0.0);
  mujoco_applied_ext_force.data[3] = 0.0;
  mujoco_applied_ext_force.data[4] = 0.0;
  mujoco_applied_ext_force.data[5] = 0.0;

  if(tick_index >= ext_force_time + 3*ext_force_duration)
  {
    mujoco_applied_ext_force.data[1] = DyrosMath::cubic(tick_index, ext_force_time + 3*ext_force_duration, ext_force_time + 4*ext_force_duration,   max_force, 0.0, 0.0, 0.0);
    mujoco_applied_ext_force.data[2] = DyrosMath::cubic(tick_index, ext_force_time + 3*ext_force_duration, ext_force_time + 4*ext_force_duration, - 2*max_force, 0.0, 0.0, 0.0);
  }
  mujoco_applied_ext_force.data[6] = 1;

  //e_tmp_graph << collide_ft_(0) << "," << collide_ft_(1) << "," << collide_ft_(2) << "," << mujoco_applied_ext_force.data[1] << "," << mujoco_applied_ext_force.data[2] << endl;
  //cout << collide_ft_(0) << "," << collide_ft_(1) << "," << collide_ft_(2) << endl;
  mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force);
}

}