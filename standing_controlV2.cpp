/*
 * Modified OSC standing controller. Changed OSC structure and assumed fixed indexing for matrix non-zeros for faster computation.
 */


#include <stdio.h>
#include <chrono>
#include <stdlib.h>
#include <unistd.h>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "lowlevelapi.h"
#include "analytical_expressions.hpp"
#include "kin_left_arm.hpp"
#include "kin_right_arm.hpp"
#include "cpptoml/include/cpptoml.h"
//#include "toml.hpp"

// TODO: Move to head file
namespace joint{
  int left_hip_roll = 0;
  int left_hip_yaw = 1;
  int left_hip_pitch = 2;
  int left_knee = 3;
  int left_toe_A = 4;
  int left_toe_B = 5;
  int right_hip_roll = 6;
  int right_hip_yaw = 7;
  int right_hip_pitch = 8;
  int right_knee = 9;
  int right_toe_A = 10;
  int right_toe_B = 11;
  int left_shoulder_roll = 12;
  int left_shoulder_pitch = 13;
  int left_shoulder_yaw = 14;
  int left_elbow = 15;
  int right_shoulder_roll = 16;
  int right_shoulder_pitch = 17;
  int right_shoulder_yaw = 18;
  int right_elbow = 19;

  int left_shin = 0;
  int left_tarsus = 1;
  int left_toe_pitch = 2;
  int left_toe_roll = 3;
  int right_shin = 5;
  int right_tarsus = 6;
  int right_toe_pitch = 7;
  int right_toe_roll = 8;
}

namespace wbc{
  int pel_x = 0;
  int pel_y = 1;
  int pel_z = 2;
  int pel_rotz = 3;
  int pel_roty = 4;
  int pel_rotx = 5;

  int left_hip_roll = 6;
  int left_hip_yaw = 7;
  int left_hip_pitch = 8;
  int left_knee = 9;
  int left_tarsus = 10;
  int left_toe_pitch = 11;
  int left_toe_roll = 12;

  int right_hip_roll = 13;
  int right_hip_yaw = 14;
  int right_hip_pitch = 15;
  int right_knee = 16;
  int right_tarsus = 17;
  int right_toe_pitch = 18;
  int right_toe_roll = 19;
}

static double target_position[] = {
  -0.0462933,
  -0.0265814,
  0.19299,
  -0.3,
  -0.0235182,
  -0.0571617,//left foot
  -0.0125,
  0.18,
  0.33,
  0.83,
  -0.485182,
  0.2371617,//right foot
  -0.3,
  -0.943845,
  0.0,
  0.3633,//left arm
  0.3,
  -0.943845,
  0.0,
  -0.3633,//right arm
};


MatrixXd get_B(VectorXd q);
MatrixXd get_Spring_Jaco();
VectorXd ToEulerAngle(VectorXd q);
MatrixXd get_fric_constraint(double mu);
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

#define NUM_FROST_STATE 28
#define NUM_Dyn_STATE 20
using namespace std;
using namespace std::chrono;
int main(int argc, char* argv[])
{
  
  OsqpEigen::Solver solver;
  int QP_initialized = 0;

  AnalyticalExpressions analytical_expressions;
  VectorXd q(NUM_MOTORS);
  VectorXd dq(NUM_MOTORS);
  VectorXd qj(NUM_JOINTS);
  VectorXd dqj(NUM_JOINTS);

  // joint used in FROST is 28, 3 base position, 3 base orientation, 22 (6 floating base, 12 actuated joints, 4 passive joints). Assuming fixed arm
  VectorXd wb_q(NUM_FROST_STATE);
  VectorXd wb_dq(NUM_FROST_STATE);
  VectorXd pb_q(NUM_Dyn_STATE);
  VectorXd pb_dq(NUM_Dyn_STATE);
  MatrixXd M_fix(20,20);
  int matrix_fixed = 0;
  double soft_start = 1;
  // The publisher address should be changed to the ip address of the robot
  const char* publisher_address = "127.0.0.1";
  llapi_init(publisher_address);

  // Define inputs and outputs (updated each iteration)
  llapi_command_t command = {0};
  llapi_observation_t observation;

  // Connect to robot (need to send commands until the subscriber connects)
  command.apply_command = false;
  while (!llapi_get_observation(&observation)) llapi_send_command(&command);

  // Get local copy of command limits (torque and damping)
  const llapi_limits_t* limits = llapi_get_limits();
  
  // Load Gains from TOML file
  std::shared_ptr<cpptoml::table> config = cpptoml::parse_file("/home/orl/Tianze_WS/Test_Control/src/config_file/osc_robot_config.toml");
  double cpx = config->get_qualified_as<double>("PD-Gains.com_P_gain_x").value_or(0);
  double cpy = config->get_qualified_as<double>("PD-Gains.com_P_gain_y").value_or(0);
  double cpz = config->get_qualified_as<double>("PD-Gains.com_P_gain_z").value_or(0);
  double cprz = config->get_qualified_as<double>("PD-Gains.com_P_gain_rz").value_or(0);
  double cpry = config->get_qualified_as<double>("PD-Gains.com_P_gain_ry").value_or(0);
  double cprx = config->get_qualified_as<double>("PD-Gains.com_P_gain_rx").value_or(0);

  double cdx = config->get_qualified_as<double>("PD-Gains.com_D_gain_x").value_or(0);
  double cdy = config->get_qualified_as<double>("PD-Gains.com_D_gain_y").value_or(0);
  double cdz = config->get_qualified_as<double>("PD-Gains.com_D_gain_z").value_or(0);
  double cdrz = config->get_qualified_as<double>("PD-Gains.com_D_gain_rz").value_or(0);
  double cdry = config->get_qualified_as<double>("PD-Gains.com_D_gain_ry").value_or(0);
  double cdrx = config->get_qualified_as<double>("PD-Gains.com_D_gain_rx").value_or(0);

  double fpx = config->get_qualified_as<double>("PD-Gains.foot_P_x").value_or(0);
  double fpy = config->get_qualified_as<double>("PD-Gains.foot_P_y").value_or(0);
  double fpz = config->get_qualified_as<double>("PD-Gains.foot_P_z").value_or(0);
  double fdx = config->get_qualified_as<double>("PD-Gains.foot_D_x").value_or(0);
  double fdy = config->get_qualified_as<double>("PD-Gains.foot_D_y").value_or(0);
  double fdz = config->get_qualified_as<double>("PD-Gains.foot_D_z").value_or(0);

  double Wcom = config->get_qualified_as<double>("QP-Params.com_W").value_or(0);
  double Wff  = config->get_qualified_as<double>("QP-Params.st_foot_W").value_or(0);
  double Wfb  = config->get_qualified_as<double>("QP-Params.st_foot_W").value_or(0);

  double force_max = config->get_qualified_as<double>("QP-Params.force_max").value_or(0);
  double mu = config->get_qualified_as<double>("QP-Params.mu").value_or(0);

  double arm_P = config->get_qualified_as<double>("PD-Gains.arm_P").value_or(0);

  // Weight Matrix and Gain Vector
  MatrixXd Weight_ToeF = Wff*MatrixXd::Identity(6,6);
  VectorXd KP_ToeF = VectorXd::Zero(6,1);
  VectorXd KD_ToeF = VectorXd::Zero(6,1);
  KP_ToeF << fpx,fpy,fpz,fpx,fpy,fpz;
  KD_ToeF << fdx,fdy,fdz,fdx,fdy,fdz;

  MatrixXd Weight_ToeB = Wfb*MatrixXd::Identity(8,8);
  VectorXd KP_ToeB = KP_ToeF;
  VectorXd KD_ToeB = KD_ToeF;

  MatrixXd Weight_pel = Wcom * MatrixXd::Identity(6,6);
  VectorXd KP_pel = VectorXd::Zero(6,1);
  VectorXd KD_pel = VectorXd::Zero(6,1);
  KP_pel << cpx,cpy,cpz,cprz,cpry,cprx;
  KD_pel << cdx,cdy,cdz,cdrz,cdry,cdrx;

  // QP Bounds
  VectorXd ddq_limit = VectorXd::Zero(20,1); // state acceleration 
  VectorXd u_limit = VectorXd::Zero(12,1);   // torque limits
  VectorXd tor_limit = VectorXd::Zero(4,1);  // generalized force
  VectorXd f_limit_max = VectorXd::Zero(12,1);   // contact force
  VectorXd f_limit_min = VectorXd::Zero(12,1);   // contact force
  VectorXd f_cons_min = VectorXd::Zero(16,1);    // friction constraint limit
  VectorXd qt = VectorXd::Zero(14,1);
  for(int i=0;i<20;i++){
    ddq_limit(i) = OsqpEigen::INFTY;
  }
  for(int i=0;i<16;i++){
    f_cons_min(i) = -OsqpEigen::INFTY;
  }
  for(int i=0;i<14;i++){
    qt(i) = OsqpEigen::INFTY;
  }

  u_limit  << 116.682, 70.1765, 206.928,220.928,35.9759,35.9759,116.682, 70.1765, 206.928,220.928,35.9759,35.9759;
  tor_limit<< OsqpEigen::INFTY,  OsqpEigen::INFTY,  OsqpEigen::INFTY,  OsqpEigen::INFTY;

  // Incorporate damping command into OSC
  double damping_dt = 0.001;
  VectorXd damping(20),D_term(20);;
  damping << VectorXd::Zero(6,1), 66.849, 26.1129, 38.05, 38.05, 0 , 15.5532, 15.5532, 
              66.849, 26.1129, 38.05, 38.05, 0 , 15.5532, 15.5532;
  MatrixXd Dmat = damping.asDiagonal();
  D_term = 0.75 * Dmat * wb_dq.block(0,0,20,1);

  //
  int Vars_Num = 20 + 12 + 4 + 12 + 14;
  int Cons_Num = 20 + 4 + Vars_Num + 16 + 14;
  MatrixXd constraint_full = MatrixXd::Zero(Cons_Num,Vars_Num);
  MatrixXd hessian_full = MatrixXd::Zero(Vars_Num,Vars_Num);
  Eigen::SparseMatrix<double> linearMatrix;
  linearMatrix.resize(Cons_Num,Vars_Num);
  std::vector<Eigen::Triplet<double>> coefficients;            // list of non-zeros coefficients 
  VectorXd counter = VectorXd::Zero(3,1);

  while (1) {
    auto time_program_start = std::chrono::system_clock::now();
    // Update observation
    int return_val = llapi_get_observation(&observation);
    if (return_val < 1) {
      // Error occurred
    } else if (return_val) {
      // New data received
    } else {
      // No new data
    }

    // Get state information
    for (int i = 0; i < NUM_MOTORS; i++){
      q(i) = observation.motor.position[i];
      dq(i) = observation.motor.velocity[i];
    }

    for (int i = 0; i < NUM_JOINTS; i++){
      qj(i) = observation.joint.position[i];
      dqj(i) = observation.joint.velocity[i];
    }
    
    VectorXd pel_pos(3),pel_vel(3),pel_quaternion(4),theta(3),dtheta(3);
    for (int i = 0;i < 3;i++){
        pel_pos(i) = observation.base.translation[i];
        pel_vel(i) = observation.base.linear_velocity[i];
        dtheta(i) = observation.base.angular_velocity[i];
    }
    pel_quaternion(0) = observation.base.orientation.w;
    pel_quaternion(1) = observation.base.orientation.x;
    pel_quaternion(2) = observation.base.orientation.y;
    pel_quaternion(3) = observation.base.orientation.z;
    
    theta = ToEulerAngle(pel_quaternion); // in roll, pitch, yaw order
    MatrixXd OmegaToDtheta = MatrixXd::Zero(3,3);
    OmegaToDtheta << 0 , -sin(theta(2)), cos(theta(2)) * cos(theta(1)), 0, cos(theta(2)), cos(theta(1)) * sin(theta(2)), 1, 0, -sin(theta(1));
    dtheta = OmegaToDtheta * dtheta;

    // get state vector
    wb_q  << pel_pos, theta(2), theta(1), theta(0), q(joint::left_hip_roll),q(joint::left_hip_yaw),q(joint::left_hip_pitch),q(joint::left_knee)
      ,qj(joint::left_tarsus),qj(joint::left_toe_pitch),qj(joint::left_toe_roll),
      q(joint::right_hip_roll),q(joint::right_hip_yaw),q(joint::right_hip_pitch),q(joint::right_knee),
      qj(joint::right_tarsus),qj(joint::right_toe_pitch),qj(joint::right_toe_roll),q(joint::left_shoulder_roll),q(joint::left_shoulder_pitch)
      ,q(joint::left_shoulder_yaw),q(joint::left_elbow),q(joint::right_shoulder_roll),q(joint::right_shoulder_pitch)
      ,q(joint::right_shoulder_yaw),q(joint::right_elbow);

    wb_dq  << pel_vel, dtheta(2), dtheta(1), dtheta(0), dq(joint::left_hip_roll),dq(joint::left_hip_yaw),dq(joint::left_hip_pitch),dq(joint::left_knee)
      ,dqj(joint::left_tarsus),dqj(joint::left_toe_pitch),dqj(joint::left_toe_roll),
      dq(joint::right_hip_roll),dq(joint::right_hip_yaw),dq(joint::right_hip_pitch),dq(joint::right_knee),
      dqj(joint::right_tarsus),dqj(joint::right_toe_pitch),dqj(joint::right_toe_roll),dq(joint::left_shoulder_roll),dq(joint::left_shoulder_pitch)
      ,dq(joint::left_shoulder_yaw),dq(joint::left_elbow),dq(joint::right_shoulder_roll),dq(joint::right_shoulder_pitch)
      ,dq(joint::right_shoulder_yaw),dq(joint::right_elbow);
    

    pb_q << pel_pos, theta(2), theta(1), theta(0), q(joint::left_hip_roll),q(joint::left_hip_yaw),q(joint::left_hip_pitch),q(joint::left_knee)
      ,qj(joint::left_tarsus),qj(joint::left_toe_pitch),qj(joint::left_toe_roll),q(joint::right_hip_roll),q(joint::right_hip_yaw),q(joint::right_hip_pitch)
      ,q(joint::right_knee),qj(joint::right_tarsus),qj(joint::right_toe_pitch),qj(joint::right_toe_roll);

    pb_dq << pel_vel, dtheta(2), dtheta(1), dtheta(0), dq(joint::left_hip_roll),dq(joint::left_hip_yaw),dq(joint::left_hip_pitch),dq(joint::left_knee)
      ,dqj(joint::left_tarsus),dqj(joint::left_toe_pitch),dqj(joint::left_toe_roll),dq(joint::right_hip_roll),dq(joint::right_hip_yaw),dq(joint::right_hip_pitch)
      ,dq(joint::right_knee),dqj(joint::right_tarsus),dqj(joint::right_toe_pitch),dqj(joint::right_toe_roll);
      
    // Compute Dynamics
    // Need to consider full body dynamics in the future. Arm inertia is not trivial
    MatrixXd M = analytical_expressions.InertiaMatrix(pb_q);
    MatrixXd G = analytical_expressions.GravityVector(pb_q);
    //MatrixXd C = analytical_expressions.CoriolisTerm(pb_q,pb_dq);

    // compute end effector position
    VectorXd pelvis_pos = analytical_expressions.p_Pelvis(wb_q);
    
    // toe front kinematics
    VectorXd left_toe_pos = analytical_expressions.p_left_toe_front(wb_q);
    MatrixXd left_toe_jaco  = analytical_expressions.Jp_left_toe_front(wb_q);
    //MatrixXd left_toe_djaco  = analytical_expressions.dJp_left_toe_front(wb_q,wb_dq);
    VectorXd right_toe_pos = analytical_expressions.p_right_toe_front(wb_q);    
    MatrixXd right_toe_jaco = analytical_expressions.Jp_right_toe_front(wb_q);
    //MatrixXd right_toe_djaco = analytical_expressions.dJp_right_toe_front(wb_q,wb_dq);

    // toe back kinematics
    VectorXd left_toe_back_pos = analytical_expressions.p_left_toe_back(wb_q);
    MatrixXd left_toe_back_jaco  = analytical_expressions.Jp_left_toe_back(wb_q);
    //MatrixXd left_toe_back_djaco  = analytical_expressions.dJp_left_toe_back(wb_q,wb_dq);
    VectorXd right_toe_back_pos = analytical_expressions.p_right_toe_back(wb_q);
    MatrixXd right_toe_back_jaco  = analytical_expressions.Jp_right_toe_back(wb_q);
    //MatrixXd right_toe_back_djaco  = analytical_expressions.dJp_right_toe_back(wb_q,wb_dq);

    auto elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    cout << "time used to compute system dyn and kin is: " << elapsed_time.count() << endl;

    // Get fixed arm version
    MatrixXd left_toe_jaco_fa = MatrixXd::Zero(3,20); // fa: fixed arm
    MatrixXd left_toe_back_jaco_fa = MatrixXd::Zero(3,20);
    MatrixXd right_toe_jaco_fa = MatrixXd::Zero(3,20);
    MatrixXd right_toe_back_jaco_fa = MatrixXd::Zero(3,20);

    left_toe_jaco_fa << left_toe_jaco.block(0,0,3,13) , MatrixXd::Zero(3,7);
    left_toe_back_jaco_fa << left_toe_back_jaco.block(0,0,3,13) , MatrixXd::Zero(3,7);
    right_toe_jaco_fa << right_toe_jaco.block(0,0,3,6) , MatrixXd::Zero(3,7), right_toe_jaco.block(0,13,3,7);
    right_toe_back_jaco_fa << right_toe_back_jaco.block(0,0,3,6) , MatrixXd::Zero(3,7), right_toe_back_jaco.block(0,13,3,7);

    // End effector velocity
    VectorXd  left_toe_vel = left_toe_jaco * wb_dq;
    VectorXd  right_toe_vel = right_toe_jaco * wb_dq;

    // Compute Desired Foot Traj
    VectorXd left_toe_pos_ref = VectorXd::Zero(3,1);
    VectorXd left_toe_vel_ref = VectorXd::Zero(3,1);
    VectorXd left_toe_acc_ref = VectorXd::Zero(3,1);

    VectorXd right_toe_pos_ref = VectorXd::Zero(3,1);
    VectorXd right_toe_vel_ref = VectorXd::Zero(3,1);
    VectorXd right_toe_acc_ref = VectorXd::Zero(3,1);
    
    left_toe_pos_ref(2) = 0.3 + 0.2 * sin(3.14*soft_start/200);
    left_toe_vel_ref(2) = 0.2 * 3.14/200*cos(3.14*soft_start/200);
    left_toe_acc_ref(2) = -0.2 * 3.14/200*3.14/200*sin(3.14*soft_start/200);

    right_toe_pos_ref(2) = 0.3 + 0.2 * sin(3.14*soft_start/200+3.14);
    right_toe_vel_ref(2) = 0.2 * 3.14/200*cos(3.14*soft_start/200+3.14);
    right_toe_acc_ref(2) = -0.2 * 3.14/200*3.14/200*sin(3.14*soft_start/200+3.14);
    
    //left_toe_pos_ref(2) = -0.7;
    //right_toe_pos_ref(2) = -0.7;

    // Toe weights and Gains
    // compute target position acc
    VectorXd des_acc = VectorXd::Zero(6,1);
    des_acc << -KP_ToeF(0) * (left_toe_pos(0) - 0.08) - KD_ToeF(0) * (left_toe_vel(0) - 0),
               -KP_ToeF(1) * (left_toe_pos(1) - 0.2) - KD_ToeF(1) * (left_toe_vel(1) - 0), 
               -KP_ToeF(2) * (left_toe_pos(2) - left_toe_pos_ref(2)) - KD_ToeF(2) * (left_toe_vel(2) - left_toe_vel_ref(2)) + left_toe_acc_ref(2),
               -KP_ToeF(3) * (right_toe_pos(0) - 0.08) - KD_ToeF(3) * (right_toe_vel(0) - 0),
               -KP_ToeF(4) * (right_toe_pos(1) + 0.2) - KD_ToeF(4) * (right_toe_vel(1) - 0), 
               -KP_ToeF(5) * (right_toe_pos(2) - right_toe_pos_ref(2)) - KD_ToeF(5) * (right_toe_vel(2) - right_toe_vel_ref(2)) + right_toe_acc_ref(2);


    // Control another contact point on toe back to enable foot rotation control


    VectorXd left_toe_rot  = VectorXd::Zero(4,1);
    VectorXd left_toe_drot = VectorXd::Zero(4,1);
    MatrixXd left_toe_rot_jaco_fa = MatrixXd::Zero(4,20);
    VectorXd right_toe_rot  = VectorXd::Zero(4,1);
    VectorXd right_toe_drot = VectorXd::Zero(4,1);
    MatrixXd right_toe_rot_jaco_fa = MatrixXd::Zero(4,20);

    // Currrently, need to compute the yaw,roll rotation of the leg explicitly
    // Seems to be caused by the toe yaw DOF in Mujoco
    left_toe_rot.block(0,0,3,1) = left_toe_back_pos;
    left_toe_drot.block(0,0,3,1) = left_toe_back_jaco * wb_dq;
    left_toe_rot(3) =  q(joint::left_hip_yaw); 
    left_toe_drot(3) = dq(joint::left_hip_yaw);
    left_toe_rot_jaco_fa.block(0,0,3,20) = left_toe_back_jaco_fa;
    left_toe_rot_jaco_fa(3,wbc::left_hip_yaw) = 1;

    right_toe_rot.block(0,0,3,1) = right_toe_back_pos;
    right_toe_drot.block(0,0,3,1) = right_toe_back_jaco * wb_dq;
    right_toe_rot(3) =  q(joint::right_hip_yaw); 
    right_toe_drot(3) = dq(joint::right_hip_yaw);
    right_toe_rot_jaco_fa.block(0,0,3,20) = right_toe_back_jaco_fa;
    right_toe_rot_jaco_fa(3,wbc::right_hip_yaw) = 1;


    VectorXd des_acc_toe = VectorXd::Zero(8,1);
    // Currently, need the forth dimension to control leg yaw rotation
    des_acc_toe << -KP_ToeB(0) * (left_toe_rot(0) + 0.15) - KD_ToeB(0) * (left_toe_drot(0) - 0),
                   -KP_ToeB(1) * (left_toe_rot(1) - 0.2) - KD_ToeB(1) * (left_toe_drot(1) - 0),
                   -KP_ToeB(2) * (left_toe_rot(2) - left_toe_pos_ref(2)) - KD_ToeB(2) * (left_toe_drot(2) - left_toe_vel_ref(2)) + left_toe_acc_ref(2),
                   -KP_ToeB(2) * (left_toe_rot(3) - 0) - KD_ToeB(2) * (left_toe_drot(3) - 0),
                   -KP_ToeB(3) * (right_toe_rot(0) - 0.15) - KD_ToeB(3) * (right_toe_drot(0) - 0),
                   -KP_ToeB(4) * (right_toe_rot(1) + 0.2) - KD_ToeB(4) * (right_toe_drot(1) - 0),
                   -KP_ToeB(5) * (right_toe_rot(2) - right_toe_pos_ref(2)) - KD_ToeB(5) * (right_toe_drot(2) - right_toe_vel_ref(2)) + right_toe_acc_ref(2),
                   -KP_ToeB(5) * (right_toe_rot(3) - 0) - KD_ToeB(5) * (right_toe_drot(3) - 0);
    
    // Compute JdotV. TODO: check if this is needed for control
    des_acc = VectorXd::Zero(6,1);
    des_acc_toe.block(0,0,3,1) = VectorXd::Zero(3,1); // for stance foot, desired acc is 0
    des_acc_toe.block(4,0,3,1) = VectorXd::Zero(3,1);

    //.block(0,0,3,1) -= left_toe_back_djaco * wb_dq; 
    //des_acc_toe.block(4,0,3,1) -= right_toe_back_djaco * wb_dq; 

    //des_acc.block(0,0,3,1) -= left_toe_djaco * wb_dq;
    //des_acc.block(3,0,3,1) -= right_toe_djaco * wb_dq; 

    // B matrix
    MatrixXd B = get_B(wb_q);

    // Spring Jacobian
    MatrixXd Spring_Jaco = get_Spring_Jaco();

    // For pelvis control in standing OSC
    VectorXd des_acc_pel = VectorXd::Zero(6,1);
    MatrixXd pel_jaco = MatrixXd::Zero(6,20);
    pel_jaco.block(0,0,6,6) = MatrixXd::Identity(6,6);
    des_acc_pel << -KP_pel(0) * (pel_pos(0) - 0.05) - KD_pel(0) * (pel_vel(0) - 0),
                   -KP_pel(1) * (pel_pos(1) - 0) - KD_pel(1) * (pel_vel(1) - 0),
                   -KP_pel(2) * (pel_pos(2) - 1) - KD_pel(2) * (pel_vel(2) - 0),
                   -KP_pel(3) * (theta(2) - 0) - KD_pel(3) * (dtheta(2) - 0),
                   -KP_pel(4) * (theta(1) - 0) - KD_pel(4) * (dtheta(1) - 0),
                   -KP_pel(5) * (theta(0) - 0) - KD_pel(5) * (dtheta(0) - 0);

    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    cout << "time used to compute system dyn and kin + acc + QP Form: " << elapsed_time.count() << endl;
    // Solve OSC QP

    solver.data()->setNumberOfVariables(Vars_Num);
    solver.data()->setNumberOfConstraints(Cons_Num);
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient(Vars_Num,1);
 
    Eigen::VectorXd lowerBound(Cons_Num,1);
    Eigen::VectorXd upperBound(Cons_Num,1);
    Eigen::VectorXd QPSolution;

    hessian.resize(Vars_Num,Vars_Num);


    // Construct QP Matrix
    // gradient
    /*VectorXd ddp_grad = -2 * (left_toe_jaco_fa.block(0,6,3,14)).transpose() * Weight_ToeF.block(0,0,3,3) * des_acc.block(0,0,3,1)
                    -2 * (left_toe_rot_jaco_fa.block(0,6,4,14)).transpose() * Weight_ToeB.block(0,0,4,4) * des_acc_toe.block(0,0,4,1)
                    -2 * (right_toe_jaco_fa.block(0,6,3,14)).transpose() * Weight_ToeF.block(3,3,3,3) * des_acc.block(3,0,3,1)
                    -2 * (right_toe_rot_jaco_fa.block(0,6,4,14)).transpose() * Weight_ToeB.block(4,4,4,4) * des_acc_toe.block(4,0,4,1);
    */
    /*VectorXd ddp_grad = -(left_toe_jaco_fa).transpose() * Weight_ToeF.block(0,0,3,3) * des_acc.block(0,0,3,1)
                    -(left_toe_rot_jaco_fa).transpose() * Weight_ToeB.block(0,0,4,4) * des_acc_toe.block(0,0,4,1)
                    -(right_toe_jaco_fa).transpose() * Weight_ToeF.block(3,3,3,3) * des_acc.block(3,0,3,1)
                    -(right_toe_rot_jaco_fa).transpose() * Weight_ToeB.block(4,4,4,4) * des_acc_toe.block(4,0,4,1)
                    -pel_jaco.transpose() * Weight_pel * des_acc_pel;
    gradient << ddp_grad, VectorXd::Zero(28,1);*/

    //VectorXd ddp_grad(20,1);
    //ddp_grad << -des_acc.block(0,0,3,1),des_acc_toe.block(0,0,4,1),des_acc.block(3,0,3,1),des_acc_toe.block(4,0,4,1),
    gradient << -Weight_pel * des_acc_pel, VectorXd::Zero(42,1),-Weight_ToeF.block(0,0,3,3) * des_acc.block(0,0,3,1),
                -Weight_ToeB.block(0,0,4,4) * des_acc_toe.block(0,0,4,1),
                -Weight_ToeF.block(3,3,3,3) * des_acc.block(3,0,3,1),
                -Weight_ToeB.block(4,4,4,4) * des_acc_toe.block(4,0,4,1);
                

    // upper and lower bounds
    for(int i=0;i<12;i++){
      f_limit_max(i) = force_max * mu;
      f_limit_min(i) = -force_max * mu;
    }
    f_limit_max(2) = force_max;
    f_limit_max(5) = force_max;
    f_limit_max(8) = force_max;
    f_limit_max(11) = force_max;
    f_limit_min(2) = 0;
    f_limit_min(5) = 0;
    f_limit_min(8) = 0;
    f_limit_min(11) = 0;
    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    cout << "compute vectors: " << elapsed_time.count() << endl;

    // Concatenate bound vector
    lowerBound << -G + D_term, VectorXd::Zero(4,1) , -ddq_limit, -u_limit, -tor_limit, f_limit_min, -qt,f_cons_min,VectorXd::Zero(14,1);
    upperBound << -G + D_term, VectorXd::Zero(4,1) ,  ddq_limit,  u_limit, tor_limit , f_limit_max, qt, VectorXd::Zero(16,1),VectorXd::Zero(14,1);
    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    cout << "compute vectors: " << elapsed_time.count() << endl;
    // Hessian Matrix
    /*hessian_full.block(0,0,20,20) = 1*(left_toe_jaco_fa).transpose() * Weight_ToeF.block(0,0,3,3) * left_toe_jaco_fa
                                   +1*(left_toe_rot_jaco_fa).transpose() * Weight_ToeB.block(0,0,4,4) * left_toe_rot_jaco_fa
                                   +1*(right_toe_jaco_fa).transpose() * Weight_ToeF.block(3,3,3,3) * right_toe_jaco_fa
                                   +1*(right_toe_rot_jaco_fa).transpose() * Weight_ToeB.block(4,4,4,4) * right_toe_rot_jaco_fa
                                   +1*pel_jaco.transpose() * Weight_pel * pel_jaco;*/
    hessian_full.block(0,0,6,6) = Weight_pel;
    hessian_full.block(48,48,3,3) = Weight_ToeF.block(0,0,3,3);
    hessian_full.block(51,51,4,4) = Weight_ToeB.block(0,0,4,4);
    hessian_full.block(55,55,3,3) = Weight_ToeF.block(3,3,3,3);
    hessian_full.block(58,58,4,4) = Weight_ToeB.block(4,4,4,4);
    for(int i = 0;i<hessian_full.rows();i++){
      if(i<6||i>48){
        hessian.insert(i,i) = hessian_full(i,i);
      }
    }

    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    cout << "assign value to hessian matrix: " << elapsed_time.count() << endl;
    // Works like reflected inertia. Otherwise, need to have large P gains. But is is modeled in the mujoco sim?
    // Important Question??? seems not helping in standing phase. Maybe it is because the cross terms are not considered in
    // the fixed base controller

    // Constraint Matrix
    if(QP_initialized == 0){
      constraint_full.block(0,0,20,20) = M - damping_dt * Dmat;
      constraint_full.block(0,20,20,12) = -B;
      constraint_full.block(0,32,20,4) = -(Spring_Jaco).transpose();
      constraint_full.block(0,36,20,3) = -left_toe_jaco_fa.transpose();
      constraint_full.block(0,39,20,3) = -left_toe_back_jaco_fa.transpose();
      constraint_full.block(0,42,20,3) = -right_toe_jaco_fa.transpose();
      constraint_full.block(0,45,20,3) = -right_toe_back_jaco_fa.transpose();
      constraint_full.block(20,0,4,20) = Spring_Jaco;
      constraint_full.block(24,0,Vars_Num,Vars_Num) = MatrixXd::Identity(Vars_Num,Vars_Num);
      constraint_full.block(24+Vars_Num,Vars_Num-12,16,12) = get_fric_constraint(mu);
      constraint_full.block(40+Vars_Num,0,3,20) = left_toe_jaco_fa;
      constraint_full.block(43+Vars_Num,0,4,20) = left_toe_rot_jaco_fa;
      constraint_full.block(47+Vars_Num,0,3,20) = right_toe_jaco_fa;
      constraint_full.block(50+Vars_Num,0,4,20) = right_toe_rot_jaco_fa;
      constraint_full.block(40+Vars_Num,Vars_Num-14,14,14) = -MatrixXd::Identity(14,14);

      elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
      cout << "formulate constraint matrix: " << elapsed_time.count() << endl;
      for(int i = 0;i<constraint_full.cols();i++){
        for(int j = 0;j<constraint_full.rows();j++){
          if(constraint_full(j,i) != 0){
            coefficients.push_back(Eigen::Triplet<double>(j,i,constraint_full(j,i)));
            linearMatrix.insert(j,i) = constraint_full(j,i);
          }
        }
      }
    }
    else{
      constraint_full.block(0,0,20,20) = M - damping_dt * Dmat;
      //constraint_full.block(0,20,20,12) = -B;
      //constraint_full.block(0,32,20,4) = -(Spring_Jaco).transpose();
      constraint_full.block(0,36,20,3) = -left_toe_jaco_fa.transpose();
      constraint_full.block(0,39,20,3) = -left_toe_back_jaco_fa.transpose();
      constraint_full.block(0,42,20,3) = -right_toe_jaco_fa.transpose();
      constraint_full.block(0,45,20,3) = -right_toe_back_jaco_fa.transpose();
      //constraint_full.block(20,0,4,20) = Spring_Jaco;
      //constraint_full.block(24,0,Vars_Num,Vars_Num) = MatrixXd::Identity(Vars_Num,Vars_Num);
      //constraint_full.block(24+Vars_Num,Vars_Num-12,16,12) = get_fric_constraint(mu);
      constraint_full.block(40+Vars_Num,0,3,20) = left_toe_jaco_fa;
      constraint_full.block(43+Vars_Num,0,4,20) = left_toe_rot_jaco_fa;
      constraint_full.block(47+Vars_Num,0,3,20) = right_toe_jaco_fa;
      constraint_full.block(50+Vars_Num,0,4,20) = right_toe_rot_jaco_fa;
      //constraint_full.block(40+Vars_Num,Vars_Num-14,14,14) = -MatrixXd::Identity(14,14);
      
      int coeff_iter = 0;
      for (int k=0; k<linearMatrix.outerSize(); ++k){
        for (SparseMatrix<double>::InnerIterator it(linearMatrix,k); it; ++it){
          it.valueRef() = constraint_full(coefficients[coeff_iter].row(),coefficients[coeff_iter].col());
          coeff_iter++;
        }
      }
      
      /*
      elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
      cout << "formulate constraint matrix: " << elapsed_time.count() << endl;
      std::vector<Eigen::Triplet<double>> coefficients1;            // list of non-zeros coefficients 
      for(auto T:coefficients){
        coefficients1.push_back(Eigen::Triplet<double>(T.row(),T.col(),constraint_full(T.row(),T.col())));
      }
      elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
      cout << "formulate constraint matrix: " << elapsed_time.count() << endl;

      linearMatrix.setFromTriplets(coefficients1.begin(), coefficients1.end());*/
      /*
      for(int i = 0;i<constraint_full.rows();i++){
        if(i<20){
          for(int j = 0;j<48;j++){
              if(constraint_full(i,j) != 0){
                //linearMatrix.insert(i,j) = constraint_full(i,j);
              }
          }
        }
        if(i>40+Vars_Num){
          for(int j = 0;j<20;j++){
              if(constraint_full(i,j) != 0){
                //linearMatrix.insert(i,j) = constraint_full(i,j);
              }
          }
          for(int j = 0;j>Vars_Num-14;j++){
              if(constraint_full(i,j) != 0){
                //linearMatrix.insert(i,j) = constraint_full(i,j);
              }
          }
        }

      }*/
    }
    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    cout << "set up sparse constraint: " << elapsed_time.count() << endl;
    if(QP_initialized == 0){
      solver.settings() ->setPrimalInfeasibilityTolerance(1e-4);
      solver.settings() ->setCheckTermination(10);
      solver.settings() ->setAbsoluteTolerance(1e-4);
      solver.settings() ->setRelativeTolerance(1e-4);
      solver.data()->setHessianMatrix(hessian);
      solver.data()->setGradient(gradient);
      solver.data()->setLinearConstraintsMatrix(linearMatrix);
      solver.data()->setLowerBound(lowerBound);
      solver.data()->setUpperBound(upperBound);
      solver.initSolver();
      QP_initialized = 1;

    }
    else{
      solver.updateGradient(gradient);
      solver.updateHessianMatrix(hessian);
      solver.updateLinearConstraintsMatrix(linearMatrix);
      solver.updateBounds(lowerBound,upperBound);
    }

    solver.solveProblem();
    QPSolution = solver.getSolution();
    VectorXd torque = VectorXd::Zero(12,1);
    for(int i = 0;i<12;i++)
      torque(i) = QPSolution(20+i);

    // OSC on leg, PD on arm
    elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
    cout << "time used to compute system dyn and kin + QP formulation + Solving: " << elapsed_time.count() << endl;
    if(elapsed_time.count()<1200) counter(0)+=1;
    else counter(1)+=1;
    counter(2) += elapsed_time.count();
    cout << "solving time summary" << endl;
    cout << counter << endl;
    cout << "average solving time: " << counter(2)/(counter(0) + counter(1)) << endl;

    // Integrate osc ddq to find velocity command
    VectorXd wb_dq_next = VectorXd::Zero(12,1);
    for(int i=0;i<12;i++){
      if(i<4) 
        wb_dq_next(i) = wb_dq(6+i) + damping_dt * QPSolution(6+i); // skip passive joint tarsus
      else if(i<10)
        wb_dq_next(i) = wb_dq(7+i) + damping_dt * QPSolution(7+i);
      else
        wb_dq_next(i) = wb_dq(8+i) + damping_dt * QPSolution(8+i);
    }

    // arm control, trial implementation. Incorporate to analytical_expressions class in the future
    VectorXd ql = VectorXd::Zero(10,1);
    VectorXd p_lh = VectorXd::Zero(3,1);
    MatrixXd J_lh = MatrixXd::Zero(3,10);
    VectorXd p_lh_ref = VectorXd::Zero(3,1);
    VectorXd p_lh_err = VectorXd::Zero(3,1);

    VectorXd qr = VectorXd::Zero(10,1);
    VectorXd p_rh = VectorXd::Zero(3,1);
    MatrixXd J_rh = MatrixXd::Zero(3,10);
    VectorXd p_rh_ref = VectorXd::Zero(3,1);
    VectorXd p_rh_err = VectorXd::Zero(3,1);

    int period = 1000;
    soft_start++;
    if(soft_start>period){
      soft_start = 0;
    }

    if(soft_start<period/2){
      p_lh_ref << 0.2, 0.2 - .1 * cos(soft_start/period*2*3.14),1.2 - .2 * sin(soft_start/period*2*3.14);
      p_rh_ref << 0.2,-0.2 + .1 * cos(soft_start/period*2*3.14),1.2 - .2 * sin(soft_start/period*2*3.14);
    }
    else{
      p_lh_ref << 0.2, 0.2 - .1 * cos(soft_start/period*2*3.14),1.2 + .2 * sin(soft_start/period*2*3.14);
      p_rh_ref << 0.2,-0.2 + .1 * cos(soft_start/period*2*3.14),1.2 + .2 * sin(soft_start/period*2*3.14);
    }

    // initial q
    for(int i = 0;i<6;i++){
      ql(i) = wb_q(i);
      qr(i) = wb_q(i);
    }
    ql.block(6,0,4,1) = wb_q.block(20,0,4,1);
    qr.block(6,0,4,1) = wb_q.block(24,0,4,1);
    kin_left_arm(ql.data(),p_lh.data(), J_lh.data());
    kin_right_arm(qr.data(),p_rh.data(), J_rh.data());
    J_lh.block(0,0,3,7) = MatrixXd::Zero(3,7); // base is fixed for arm IK
    J_rh.block(0,0,3,7) = MatrixXd::Zero(3,7); // base is fixed for arm IK

    // left arm IK
    double error;
    error = (p_lh - p_lh_ref).norm();
    double iter = 0;
    while(error >0.01 && iter<5){
      // solve for new joint
      ql += J_lh.colPivHouseholderQr().solve(p_lh_ref - p_lh);
      // Clip joints
      //ql(6) = max(min(ql(6),deg2rad(75)),deg2rad(-75));
      ql(6) = -0.3;
      ql(7) = max(min(ql(7),deg2rad(145)),deg2rad(-145));
      ql(8) = max(min(ql(8),deg2rad(100)),deg2rad(-100));
      ql(9) = max(min(ql(9),deg2rad(77.5)),deg2rad(-77.5));
      // Evaluate new pos
      kin_left_arm(ql.data(),p_lh.data(), J_lh.data());
      J_lh.block(0,0,3,7) = MatrixXd::Zero(3,7); // base is fixed for arm IK
      error = (p_lh - p_lh_ref).norm();
      iter++;
    }
    target_position[12] = ql(6);
    target_position[13] = ql(7);
    target_position[14] = ql(8);
    target_position[15] = ql(9); 

    // right arm IK
    error = (p_rh - p_rh_ref).norm();
    iter = 0;
    while(error >0.01 && iter<5){
      qr += J_rh.colPivHouseholderQr().solve(p_rh_ref - p_rh);
      // Clip joints
      //qr(6) = max(min(qr(6),deg2rad(75)),deg2rad(-75));
      qr(6) = 0.3;
      qr(7) = max(min(qr(7),deg2rad(145)),deg2rad(-145));
      qr(8) = max(min(qr(8),deg2rad(100)),deg2rad(-100));
      qr(9) = max(min(qr(9),deg2rad(77.5)),deg2rad(-77.5));
      // Evaluate new pos
      kin_right_arm(qr.data(),p_rh.data(), J_rh.data());
      J_rh.block(0,0,3,7) = MatrixXd::Zero(3,7); // base is fixed for arm IK
      error = (p_rh - p_rh_ref).norm();
      iter++;
    }
    target_position[16] = qr(6);
    target_position[17] = qr(7);
    target_position[18] = qr(8);
    target_position[19] = qr(9); 

 /*   cout << "arm" << std::fixed << std::setprecision(2) << p_lh << endl;
    for(int i =0;i<3;i++){
      for(int j=0;j<10;j++){
        cout << J_lh(i,j) << " ";
      }
      cout << endl;
    }
*/
    
   elapsed_time = duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - time_program_start);
   cout << "time used to compute system dyn and kin + QP formulation + Solving + Arm IK: " << elapsed_time.count() << endl;
    for (int i = 0; i < NUM_MOTORS; ++i) {
      if(i>=12){
        command.motors[i].torque =
          arm_P * (target_position[i] - observation.motor.position[i]);
          command.motors[i].velocity = 0;
          command.motors[i].damping = 0.75 * limits->damping_limit[i];
      }
      else{
        command.motors[i].torque = torque(i);
        command.motors[i].velocity = wb_dq_next(i);
        command.motors[i].damping = 0.75 * limits->damping_limit[i];
      }
      
    }
    command.fallback_opmode = Locomotion; // Useful for simulation
    command.apply_command = true;
    llapi_send_command(&command);

    // Check if llapi has become disconnected
    if (!llapi_connected()) {
      // Handle error case. You don't need to re-initialize subscriber
      // Calling llapi_send_command will keep low level api open
    }

    // Sleep to keep to a reasonable update rate
    //usleep(2000);
  }
}

MatrixXd get_Spring_Jaco(){
  MatrixXd Spring_Jaco = MatrixXd::Zero(4,20);
  Spring_Jaco(0,wbc::left_knee) = 1;
  Spring_Jaco(0,wbc::left_tarsus) = 1; 
  Spring_Jaco(1,wbc::left_toe_pitch) = 0; 
  Spring_Jaco(1,wbc::left_toe_roll) = 0; 

  Spring_Jaco(2,wbc::right_knee) = 1;
  Spring_Jaco(2,wbc::right_tarsus) = 1; 
  Spring_Jaco(3,wbc::right_toe_pitch) = 0; 
  Spring_Jaco(3,wbc::right_toe_roll) = 0; 

  return Spring_Jaco;
}

MatrixXd get_B(VectorXd q){
  // map: motor torque to joint torque
  MatrixXd B = MatrixXd::Zero(20,12);
  VectorXd a,b,c,d;
  a.resize(6);
  b.resize(6);
  c.resize(6);
  d.resize(6);
  a << 0.003061, -0.9412, 0.2812, -0.1121, 0.1288, 0.06276; // higher order approximation?
  b << -0.003154, 0.9416, 0.2848, 0.1133, 0.1315, -0.06146;
  c << -0.003061, -0.9412, 0.2812, 0.1121, -0.1288, -0.06276;
  d << 0.003154, 0.9416, 0.2848, -0.1133, -0.1315, 0.06146;

  VectorXd left_toe_j = VectorXd::Zero(2,1);
  MatrixXd left_J = MatrixXd::Zero(2,2);
  left_toe_j << q(11) , q(12);

  left_J(0,0) = a(1) + 2*a(3) * left_toe_j(0) + a(4) * left_toe_j(1);
  left_J(0,1) = a(2) + 2*a(5) * left_toe_j(1) + a(4) * left_toe_j(0); 
  left_J(1,0) = b(1) + 2*b(3) * left_toe_j(0) + b(4) * left_toe_j(1);
  left_J(1,1) = b(2) + 2*b(5) * left_toe_j(1) + b(4) * left_toe_j(0);

  B(wbc::left_hip_roll,0) = 1;
  B(wbc::left_hip_yaw,1) = 1;
  B(wbc::left_hip_pitch,2) = 1;
  B(wbc::left_knee,3) = 1;
  //B(wbc::left_toe_pitch,4) = 1;
  //B(wbc::left_toe_roll,5) = 1;

  B(wbc::left_toe_pitch,4) = left_J(0,0);
  B(wbc::left_toe_pitch,5) = left_J(1,0);
  B(wbc::left_toe_roll,4) = left_J(0,1);
  B(wbc::left_toe_roll,5) = left_J(1,1);

  VectorXd right_toe_j = VectorXd::Zero(2,1);
  MatrixXd right_J = MatrixXd::Zero(2,2);
  right_toe_j << q(18) , q(19);

  right_J(0,0) = c(1) + 2*c(3) * right_toe_j(0) + c(4) * right_toe_j(1);
  right_J(0,1) = c(2) + 2*c(5) * right_toe_j(1) + c(4) * right_toe_j(0); 
  right_J(1,0) = d(1) + 2*d(3) * right_toe_j(0) + d(4) * right_toe_j(1);
  right_J(1,1) = d(2) + 2*d(5) * right_toe_j(1) + d(4) * right_toe_j(0);

  B(wbc::right_hip_roll,6) = 1;
  B(wbc::right_hip_yaw,7) = 1;
  B(wbc::right_hip_pitch,8) = 1;
  B(wbc::right_knee,9) = 1;
  //B(wbc::right_toe_pitch,10) = 1;
  //B(wbc::right_toe_roll,11) = 1;

  B(wbc::right_toe_pitch,10) = right_J(0,0);
  B(wbc::right_toe_pitch,11) = right_J(1,0);
  B(wbc::right_toe_roll,10) = right_J(0,1);
  B(wbc::right_toe_roll,11) = right_J(1,1);
  return B;
}

MatrixXd get_fric_constraint(double mu){
  MatrixXd fric_cons = MatrixXd::Zero(16,12);
  MatrixXd fric_sub  = MatrixXd::Zero(4,3);
  fric_sub << 1,1,-mu,1,-1,-mu,-1,1,-mu,-1,-1,-mu;
  for(int i=0;i<4;i++){
    fric_cons.block(4*i,3*i,4,3) = fric_sub;
  }
  return fric_cons;
}
VectorXd ToEulerAngle(VectorXd q){
    VectorXd theta = VectorXd::Zero(3,1);
    double w,x,y,z;
    w = q(0);
    x = q(1);
    y = q(2);
    z = q(3);

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    theta(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
      theta(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
      theta(1) = std::asin(sinp);
    }
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    theta(2) = std::atan2(siny_cosp, cosy_cosp);

    return theta;
}