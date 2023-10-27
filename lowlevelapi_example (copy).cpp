/*
 * Copyright 2021 Agility Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "lowlevelapi.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include "analytical_expressions.hpp"

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
  0.943845,
  0.0,
  0.3633,//left arm
  0.3,
  -0.943845,
  0.0,
  -0.3633,//right arm
};

MatrixXd get_B();
MatrixXd get_Spring_Jaco();
#define NUM_FROST_STATE 28
#define NUM_Dyn_STATE 20
using namespace std;
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

  while (1) {
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

    // get state vector
    wb_q  << 0,0,0,0,0,0, q(joint::left_hip_roll),q(joint::left_hip_yaw),q(joint::left_hip_pitch),q(joint::left_knee)
      ,qj(joint::left_tarsus),qj(joint::left_toe_pitch),qj(joint::left_toe_roll),q(joint::left_shoulder_roll),q(joint::left_shoulder_pitch)
      ,q(joint::left_shoulder_yaw),q(joint::left_elbow),
      q(joint::right_hip_roll),q(joint::right_hip_yaw),q(joint::right_hip_pitch),q(joint::right_knee),
      qj(joint::right_tarsus),qj(joint::right_toe_pitch),qj(joint::right_toe_roll),q(joint::right_shoulder_roll),q(joint::right_shoulder_pitch)
      ,q(joint::right_shoulder_yaw),q(joint::right_elbow);

    wb_dq  << 0,0,0,0,0,0, dq(joint::left_hip_roll),dq(joint::left_hip_yaw),dq(joint::left_hip_pitch),dq(joint::left_knee)
      ,dqj(joint::left_tarsus),dqj(joint::left_toe_pitch),dqj(joint::left_toe_roll),dq(joint::left_shoulder_roll),dq(joint::left_shoulder_pitch)
      ,dq(joint::left_shoulder_yaw),dq(joint::left_elbow),
      dq(joint::right_hip_roll),dq(joint::right_hip_yaw),dq(joint::right_hip_pitch),dq(joint::right_knee),
      dqj(joint::right_tarsus),dqj(joint::right_toe_pitch),dqj(joint::right_toe_roll),dq(joint::right_shoulder_roll),dq(joint::right_shoulder_pitch)
      ,dq(joint::right_shoulder_yaw),dq(joint::right_elbow);
    

    pb_q << VectorXd::Zero(6), q(joint::left_hip_roll),q(joint::left_hip_yaw),q(joint::left_hip_pitch),q(joint::left_knee)
      ,qj(joint::left_tarsus),q(joint::left_toe_A),q(joint::left_toe_B),q(joint::right_hip_roll),q(joint::right_hip_yaw),q(joint::right_hip_pitch)
      ,q(joint::right_knee),qj(joint::right_tarsus),qj(joint::right_toe_pitch),qj(joint::right_toe_roll);

    pb_dq << VectorXd::Zero(6), dq(joint::left_hip_roll),dq(joint::left_hip_yaw),dq(joint::left_hip_pitch),dq(joint::left_knee)
      ,dqj(joint::left_tarsus),dq(joint::left_toe_A),dq(joint::left_toe_B),dq(joint::right_hip_roll),dq(joint::right_hip_yaw),dq(joint::right_hip_pitch)
      ,dq(joint::right_knee),dqj(joint::right_tarsus),dqj(joint::right_toe_pitch),dqj(joint::right_toe_roll);

    // Compute Dynamics
    MatrixXd M = analytical_expressions.InertiaMatrix(pb_q);
    MatrixXd G = analytical_expressions.GravityVector(pb_q);
    MatrixXd C = analytical_expressions.CoriolisTerm(pb_q,pb_dq);

    // compute end effector position
    VectorXd pelvis_pos = analytical_expressions.p_Pelvis(wb_q);

    VectorXd left_toe_pos = analytical_expressions.p_left_toe_front(wb_q);
    MatrixXd left_toe_jaco  = analytical_expressions.Jp_left_toe_front(wb_q);

    VectorXd right_toe_pos = analytical_expressions.p_right_toe_front(wb_q);    
    MatrixXd right_toe_jaco = analytical_expressions.Jp_right_toe_front(wb_q);
    
    MatrixXd left_toe_jaco_fa = MatrixXd::Zero(3,20); // fa: fixed arm
    left_toe_jaco_fa << left_toe_jaco.block(0,0,3,13) , MatrixXd::Zero(3,7);

    MatrixXd right_toe_jaco_fa = MatrixXd::Zero(3,20);
    right_toe_jaco_fa << right_toe_jaco.block(0,0,3,6) , MatrixXd::Zero(3,7), right_toe_jaco.block(0,17,37);

    VectorXd  left_toe_vel = left_toe_jaco * wb_dq;
    VectorXd  right_toe_vel = right_toe_jaco * wb_dq;

    // Weights and Gains
    MatrixXd Weight = MatrixXd::Identity(3,3);
    VectorXd KP = VectorXd::Zero(3,1);
    VectorXd KD = VectorXd::Zero(3,1);
    KP << 1225,1225,1225;
    KD << 5,5,5;

    double z_des = 0.7 + 0.2 * sin(3.14*soft_start/1000);
    double dz_des = 0.2 * 3.14/1000*cos(3.14*soft_start/1000);
    double ddz_des = -0.2 * 3.14/1000*3.14/1000*sin(3.14*soft_start/1000);

    // compute target position acc
    VectorXd des_acc = VectorXd::Zero(3,1);
    des_acc << -KP(0) * (left_toe_pos(0) - 0.06) - KD(0) * (left_toe_vel(0) - 0),
               -KP(1) * (left_toe_pos(1) - 0.0) - KD(1) * (left_toe_vel(1) - 0), 
               -KP(2) * (left_toe_pos(2) + z_des) - KD(2) * (left_toe_vel(2) - dz_des) + ddz_des;

    // Add toe rotation control
    MatrixXd Weight_Toe = MatrixXd::Identity(4,4);
    VectorXd KP_Toe = KP;
    VectorXd KD_Toe = KD;
    
    //VectorXd left_toe_pos = analytical_expressions.p_left_toe_back(wb_q);
    MatrixXd left_toe_back_jaco  = analytical_expressions.Jp_left_toe_back(wb_q);
    VectorXd left_toe_rot  = VectorXd::Zero(4,1);
    left_toe_rot.block(0,0,3,1) = analytical_expressions.p_left_toe_back(wb_q);
    VectorXd left_toe_drot = VectorXd::Zero(4,1);
    left_toe_drot.block(0,0,3,1) = left_toe_back_jaco * wb_dq;

    MatrixXd left_toe_rot_jaco = MatrixXd::Zero(4,14);
    left_toe_rot_jaco.block(0,0,3,7) = left_toe_back_jaco.block(0,6,3,7);
    VectorXd des_acc_toe = VectorXd::Zero(4,1);

    left_toe_rot(3) =  q(joint::left_hip_yaw); 
    left_toe_drot(3) = dq(joint::left_hip_yaw);
    left_toe_rot_jaco.block(3,0,1,14) = MatrixXd::Zero(1,14);
    left_toe_rot_jaco(3,1) = 1;

    // Currently, need the forth dimension to control leg yaw rotation
    des_acc_toe << -KP_Toe(0) * (left_toe_rot(0) + 0.06) - KD_Toe(0) * (left_toe_drot(0) - 0),
                   -KP_Toe(1) * (left_toe_rot(1) - 0.0) - KD_Toe(1) * (left_toe_drot(1) - 0),
                   -KP_Toe(2) * (left_toe_rot(2) + z_des) - KD_Toe(2) * (left_toe_drot(2) - 0),
                   -KP_Toe(2) * (left_toe_rot(3) - 0) - KD_Toe(2) * (left_toe_drot(3) - 0);

    cout << "dddddddd" << wb_q.block(6,0,7,1) << endl << "dddddddd" << endl;
    //cout << "**************" << endl << left_toe_pos << endl << des_acc << endl << "************" << endl;
    //cout << "**************" << endl << left_toe_rot << endl << des_acc_toe << endl << "************" << endl;
    // B matrix
    MatrixXd B = get_B();
    
    // Spring Jacobian
    MatrixXd Spring_Jaco = get_Spring_Jaco();


    // Formulate QP
    /*int Vars_Num = 20 + 12 + 1;
    int Cons_Num = 20 + 1 + Vars_Num;
    
    // Solve Inverse Dynamics QP
    solver.data()->setNumberOfVariables(Vars_Num);
    solver.data()->setNumberOfConstraints(Cons_Num);

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient(Vars_Num,1);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound(Cons_Num,1);
    Eigen::VectorXd upperBound(Cons_Num,1);
    Eigen::VectorXd QPSolution;

    hessian.resize(Vars_Num,Vars_Num);
    linearMatrix.resize(Cons_Num,Vars_Num);

    // Construct QP Matrix

    // gradient
    VectorXd temp = -2 * left_toe_jaco_fa.transpose() * Weight * des_acc;
    gradient << temp, VectorXd::Zero(13,1);
    
    // upper and lower bounds
    VectorXd ddq_limit = VectorXd::Zero(20,1);
    for(int i=0;i<20;i++){
      ddq_limit(i) = OsqpEigen::INFTY;
    }
    lowerBound << -G-C, VectorXd::Zero(1,1) , -ddq_limit, -116.682, -70.1765, -206.928,-220.928,-35.9759,-35.9759,-116.682, -70.1765, -206.928,-220.928,-35.9759,-35.9759, -OsqpEigen::INFTY;
    upperBound << -G-C, VectorXd::Zero(1,1) ,  ddq_limit, 116.682, 70.1765, 206.928,220.928,35.9759,35.9759,116.682, 70.1765, 206.928,220.928,35.9759,35.9759,  OsqpEigen::INFTY;

    // Hessian Matrix
    MatrixXd hessian_full = MatrixXd::Zero(Vars_Num,Vars_Num);
    hessian_full.block(0,0,20,20) = left_toe_jaco_fa.transpose() * Weight * left_toe_jaco_fa;
    for(int i = 0;i<hessian.rows();i++){
      for(int j = 0;j<hessian.cols();j++){
        if(hessian_full(i,j) != 0){
          hessian.insert(i,j) = hessian_full(i,j);
        }
      }
    }

    // Constraint Matrix
    if(matrix_fixed == 0){
      M_fix = M;
      matrix_fixed = 1;
    }

    MatrixXd constraint_full = MatrixXd::Zero(Cons_Num,Vars_Num);
    constraint_full.block(0,0,20,20) = M_fix;
    constraint_full.block(0,20,20,12) = -B;
    constraint_full.block(0,32,20,1) = -Spring_Jaco.transpose();
    constraint_full.block(20,0,1,20) = Spring_Jaco;
    constraint_full.block(21,0,Vars_Num,Vars_Num) = MatrixXd::Identity(Vars_Num,Vars_Num);

    for(int i = 0;i<constraint_full.rows();i++){
      for(int j = 0;j<constraint_full.cols();j++){
        if(constraint_full(i,j) != 0){
          linearMatrix.insert(i,j) = constraint_full(i,j);
        }
      }
    }

    if(QP_initialized == 0){
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

    VectorXd torque = VectorXd::Zero(6,1);
    for(int i = 0;i<4;i++)
      torque(i) = QPSolution(20+i);
    torque(4) = -0.5 * QPSolution(20+4) + 1.5 * 0.5 * QPSolution(20+5);
    torque(5) =  0.5 * QPSolution(20+4) + 1.5 * 0.5 * QPSolution(20+5);
    */

    int Vars_Num = 14 + 12 + 2;
    int Cons_Num = 14 + 2 + Vars_Num;
    
    // Solve Inverse Dynamics QP
    solver.data()->setNumberOfVariables(Vars_Num);
    solver.data()->setNumberOfConstraints(Cons_Num);

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient(Vars_Num,1);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound(Cons_Num,1);
    Eigen::VectorXd upperBound(Cons_Num,1);
    Eigen::VectorXd QPSolution;

    hessian.resize(Vars_Num,Vars_Num);
    linearMatrix.resize(Cons_Num,Vars_Num);

    // Construct QP Matrix
    // gradient
    VectorXd temp = -2 * (left_toe_jaco_fa.block(0,6,3,14)).transpose() * Weight * des_acc 
    - 2 * left_toe_rot_jaco.transpose() * Weight_Toe * des_acc_toe;
    gradient << temp, VectorXd::Zero(14,1);
    // upper and lower bounds
    VectorXd ddq_limit = VectorXd::Zero(14,1);
    for(int i=0;i<14;i++){
      ddq_limit(i) = OsqpEigen::INFTY;
    }
    lowerBound << -G.block(6,0,14,1) -C.block(6,0,14,1), VectorXd::Zero(2,1) , -ddq_limit, -116.682, -70.1765, -206.928,-220.928,-35.9759,1*-35.9759,-116.682, -70.1765, -206.928,-220.928,-35.9759,-35.9759, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    upperBound << -G.block(6,0,14,1) -C.block(6,0,14,1), VectorXd::Zero(2,1) ,  ddq_limit, 116.682, 70.1765, 206.928,220.928,35.9759,1*35.9759,116.682, 70.1765, 206.928,220.928,35.9759,35.9759,  OsqpEigen::INFTY,  OsqpEigen::INFTY;

    // Hessian Matrix
    MatrixXd hessian_full = MatrixXd::Zero(Vars_Num,Vars_Num);
    hessian_full.block(0,0,14,14) = 2*(left_toe_jaco_fa.block(0,6,3,14)).transpose() * Weight * left_toe_jaco_fa.block(0,6,3,14)
     + 2 * left_toe_rot_jaco.transpose() * Weight_Toe * left_toe_rot_jaco;
    for(int i = 0;i<hessian.rows();i++){
      for(int j = 0;j<hessian.cols();j++){
        if(hessian_full(i,j) != 0){
          hessian.insert(i,j) = hessian_full(i,j);
        }
      }
    }
 
    // Constraint Matrix
    MatrixXd constraint_full = MatrixXd::Zero(Cons_Num,Vars_Num);
    constraint_full.block(0,0,14,14) = M.block(6,6,14,14);
    constraint_full.block(0,14,14,12) = -B.block(6,0,14,12);
    constraint_full.block(0,26,14,2) = -(Spring_Jaco.block(0,6,2,14)).transpose();
    constraint_full.block(14,0,2,14) = Spring_Jaco.block(0,6,2,14);
    constraint_full.block(16,0,Vars_Num,Vars_Num) = MatrixXd::Identity(Vars_Num,Vars_Num);
    
    for(int i = 0;i<constraint_full.rows();i++){
      for(int j = 0;j<constraint_full.cols();j++){
        if(constraint_full(i,j) != 0){
          linearMatrix.insert(i,j) = constraint_full(i,j);
        }
      }
    }

    if(QP_initialized == 0){
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
    VectorXd torque = VectorXd::Zero(6,1);
    for(int i = 0;i<6;i++)
      torque(i) = QPSolution(14+i);
    //torque(4) += 20;
    //torque(5) += 20;
    //torque(4) = -0.5 * QPSolution(14+4) + 1.5 * QPSolution(14+5);
    //torque(5) =  0.5 * QPSolution(14+4) + 1.5 * QPSolution(14+5);

    // Need to think about damping mode/ Or OSC torque command
    // PD control on positions

    soft_start++;
    target_position[10] = 0 - 1.18 * sin(3.14 * soft_start/1000);
    target_position[11] = 0 + 1.18 * sin(3.14 * soft_start/1000);
    for (int i = 0; i < NUM_MOTORS; ++i) {
      if(i>=5){
        command.motors[i].torque =
          150.0 * (target_position[i] - observation.motor.position[i]);
          command.motors[i].velocity = 0.0;
          command.motors[i].damping = 0.75 * limits->damping_limit[i];
      }
      else{
        command.motors[i].torque = torque(i);
        command.motors[i].velocity = 0.0;
        command.motors[i].damping = 0.75 * limits->damping_limit[i];
      }
      
    }
    command.fallback_opmode = Damping;
    command.apply_command = true;
    llapi_send_command(&command);

    // Check if llapi has become disconnected
    if (!llapi_connected()) {
      // Handle error case. You don't need to re-initialize subscriber
      // Calling llapi_send_command will keep low level api open
    }

    // Sleep to keep to a reasonable update rate
    usleep(500);
  }
}

MatrixXd get_Spring_Jaco(){
  MatrixXd Spring_Jaco = MatrixXd::Zero(2,20);
  Spring_Jaco(0,wbc::left_knee) = 1;
  Spring_Jaco(0,wbc::left_tarsus) = 1; 
  Spring_Jaco(1,wbc::left_toe_pitch) = 0; 
  Spring_Jaco(1,wbc::left_toe_roll) = -0; 
  return Spring_Jaco;
}

MatrixXd get_B(){
  MatrixXd B = MatrixXd::Zero(20,12);
  B(wbc::left_hip_roll,0) = 1;
  B(wbc::left_hip_yaw,1) = 1;
  B(wbc::left_hip_pitch,2) = 1;
  B(wbc::left_knee,3) = 1;
  B(wbc::left_toe_pitch,4) = 1;
  B(wbc::left_toe_roll,5) = 1;

  B(wbc::left_toe_pitch,4) = -1;
  B(wbc::left_toe_pitch,5) = 1;
  B(wbc::left_toe_roll,4) = 0.33;
  B(wbc::left_toe_roll,5) = 0.33;

  //B(wbc::left_toe_pitch,4) = -0.5;
  //B(wbc::left_toe_pitch,5) = 1.5;
  //B(wbc::left_toe_roll,4) = -0.5;
  //B(wbc::left_toe_roll,5) = 1.5;

  B(wbc::right_hip_roll,6) = 1;
  B(wbc::right_hip_yaw,7) = 1;
  B(wbc::right_hip_pitch,8) = 1;
  B(wbc::right_knee,9) = 1;
  B(wbc::right_toe_pitch,10) = 1;
  B(wbc::right_toe_roll,11) = 1;

  /*
  for(int i = 0;i<B.rows();i++){
    for(int j = 0;j<B.cols();j++){
      cout << std::fixed << std::setprecision(1) << B(i,j) << " ";
    }
    cout << endl;
  }
  */
  return B;
}