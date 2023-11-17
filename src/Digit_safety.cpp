#include "Digit_safety.hpp"

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

Digit_safety::Digit_safety(int wd_sz, int j_sz){
    safety_trigger = false;
    jp_min = VectorXd::Zero(j_sz);
    jp_max = VectorXd::Zero(j_sz);
    jv_lim = VectorXd::Zero(j_sz);

    jp_max << deg2rad(60),deg2rad(40),deg2rad(90),deg2rad(58.4),deg2rad(71.6),deg2rad(34),deg2rad(33) // left leg upper bound
             ,deg2rad(60),deg2rad(40),deg2rad(60),deg2rad(80),deg2rad(50.3),deg2rad(44),deg2rad(37);  // right leg upper bound
    jp_min << deg2rad(-60),deg2rad(-40),deg2rad(-60),deg2rad(-80),deg2rad(-50.3),deg2rad(-44),deg2rad(-37) // left leg lower bound
             ,deg2rad(-60),deg2rad(-40),deg2rad(-90),deg2rad(-58.4),deg2rad(-71.6),deg2rad(-34),deg2rad(-33);  // right leg lower bound
    jv_lim << deg2rad(60),deg2rad(60),deg2rad(60),deg2rad(60),deg2rad(60),deg2rad(60),deg2rad(60)     // left leg velocity bound
             ,deg2rad(60),deg2rad(60),deg2rad(60),deg2rad(60),deg2rad(60),deg2rad(60),deg2rad(60);    // right leg velocity bound
}

bool Digit_safety::checkSafety(){
    return safety_trigger;
}

void Digit_safety::resetSafety(){
    safety_trigger = false;
}

void Digit_safety::updateSafety(VectorXd cur_q, VectorXd cur_dq){
    safety_trigger = false;
}