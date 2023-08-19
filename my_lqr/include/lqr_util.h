#ifndef __LQR_UTIL_H_
#define __LQR_UTIL_H_

#include <iostream>
#include <vector>
#include </usr/local/include/eigen3/Eigen/Dense>


#define CAR_L   0.26
#define EPS 1.0e-3
#define N 200


using namespace std;
using namespace Eigen;
#define pi 3.14159

class LQRUtil 
{
public:
    LQRUtil();
    void update_car_state(float vel);
    float cal_steer(float e_phi, float e_d, float car_vel, float ff);
    void init_mat(float q, float r);
    void init_feedback_k();
    // 调试时使用
    void look_k();

private:
    void _init_Q_R(float q, float r);
    MatrixXd _cal_ricatti();
    float _cal_fb(float e_phi, float e_d, float car_vel);
    float _cal_ff(float ff);

private:
    Matrix2d A;
    Vector2d B;
    MatrixXd Q;
    MatrixXd R;
    vector<MatrixXd> feedback_k;
    
    int Nx, Nu;
    float T;
};


#endif