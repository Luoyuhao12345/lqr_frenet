#include "lqr_util.h"

LQRUtil::LQRUtil()
{
	T = 0.1;
	Nx = 2;
	Nu = 1;
}

void LQRUtil::init_mat(float q, float r)
{
    cout<<"init mat"<<endl;
    A << 1,0,
              0,1;
	B<<0,0;
	_init_Q_R(q, r);
}

void LQRUtil::update_car_state(float vel)
{
	A(1,0) = T*vel;
	B(0,0) = T*vel/CAR_L;
}

float LQRUtil::cal_steer(float e_phi, float e_d, float car_vel, float ff)
{
    float a = _cal_fb(e_phi, e_d, car_vel);
    float b = _cal_ff(ff);
    float c = a+b;
    if(c >= 40.0/180.0*pi) c = 40.0/180.0*pi;
    if(c <= -40.0/180.0*pi) c = -40.0/180.0*pi;
    return c;
}

void LQRUtil::init_feedback_k()
{
    cout<<"k init..."<<endl;
    MatrixXd P = _cal_ricatti();
    float min_vel = 0.2;
    float max_vel = 1.8;
    float temp_vel = min_vel;
    while (temp_vel<max_vel)
    {
        A(1, 0) = T*temp_vel;
        B(0, 0) = T*temp_vel/CAR_L;
        MatrixXd K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
        feedback_k.push_back(K);
        temp_vel += 0.01;
    }
    cout<<"k init done"<<endl;
}

void LQRUtil::_init_Q_R(float q, float r)
{
	// Q = Eigen::MatrixXd::Identity(Nx, Nx)*q;
    Q.resize(Nx, Nx);
    Q<<1,0,
            0, 1;
    Q = q*Q;
	R = Eigen::MatrixXd::Identity(Nu, Nu)*r;
}

MatrixXd LQRUtil::_cal_ricatti()
{
    MatrixXd Qf= Q;
    MatrixXd P = Qf;
    MatrixXd P_;
    for(int i=0;i<N;i++)
    {
        P_ = Q+A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
        if((P_-P).maxCoeff()<EPS&&(P-P_).maxCoeff()<EPS) break;
        P = P_;
    }
    return P_;
}

void LQRUtil::look_k()
{
    int n = feedback_k.size();
    for(int i = 0; i<n; i+=10)
    {
        cout<<feedback_k[i]<<endl;
    }
}

float LQRUtil::_cal_fb(float e_phi, float e_d, float car_vel)
{
    Eigen::MatrixXd e;
	e.resize(Nx, Nu);
	e << e_phi,e_d;
    int index = car_vel/0.01;
    MatrixXd u = feedback_k[index]*e;
    return u(0,0);
}

float LQRUtil::_cal_ff(float ff)
{
    return ff;
}
