#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "lqr_util.h"
#include "pid.h"
#include "util.h"
#include "car_obj.h"
#include "car_info.h"
#include "way_points.h"

LQRUtil lu = LQRUtil();
PID pid = PID(2, 0, 0);
CarObj car_state = CarObj();
bool receive_path = false;
my_lqr::way_points ref_path;
ros::Publisher cmd_pub;

void control_cb(const ros::TimerEvent& event)
{
    if (!receive_path)
    {
        cout<<"wait for path"<<endl;
        return;
    }
    int index = find_match_point(ref_path, car_state.x, car_state.y);
    lu.update_car_state(car_state.cur_vel);
    float ref_steer = ref_path.way_points[index].delta;
    cout<<"ref_steer: "<<rad2deg(ref_steer)<<endl;
    float ref_yaw = ref_path.way_points[index].yaw;
    float ref_x = ref_path.way_points[index].x;
    float ref_y = ref_path.way_points[index].y;
    float e_d = -sin(ref_yaw)*(car_state.x-ref_x)+cos(ref_yaw)*(car_state.y-ref_y);
    float e_phi = car_state.yaw - ref_yaw;
    e_phi = yaw2yaw(e_phi);
    cout<<"e_phi:"<<rad2deg(e_phi)<<", e_d:"<<e_d<<endl;
    geometry_msgs::Twist msg;
    float car_steer = lu.cal_steer(e_phi, e_d, car_state.cur_vel, ref_steer);
    car_steer = pid.pid(car_steer, e_d);
    car_state.steer = car_steer;
    msg.angular.z = car_state.steer;
    msg.linear.x = car_state.get_aim_vel();
    cmd_pub.publish(msg);
    cout<<"car_steer: "<<rad2deg(car_state.steer)<<", car_vel: "<<car_state.cur_vel<<endl;
}

void car_info_cb(const my_lqr::car_info::ConstPtr& msg)
{
    car_state.x = msg->car_x;
    car_state.y = msg->car_y;
    car_state.yaw = msg->car_yaw;
    // car_state.cur_vel = msg->car_v;
    car_state.cur_vel = car_state.get_aim_vel();
}

void ref_path_cb(const my_lqr::way_points::ConstPtr& msg)
{
    ref_path = *msg;
    receive_path = true;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "car_controller");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber car_info_sub = nh.subscribe("/car_info", 1, car_info_cb);
    ros::Subscriber path_sub = nh.subscribe("/ref_path", 1, ref_path_cb);
    ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), control_cb);

    float q, r;
    nh.param("/car_controller/q", q, float(2));
    nh.param("/car_controller/r", r, float(100));
    cout<<"q:"<<q<<", r:"<<r<<endl;
    lu.init_mat(q, r);
    lu.init_feedback_k();

    float v1, v2;
    nh.param("/car_controller/straight_vel", v1, float(0.6));
    nh.param("/car_controller/arc_vel", v2, float(0.3));
    cout<<"v1:"<<v1<<", v2:"<<v2<<endl;
    car_state.straight_vel = v1;
    car_state.arc_vel = v2;

    ros::spin();
    return 0;
}