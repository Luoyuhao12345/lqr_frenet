#include "util.h"


int find_match_point(my_lqr::way_points ref_path, float car_x, float car_y)
{
    int n = ref_path.way_points.size();
    float min_dis = 99999;
    int index = 0;
    for (int i = 0; i < n; i++)
    {
        float x = ref_path.way_points[i].x;
        float y = ref_path.way_points[i].y;
        float cur_dis = sqrt(pow(car_x-x, 2)+pow(car_y-y, 2));
        if (cur_dis < min_dis)
        {
            min_dis = cur_dis;
            index = i;
        }
    }
    float lf = 0.2;
    int index2 =index;
    for(int i = index; i<n; i++)
    {
        float x = ref_path.way_points[i].x;
        float y = ref_path.way_points[i].y;
        float cur_dis = sqrt(pow(car_x-x, 2)+pow(car_y-y, 2));
        if (cur_dis < lf) index2 = i;
        else break;
    }
    return index2;
}


float yaw2yaw(float yaw)
{
    if(yaw >= pi*3/4) yaw -= 2.0*pi;
    if(yaw <= -pi*3/4) yaw += 2.0*pi;
    return yaw;
}

float rad2deg(float x)
{
    return x/pi*180.0;
}