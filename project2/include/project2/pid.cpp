#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

PID::PID(){

    pre_error = 0;
    error = 0;
    error_sum = 0;
    error_diff = 0;
    Kp = 12;
    Ki = 0.9;
    Kd = 7.8; 
}

void PID::reset() {
    error = 0;
    error_sum = 0;
    error_diff = 0;
}

float PID::get_control(point car_pose, point goal_pose){

    float ctrl;

    // the angle to the goal_pose from the car_pose
    double theta_g;

    // Use arccos to find the theta_g
    // Make theta_g included in boundary [0, 2*M_PI)
    if (goal_pose.y - car_pose.y > 0)
	theta_g = acos((goal_pose.x-car_pose.x)/(sqrt(pow(goal_pose.x - car_pose.x,2.0) + pow(goal_pose.y - car_pose.y,2.0))));
    else if (goal_pose.y - car_pose.y < 0)
	theta_g = 2*PI - acos((goal_pose.x-car_pose.x)/(sqrt(pow(goal_pose.x - car_pose.x,2.0) + pow(goal_pose.y - car_pose.y,2.0))));
    else{
	if(goal_pose.x - car_pose.x >0)
	    theta_g = 0.0;
	else
	    theta_g = PI;
    }

    // e(t) = theta_g - theta_h, theta_h = car_pose.th
    error = (float)theta_g - (float)car_pose.th;

    // Make error included in boundary [-M_PI, M_PI], because rotating over M_PI can be replace by rotating to opposite direction 
    if (error > PI) error = error - 2.0 * PI;
    else if (error < (-1.0) * PI) error = error + 2.0 * PI;

    // Calculate sum of error(for integral part), difference of error(for derivative part)
    error_sum += error;
    error_diff = error - pre_error;

    // Calculate proportional, integral, derivative part
    //In pidmain.cpp, control rate is 10Hz, so set dt = 0.1
    float P = Kp * error;
    float I = Ki * error_sum / 60;
    float D = Kd * error_diff * 60;

    // Calculate ctrl
    ctrl = P + I + D;

    // Store error to pre_error
    pre_error = error;

    return ctrl;
}
