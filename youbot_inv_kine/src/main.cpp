#include <unistd.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <stdint.h>

#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"

#include <eigen3/Eigen/Dense>

#define PI  3.14159265358979323846

using namespace std;
using namespace Eigen;

double youbot_a[5] = {-19, 155, 135, 0.0, 0.0};
double youbot_alpha[5] =  {PI/2, 0.0, 0.0, PI/2, 0.0};
double youbot_d[5] =    {147, 0.0, 0.0, 0.0, 218};
double youbot_theta[5] =  {0.0, PI/2, 0.0, PI/2, 0.0};

double current_joint[5];
double next_joint[6];

int direction = 0;
double inc = 0.0;

bool move_inc = false;

Matrix4d T_youbot, T_inc;

Matrix4d DH_matrix(double a, double alpha, double d, double theta)
{
    Matrix4d A;

    double sa = sin(alpha);
    double ca = cos(alpha);
    double st = sin(theta);
    double ct = cos(theta);

    A(0, 0) =  ct;  A(0, 1) = -st * ca;  A(0, 2) =  st * sa;  A(0, 3) =  ct * a;
    A(1, 0) =  st;  A(1, 1) =  ct * ca;  A(1, 2) = -ct * sa;  A(1, 3) =  st * a;
    A(2, 0) =  0.0; A(2, 1) =  sa;       A(2, 2) =  ca;       A(2, 3) =  d;
    A(3, 0) =  0.0; A(3, 1) =  0.0;      A(3, 2) =  0.0;      A(3, 3) =  1.0;

    return A;
}

Matrix4d rot_x(double degree)
{
    Matrix4d A;
    A(0, 0) = 1.0; A(0, 1) = 0.0;         A(0, 2) = 0.0;          A(0, 3) = 0.0;
    A(1, 0) = 0.0; A(1, 1) = cos(degree); A(1, 2) = -sin(degree); A(1, 3) = 0.0;
    A(2, 0) = 0.0; A(2, 1) = sin(degree); A(2, 2) = cos(degree);  A(2, 3) = 0.0;
    A(3, 0) = 0.0; A(3, 1) = 0.0;         A(3, 2) = 0.0;          A(3, 3) = 1.0;
    return A;
}

Matrix4d rot_y(double degree)
{
    Matrix4d A;
    A(0, 0) = cos(degree);  A(0, 1) = 0.0; A(0, 2) = sin(degree);  A(0, 3) = 0.0;
    A(1, 0) = 0.0;          A(1, 1) = 1.0; A(1, 2) = 0.0;          A(1, 3) = 0.0;
    A(2, 0) = -sin(degree); A(2, 1) = 0.0; A(2, 2) = cos(degree);  A(2, 3) = 0.0;
    A(3, 0) = 0.0;          A(3, 1) = 0.0; A(3, 2) = 0.0;          A(3, 3) = 1.0;
    return A;
}

Matrix4d rot_z(double degree)
{
    Matrix4d A;
    A(0, 0) = cos(degree);  A(0, 1) = -sin(degree); A(0, 2) = 0.0;  A(0, 3) = 0.0;
    A(1, 0) = sin(degree);  A(1, 1) =  cos(degree); A(1, 2) = 0.0;  A(1, 3) = 0.0;
    A(2, 0) = 0.0;          A(2, 1) = 0.0;          A(2, 2) = 1.0;  A(2, 3) = 0.0;
    A(3, 0) = 0.0;          A(3, 1) = 0.0;          A(3, 2) = 0.0;  A(3, 3) = 1.0;
    return A;
}

Matrix4d tran_x(double d)
{
    Matrix4d A;
    A.setIdentity();
    A(0, 3) = d;
    return A;
}

Matrix4d tran_y(double d)
{
    Matrix4d A;
    A.setIdentity();
    A(1, 3) = d;
    return A;
}

Matrix4d tran_z(double d)
{
    Matrix4d A;
    A.setIdentity();
    A(2, 3) = d;
    return A;
}

Matrix4d youbot_forward_kine(double input_theta[])
{
    Matrix4d current_pose;
    current_pose.setIdentity();
    for(int i = 0; i < 5; i++)
    {
        current_pose = current_pose*DH_matrix(youbot_a[i], youbot_alpha[i], youbot_d[i], youbot_theta[i] + input_theta[i]);
    }
    return current_pose;
}

VectorXd youbot_inv_kine(Matrix4d pose, double current_joint[])
{
    VectorXd inv_sol(5);
    cout << "next pose" << endl;
    cout << pose << endl;

    double th1 = atan2(pose(1, 3), pose(0, 3));

    if (th1 > 0.0)
    {
        if (abs(th1 - current_joint[0]) <= abs(th1 - PI - current_joint[0]))
            inv_sol(0) = th1;
        else
            inv_sol(0) = th1 - PI;
    }
    else
    {
        if (abs(th1 - current_joint[0]) <= abs(th1 + PI - current_joint[0]))
            inv_sol(0) = th1;
        else
            inv_sol(0) = th1 + PI;
    }

    inv_sol(4) = atan2(pose(0, 0)*sin(inv_sol(0)) - pose(1, 0)*cos(inv_sol(0)), \
    pose(0, 1)*sin(inv_sol(0)) - pose(1, 1)*cos(inv_sol(0)));

    Matrix4d T1 = tran_z(youbot_d[0])*rot_z(inv_sol(0))*tran_x(youbot_a[0])*rot_x(youbot_alpha[0]);
    Matrix4d T5 = tran_z(youbot_d[4])*rot_z(inv_sol(4))*tran_x(youbot_a[4])*rot_x(youbot_alpha[4]);
    Matrix4d T24 = T1.inverse()*pose*T5.inverse();

    double th234 = atan2(-T24(1, 0), T24(1, 2));

    double acosval = (pow(T24(0, 3), 2) + pow(T24(1, 3), 2) - pow(youbot_a[1], 2) - pow(youbot_a[2], 2))\
    /(2*youbot_a[1]*youbot_a[2]);

    if (abs(acosval) > 1.0)
    {
        inv_sol(0) = 999.0;
    }
    else
    {
        double th3, th2[2];

        th3 = acos(acosval);

        if (abs(th3 - current_joint[2]) <= abs(-th3 - current_joint[2]))
            inv_sol(2) =  th3;
        else
            inv_sol(2) = -th3;

        double A = youbot_a[1] + youbot_a[2]*cos(inv_sol(2));
        double B = youbot_a[2]*sin(inv_sol(2));
        double beta = atan2(B, A);


        th2[0] =  acos(T24(1, 3)/sqrt(A*A + B*B)) - beta;
        th2[1] = -acos(T24(1, 3)/sqrt(A*A + B*B)) - beta;

        if (abs(th2[0] - current_joint[1]) <= abs(th2[1] - current_joint[1]))
            inv_sol(1) = th2[0];
        else
            inv_sol(1) = th2[1];

        inv_sol(3) = th234 - inv_sol(1) - inv_sol(2);
    }

    if (inv_sol(0) == 999.0)
    {
        cout << "Cannot find an inverse kinematic solution for the pose." << endl;
        cout << "Return current pose." << endl;

        for (int i = 0; i < 5; i++)
            inv_sol(i) = current_joint[i];
    }

    for (int i = 0; i < 5; i++)
        cout << "Joint " << i  << "  = " << current_joint[i] << "  ---->  " << inv_sol(i) << endl;

    return inv_sol;
}

int pose_increment_callback(const std_msgs::Float64MultiArray::ConstPtr& joint_cmd)
{
    int i = 0;
    for(std::vector<double>::const_iterator it = joint_cmd->data.begin(); it != joint_cmd->data.end(); ++it)
    {
        if (i == 0)
            direction = (int)*it;
        else
            inc = *it;
        i++;
    }

    if (direction != 7)
        next_joint[5] = 0.0;
    else
        next_joint[5] = inc;

    switch (direction)
    {
        case 1:
            T_inc = rot_x(inc);
            break;
        case 2:
            T_inc = rot_y(inc);
            break;
        case 3:
            T_inc = rot_z(inc);
            break;
        case 4:
            T_inc = tran_x(inc);
            break;
        case 5:
            T_inc = tran_y(inc);
            break;
        case 6:
            T_inc = tran_z(inc);
            break;
        default:
            break;
    }
    cout << "T_current_youbot = " << endl;
    cout << T_youbot << endl;
    cout << "Increment mat = " << endl;
    cout << T_inc << endl;
    VectorXd inv_sol = youbot_inv_kine(T_youbot*T_inc, current_joint);

    for (int i = 0; i < 5; i++)
        next_joint[i] = inv_sol(i);
    move_inc = true;
    return 0;
};

void compute_forward_kine(const sensor_msgs::JointState::ConstPtr &measure_joint)
{
    int i = 0;
    for(std::vector<double>::const_iterator it = measure_joint->position.begin(); it != measure_joint->position.end(); ++it)
    {
        current_joint[i] = *it;
        i++;
    }
    T_youbot = youbot_forward_kine(current_joint);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_inv_node");

    ros::NodeHandle nhJointPub_;
    ros::Publisher  JointPublisher;
    ros::Subscriber CommandPoseInc;
    ros::Subscriber JointSubscriber;

    JointPublisher   = nhJointPub_.advertise<trajectory_msgs::JointTrajectoryPoint>("/arm/cmd_joint_traj", 1);
    CommandPoseInc = nhJointPub_.subscribe<std_msgs::Float64MultiArray>("/pose_inc", 1, pose_increment_callback);
    JointSubscriber = nhJointPub_.subscribe<sensor_msgs::JointState>("/arm/joint_states", 1, compute_forward_kine);

    trajectory_msgs::JointTrajectoryPoint JointData;

    for(int i = 0; i < 5; i++)
        next_joint[i] = current_joint[i];

    while (ros::ok())
    {
        if (move_inc)
        {
            JointData.positions.clear();

            for (int i = 0; i < 5; i++)
                JointData.positions.push_back(next_joint[i] * 180.0 / PI);
            JointData.positions.push_back(next_joint[5]);

            JointPublisher.publish(JointData);
            move_inc = false;
        }
        ros::spinOnce();
        usleep(3);
    }

    return 0;
}

