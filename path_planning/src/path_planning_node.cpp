#include "ros/ros.h"
#include "youbot_fk_ik/youbotKine.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"

#include "boost/foreach.hpp"
#include <string>

#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <cmath>
#include "visualization_msgs/MarkerArray.h"

#include <list>

using namespace Eigen;
#define POINTS_DATA 500
trajectory_msgs::JointTrajectoryPoint traj_pt;

//TODO: You can design your code to achieve the q6 task 
//however you want as long as it stays in this file and it runs.

struct points {
    Matrix4d transform;
};

/*void show_degrees(double points[], char name[]) 
{
    std::cout << "Angulos de " << name << ": \n" << std::endl;
    for (int i = 0; i<5;i++)
        std::cout << points[i]*180/M_PI << std::endl;
}*/

MatrixXd quaternion_rotation(double q[]) 
{
    /**
     * @brief Compute the rotation matrix from a quaternion
     * 
     * @param q Array of a quaternion
     * 
     * @return Rotation matrix 
     */
    MatrixXd rotation(3,3);
    double qx = q[0];
    double qy = q[1];
    double qz = q[2];
    double qw = q[3];
    rotation(0,0) = 1 - 2*qy*qy - 2*qz*qz; 
    rotation(0,1) = 2*qx*qy - 2*qz*qw; 
    rotation(0,2) = 2*qx*qz + 2*qy*qw;
    rotation(1,0) = 2*qx*qy + 2*qz*qw; 
    rotation(1,1) = 1 - 2*qx*qx - 2*qz*qz;
    rotation(1,2) = 2*qy*qz - 2*qx*qw;
    rotation(2,0) = 2*qx*qz - 2*qy*qw;
    rotation(2,1) = 2*qy*qz + 2*qx*qw; 
    rotation(2,2) = 1 - 2*qx*qx - 2*qy*qy;
    return rotation;
}

bool intesect_object(double *point) 
{
    /**
     * @brief Determine if a 3D points intersects with an object
     * 
     * @param point Reference of a point
     * 
     * @return If the points intersect with any of the objects 
     */
    double object_1[3][2] = { 
        0.25751, 0.37751, 
        0, 0.13, 
        0, 0.14 };// cylinder 1
    double object_2[3][2] = { 
        -0.378049, -0.278049, 
        -0.276524, -0.176524, 
        0, 0.08 };// cylinder 0
    double object_3[3][2] = { 
        -0.4118985, -0.2178655,
        -0.200138, -0.1231,
        0, 0.2};// box
    bool intersect = false;
    if (point[0] > object_1[0][0] && point[0] < object_1[0][1]){
        if (point[1] > object_1[1][0] && point[1] < object_1[1][1]){
            if (point[2] > object_1[2][0] && point[2] < object_1[2][1]){
                intersect = true;
            }
        }
    }
    if (point[0] > object_2[0][0] && point[0] < object_2[0][1]){
        if (point[1] > object_2[1][0] && point[1] < object_2[1][1]){
            if (point[2] > object_2[2][0] && point[2] < object_2[2][1]){
                intersect = true;
            }
        }
    }
    if (point[0] > object_3[0][0] && point[0] < object_3[0][1]){
        if (point[1] > object_3[1][0] && point[1] < object_3[1][1]){
            if (point[2] > object_3[2][0] && point[2] < object_3[2][1]){
                intersect = true;
            }
        }
    }
    return intersect;
}

void generate_points(double intervals[3][2], double data[POINTS_DATA][3])
{
    /**
     * @brief Generates data points to use path planning algorithm
     * 
     * @param intervals The workspace intervals without the avoiding objects 
     * @param data Data points generated randomly to make path planning
     */
    double x_interval[2] = {intervals[0][0], intervals[0][1]};
    double y_interval[2] = {intervals[1][0], intervals[1][1]};
    double z_interval[2] = {intervals[2][0], intervals[2][1]};
    int n_data = 0;
    while (n_data < POINTS_DATA)
    {
        double data_aux[3];
        for (int i = 0; i < 3; i++) 
        {
            double range = intervals[i][1] - intervals[i][0];
            double rand_num = ((double) rand() / (RAND_MAX));
            data_aux[i] = rand_num*range + intervals[i][0];
        }
        if (!intesect_object(data_aux)){
            data[n_data][0] = data_aux[0];
            data[n_data][1] = data_aux[1];
            data[n_data][2] = data_aux[2];
            n_data++;
        }
    }
}

std::list<double*> connect_points(double data[POINTS_DATA+5][3])
{
    /**
     * @brief Connect points through the nearest neighbor and a distance limit
     * 
     * @param data Data points generated randomly to make path planning
     * 
     * @return Connections between data points
     */
    std::list<double*> connections;
    double distance_limit = 0.1;
    double interpolate_points = 50;
    for (int i = 0; i < POINTS_DATA + 5; i++)
    {
        double x_pos = data[i][0];
        double y_pos = data[i][1];
        double z_pos = data[i][2];
        for (int j = i + 1; j < POINTS_DATA + 5;j++) 
        {
            double x_comp = data[j][0];
            double y_comp = data[j][1];
            double z_comp = data[j][2];
            double distance = sqrt(pow(x_comp - x_pos, 2) + pow(y_comp - y_pos, 2) + pow(z_comp - z_pos, 2));
            bool is_connection = false;
            if (distance < distance_limit) 
            {
                is_connection = true;
                double x_pace = (x_comp - x_pos)/interpolate_points;
                double y_pace = (y_comp - y_pos)/interpolate_points;
                double z_pace = (z_comp - z_pos)/interpolate_points;
                for (int k = 1; k <= interpolate_points; k++)
                {
                    double data_aux[3];
                    data_aux[0] = x_pos + x_pace*k;
                    data_aux[1] = y_pos + y_pace*k;
                    data_aux[2] = z_pos + z_pace*k;
                    if (intesect_object(data_aux))
                    {
                        is_connection = false;
                        break;
                    }
                }
            }
            if (is_connection)
            {
                connections.push_back(new double[3] {(double)i, (double)j, distance});
            }
        }
    }
    return connections;
}

std::list<double*> dijkstra(std::list<double*> links, int init, int end)
{
    /**
     * @brief Apply the Dijkstra algorithm over the connections to reach the desired points
     * 
     * @param links Conectors betweem data points
     * @param init First point id
     * @param init Last point id
     */
    std::list<std::list<double*>> paths;
    std::list<int> used;
    int current_data_id = init;
    while (true)
    {
        int link_id = 0;
        std::list<double*> current_path;
        bool used_data = true;
        bool all_complete = false;
        for (std::list<std::list<double*>>::iterator pt = paths.begin(); pt != paths.end(); pt++)
        { 
            all_complete = true;
            used_data = false;
            for (std::list<int>::iterator us = used.begin(); us != used.end(); us++)
            {
                if (*us == (*pt).back()[2])
                    used_data = true;
            }
            if (used_data)
                continue;
            double min_dist = 100000000;
            if (min_dist > (*pt).back()[1])
            {
                min_dist = (*pt).back()[1];
                current_path = (*pt);
                current_data_id = (*pt).back()[2];
            }
            all_complete = false;
        }
        if (!current_path.empty() && current_path.back()[2] == (double)end){
            return current_path;
        }
        if (all_complete)
        {
            return current_path;
        }
        for (std::list<double*>::iterator it = links.begin(); it != links.end(); it++)
        {
            double *aux = *it;
            int next_point = 0;
            if (aux[0] == current_data_id) {
                next_point = aux[1];
            }
            if (aux[1] == current_data_id) {
                next_point = aux[0];
            }
            bool used_data = false;
            for (std::list<int>::iterator us = used.begin(); us != used.end(); us++)
            {
                if (*us == next_point)
                    used_data = true;
            }
            if (used_data || next_point == 0)
            {
                link_id++;
                continue;
            }
            bool is_in_path = false;
            if (!current_path.empty()){
            for (std::list<std::list<double*>>::iterator pt = paths.begin(); pt != paths.end(); pt++)
            {
                double *current_node = (*pt).back();
                double *last_node = current_path.back();
                double distance = last_node[1] + aux[2];
                if (current_node[2] == next_point && current_node[1] > distance) 
                {
                    paths.remove(*pt);
                    std::list<double*> new_path(current_path);
                    new_path.push_back(new double [3] {(double)link_id, distance, (double)next_point});
                    paths.push_back(new_path);
                    is_in_path = true;
                }
            }
            }
            if (current_path.empty())
            {
                double distance = aux[2];
                std::list<double*> new_path;
                new_path.push_back(new double [3] {(double)link_id, distance, (double)next_point});
                paths.push_back(new_path);
            }
            else if (!is_in_path)
            {
                double *last_node = current_path.back();
                double distance = last_node[1] + aux[2];
                std::list<double*> new_path(current_path);
                new_path.push_back(new double[3] {(double)link_id, distance, (double)next_point});
                paths.push_back(new_path);
            }
            link_id++;
        }
        used.push_back(current_data_id);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_cw2");

    youbot_kinematic youbot;
    youbot.init();
    trajectory_msgs::JointTrajectory my_traj;
    trajectory_msgs::JointTrajectoryPoint my_pt;
    rosbag::Bag bag;
    ros::NodeHandle nh;
    ros::NodeHandle node_handle;

    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 5);

    my_traj.header.stamp = ros::Time::now();
    my_traj.joint_names.push_back("arm_joint_1");
    my_traj.joint_names.push_back("arm_joint_2");
    my_traj.joint_names.push_back("arm_joint_3");
    my_traj.joint_names.push_back("arm_joint_4");
    my_traj.joint_names.push_back("arm_joint_5");

    my_pt.positions.resize(5);
    my_pt.velocities.resize(5);
    my_pt.accelerations.resize(5);

    int checkpoint_data = atoi(argv[1]);
    double tfs = 10;
    double* steps = new double[5];
    steps[0] = 1; steps[1] = 1; steps[2] = 50; steps[3] = 1; steps[4] = 50;
    double* origin_angles = new double[5];
    origin_angles[0] = 0;origin_angles[1] = 0;origin_angles[2] = 0;
    origin_angles[3] = 0;origin_angles[4] = 0;
    MatrixXd last_v(1, 4);
    last_v(0, 0) = 0; last_v(0, 1) = 0; last_v(0, 2) = 0; last_v(0, 3) = 1;
    double time_interval = 1;
    if (checkpoint_data == 1)
    {
        bag.open(MY_BAG_A, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("joint_data"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        double* init_angles = new double[5];
        double* end_angles = new double[5];
        int iter_n = 0;
        double angles[5][5];
        BOOST_FOREACH(rosbag::MessageInstance const m, view) 
        {
            sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
            if (J != NULL)
            {
                if (J->position.size() != 0)
                {
                    for (int i = 0; i < 5;i++)
                    {
                        angles[i][iter_n] = J->position.at(i);
                    }
                }
            }
            iter_n++;
        }
        //int order[5] = {0, 1, 2, 3, 4}; // original
        int order[5] = {0, 1, 2, 4, 3}; // left to right
        //int order[5] = {3, 4, 2, 1, 0}; // rigth to left
        for(int j = 0; j < 5 ; j++) 
        {
            double dt = time_interval/steps[j];
            for (int i = 0; i < 5;i++)
            {
                init_angles[i] = end_angles[i];
                end_angles[i] = angles[i][order[j]];
            }
            if (j == 0)
            {
                Matrix4d end_pose_matrix = youbot.forward_kine_offset(end_angles, 5);
                double* new_origin_angles = youbot.inverse_kine_ite(end_pose_matrix, origin_angles);
                double *joint_outcome = youbot.apply_offset(new_origin_angles);
                tfs = tfs + ceil(20/steps[j]);
                my_pt.time_from_start.sec = tfs;
                for (int pos = 0; pos<5; pos++)
                {
                    my_pt.positions.at(pos) = joint_outcome[pos];
                }
                my_traj.points.push_back(my_pt);
                continue;
            }
            Matrix4d init_pose_matrix = youbot.forward_kine_offset(init_angles, 5);
            Matrix4d end_pose_matrix = youbot.forward_kine_offset(end_angles, 5);
            MatrixXd rot_mat_init = init_pose_matrix.block(0,0,3,3);
            MatrixXd rot_mat_end = end_pose_matrix.block(0,0,3,3);
            Vector3d tras_mat_init = init_pose_matrix.block(0,3,3,1);
            Vector3d tras_mat_end = end_pose_matrix.block(0,3,3,1);
            double* target_angles = init_angles;
            for (int k = 1; k <= steps[j]; k++) 
            {
                MatrixXd aux_matrix(3,4);
                MatrixXd t_matrix(4,4);
                Vector3d aux_pos = tras_mat_init + dt*k*(tras_mat_end - tras_mat_init);
                MatrixXd aux_rot = rot_mat_init*((rot_mat_init.inverse()*rot_mat_end).log()*dt*k).exp();
                aux_matrix << aux_rot, aux_pos;
                t_matrix << aux_matrix, last_v;
                //std::cout << youbot.obtain_pose_vector(t_matrix) << std::endl;
                double* new_origin_angles = youbot.inverse_kine_ite(t_matrix, origin_angles);
                double *joint_outcome = youbot.apply_offset(new_origin_angles);
                tfs = tfs + ceil(20/steps[j]);
                my_pt.time_from_start.sec = tfs;
                for (int pos = 0; pos<5; pos++)
                {
                    my_pt.positions.at(pos) = joint_outcome[pos];
                }
                my_traj.points.push_back(my_pt);
            }            
        }
        sleep(5);
        traj_pub.publish(my_traj);
        ros::spinOnce();
        bag.close();
        std::cout << "Finished uploading trajectory" << std::endl;
        return 5;
    }
    else if (checkpoint_data == 2)
    {
        double intervals[3][2] = {
            -0.5, 0.5,
            -0.5, 0.5,
            0, 0.4};
        double data_points[POINTS_DATA+5][3];

        bag.open(MY_BAG_B, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("target_position"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        int iter_n= 0;
        points *data = new points[5];
        BOOST_FOREACH(rosbag::MessageInstance const m, view) 
        {
            geometry_msgs::Transform::ConstPtr J = m.instantiate<geometry_msgs::Transform>(); 
            if (J != NULL)
            {
                double q[4] = {J->rotation.x, J->rotation.y, J->rotation.z, J->rotation.w};
                VectorXd translation(3); translation(0) = J->translation.x;
                translation(1) = J->translation.y; translation(2) = J->translation.z;
                data_points[POINTS_DATA+iter_n][0] = J->translation.x;
                data_points[POINTS_DATA+iter_n][1] = J->translation.y;
                data_points[POINTS_DATA+iter_n][2] = J->translation.z;
                MatrixXd rotation = quaternion_rotation(q);
                MatrixXd aux_matrix(3,4);
                aux_matrix << rotation, translation;
                data[iter_n].transform << aux_matrix, last_v;
            }
            iter_n++;
        }
        generate_points(intervals, data_points);
        std::list<double*> links = connect_points(data_points);
        Matrix4d end_matrix;
        dijkstra(links, POINTS_DATA, POINTS_DATA + 1);
        int order[5] = {0, 1, 4, 3, 2};//0, 1, 2, 4, 3};
        for(int j = 0; j < 5 ; j++) 
        {
            Matrix4d init_matrix = end_matrix;
            init_matrix = youbot.forward_kine_offset(origin_angles, 5);
            end_matrix = data[order[j]].transform;
            if (1)// || j == 2)
            {
                //double *new_origin_angles
                origin_angles = youbot.inverse_kine_ite(end_matrix, origin_angles);
                origin_angles = youbot.norm_angle(origin_angles);
                origin_angles = youbot.apply_offset(origin_angles);
                //for (int h = 0; h < 5;h++)
                 //   std::cout << origin_angles[h]*180/M_PI << std::endl;
                //std::cout << youbot.forward_kine_offset(origin_angles, 5) << std::endl;
                tfs = tfs + 15;//ceil(50/steps[j]);
                my_pt.time_from_start.sec = tfs;
                for (int pos = 0; pos<5; pos++)
                {
                    my_pt.positions.at(pos) = origin_angles[pos];
                    //my_pt.velocities.at(pos) = 0.02;
                }
                my_traj.points.push_back(my_pt);
            }
        }
        sleep(10);
        traj_pub.publish(my_traj);
        bag.close();
        ros::spinOnce();
        std::cout << "run q6b" << std::endl;
        std::cout << "Finished uploading trajectory" << std::endl;
        //Load q6b data
        return 123;
    }

    while(ros::ok())
    {
        ros::spinOnce();
        usleep(10);
    }

    return 123;
}
