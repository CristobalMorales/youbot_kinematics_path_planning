# youbot_kinematics_path_planning
This repo has three packages containing the model of the Youbot Kinematics, another for the path planning and the last one of an example of the tracked points

## youbot_fk_ik package
This package contains the forward and inverse kinematics routine for the 5 DoF YouBot Manipulator (see the figure for a reference). 
In this package the next methods were programmed by me.

```
VectorXd youbot_kinematic::obtain_pose_vector(Matrix4d pose_matrix) 
// Transform from a 4D Transformation Matrix to a 6D pose vector
// The first three elementos are the position and the last three orientation

MatrixXd youbot_kinematic::get_jacobian(double joint_val[])
// Get the jacobian of each frame

double* youbot_kinematic::inverse_kine_ite(Matrix4d pose, double joint_val[])
// Compute the inverse kinematic iteratively

double* youbot_kinematic::norm_angle(double joint_val[])
// Normalize the angles between -pi and pi

bool youbot_kinematic::check_singularity(double joint_val[])
// Check if the trajectory falls into a singularity
     
double* youbot_kinematic::apply_offset(double joint_val[])
// Apply the offset for each joint

```

![image](https://user-images.githubusercontent.com/15234283/163908445-a49d417b-a4a5-49f1-851c-c66c59ed1edc.png)


## path_planning package
