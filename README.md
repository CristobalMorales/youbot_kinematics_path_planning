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

This package contains the path planning (dijkstra) routine for the 5 DoF YouBot Manipulator. It contains two differente trials. One with obstacles and the other without obstacles. In order to load each one the next argument in the path_planning.launch file should be modified.
```
  <arg name="question" default="1"/> // 1 for no obstacles, 2 otherwise
```

In this package the next methods were programmed by me.

```
MatrixXd quaternion_rotation(double q[]) 
// Compute the rotation matrix from a quaternion

bool intesect_object(double *point) 
// Determine if a 3D points intersects with an object

void generate_points(double intervals[3][2], double data[POINTS_DATA][3])
// Generates data points to use path planning algorithm

std::list<double*> connect_points(double data[POINTS_DATA+5][3])
// Connect points through the nearest neighbor and a distance limit


std::list<double*> dijkstra(std::list<double*> links, int init, int end)
// Apply the Dijkstra algorithm over the connections to reach the desired points
```

## path_planning package

This package contains the information of the trail (tracked points and trajectory line)
