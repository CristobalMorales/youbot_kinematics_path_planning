#include <youbot_fk_ik/youbotKine.h>

void youbot_kinematic::init()
{
    /**
     * @brief Initialize the DH parameters, the offset of the joints and the limits. 
     * Besides, subscribes to /joint_states topic
     * 
     */
    DH_params[0][0] = -0.033;   DH_params[1][0] = 0.155;  DH_params[2][0] = 0.135; DH_params[3][0] = -0.002;  DH_params[4][0] = 0.0;
    DH_params[0][1] = M_PI_2;   DH_params[1][1] = 0.0;    DH_params[2][1] = 0.0;   DH_params[3][1] = M_PI_2;  DH_params[4][1] = M_PI;
    DH_params[0][2] = 0.145;    DH_params[1][2] = 0.0;    DH_params[2][2] = 0.0;   DH_params[3][2] = 0.0;     DH_params[4][2] = -0.185;
    DH_params[0][3] = M_PI;     DH_params[1][3] = M_PI_2; DH_params[2][3] = 0.0;   DH_params[3][3] = -M_PI_2; DH_params[4][3] = M_PI;

    joint_offset[0] = 170*M_PI/180;
    joint_offset[1] = 65*M_PI/180;
    joint_offset[2] = -146*M_PI/180;
    joint_offset[3] = 102.5*M_PI/180;
    joint_offset[4] = 167.5*M_PI/180;

    joint_limit_min[0] = -169*M_PI/180;
    joint_limit_min[1] = -65*M_PI/180;
    joint_limit_min[2] = -150*M_PI/180;
    joint_limit_min[3] = -102.5*M_PI/180;
    joint_limit_min[4] = -167.5*M_PI/180;

    joint_limit_max[0] = 169*M_PI/180;
    joint_limit_max[1] = 90*M_PI/180;
    joint_limit_max[2] = 146*M_PI/180;
    joint_limit_max[3] = 102.5*M_PI/180;
    joint_limit_max[4] = 167.5*M_PI/180;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &youbot_kinematic::joint_state_callback, this);
}

void youbot_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for(int i = 0; i < 5; i++)
        current_joint_position[i] = q->position.at(i);

    current_pose = forward_kine(current_joint_position, 5);
    broadcast_pose(current_pose);
}

Matrix4d youbot_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
{
    Matrix4d A;
    A(3, 3) = 1.0;
    A(3, 2) = 0.0;
    A(3, 1) = 0.0;
    A(3, 0) = 0.0;

    A(0, 0) = cos(theta);
    A(0, 1) = -sin(theta)*cos(alpha);
    A(0, 2) = sin(theta)*sin(alpha);
    A(0, 3) = a * cos(theta);

    A(1, 0) = sin(theta);
    A(1, 1) = cos(theta)*cos(alpha);
    A(1, 2) = -cos(theta)*sin(alpha);
    A(1, 3) = a * sin(theta);

    A(2, 0) = 0.0;
    A(2, 1) = sin(alpha);
    A(2, 2) = cos(alpha);
    A(2, 3) = d;

    return A;
}

Matrix4d youbot_kinematic::forward_kine(double joint_val[], int frame)
{
    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_val[i] + DH_params[i][3]);
        T = T * A;
    }

    return T;
}

Matrix4d youbot_kinematic::forward_kine_offset(double joint_val[], int frame)
{

    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        if (i == 0)
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_offset[i] - (joint_val[i] + DH_params[i][3]));
        else
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], (joint_val[i] + DH_params[i][3]) - joint_offset[i]);

        T = T * A;
    }

    return T;
}

void youbot_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "base_link";
    T.child_frame_id = "arm_end_effector";

    pose_br.sendTransform(T);
}

VectorXd youbot_kinematic::obtain_pose_vector(Matrix4d pose_matrix) 
{
    /**
     * @brief Transform from a 4D Transformation Matrix to a 6D pose vector
     * The first three elementos are the position and the last three orientation
     * 
     * @param pose_matrix This is the pose matrix that representa a Transformation Matrix
     * 
     * @return Pose vector which is pose + orientation in radians (6-size vector)
     */
    VectorXd pose_vector(6);
    double target_angle = acos((pose_matrix(0,0) + pose_matrix(1,1) + pose_matrix(2,2) - 1)/2);
    pose_vector(0) = pose_matrix(0, 3);
    pose_vector(1) = pose_matrix(1, 3);
    pose_vector(2) = pose_matrix(2, 3);
    if (sin(target_angle) != 0)
    {
        pose_vector(3) = (1/(2*sin(target_angle))) * (pose_matrix(2,1) - pose_matrix(1,2));
        pose_vector(4) = (1/(2*sin(target_angle))) * (pose_matrix(0,2) - pose_matrix(2,0));
        pose_vector(5) = (1/(2*sin(target_angle))) * (pose_matrix(1,0) - pose_matrix(0,1));
    }
    else
    {
        pose_vector(3) = 0;
        pose_vector(4) = 0;
        pose_vector(5) = 0;
    }
    return pose_vector;
}

MatrixXd youbot_kinematic::get_jacobian(double joint_val[])
{
    /**
     * @brief Get the jacobian of each frame
     * 
     * @param joint_val The angle of each frame in radians
     * 
     * @return The jacobian (a matrix of 6 by 5)
     */
    MatrixXd jacobian(6,5);
    Matrix4d* forw_kin_matr = new Matrix4d[6];
    Vector3d* z_i = new Vector3d[6];
    Vector3d* o_i = new Vector3d[6];
    for (int frame = 0; frame <= 5; frame++) 
    {
        forw_kin_matr[frame] = forward_kine(joint_val, frame);
        z_i[frame] = forw_kin_matr[frame].block(0,2,3,1);
        o_i[frame] = forw_kin_matr[frame].block(0,3,3,1);
    }
    for (int frame = 1; frame <= 5; frame++) 
    {
        Vector3d J_pi = z_i[frame - 1].cross(o_i[5] - o_i[frame - 1]);
        Vector3d J_oi = z_i[frame - 1];
        for (int pos = 0; pos < 3; pos++) 
        {
            jacobian(pos, frame - 1) = J_pi(pos);
            jacobian(3 + pos, frame - 1) = J_oi(pos);
        }
    }
    return jacobian;
}

double* youbot_kinematic::inverse_kine_ite(Matrix4d pose, double joint_val[])
{
    /**
     * @brief Compute the inverse kinematic iteratively
     * 
     * @param pose Pose matrix of the target position
     * @param joint_val The angles of each joint in radians
     * 
     * @return The angles of each joint after the iteration
     */
    // Defining the gamma for updating the state and the error to stop iterate
    double gamma = 0.6;
    double error_var = 0.000001;
    // Auxiliar variables
    VectorXd target_pose(6);
    VectorXd current_pose(6);
    target_pose = obtain_pose_vector(pose);
    double* joint_val_post = new double[5];
    joint_val_post[0] = joint_val[0]; joint_val_post[1] = joint_val[1]; joint_val_post[2] = joint_val[2];
    joint_val_post[3] = joint_val[3]; joint_val_post[4] = joint_val[4];
    double vari = 1;
    double pose_error = 1, pose_error_prev = 0;
    VectorXd joint_val_post_aux(5);
    joint_val_post_aux << joint_val_post[0], joint_val_post[1], joint_val_post[2], joint_val_post[3], joint_val_post[4];
    int iter_n = 0;
    while (error_var < vari)
    {
        // Compute forward kinematic for the last joint
        Matrix4d current_pose_matrix = forward_kine(joint_val_post, 5);
        // Obtain the vector of the pose
        current_pose = obtain_pose_vector(current_pose_matrix);
        pose_error_prev = pose_error;
        if (check_singularity(joint_val_post))
            std::cout << "Is singularity?: " << check_singularity(joint_val_post) << std::endl;
        // Get the jacobian
        MatrixXd jacobian = get_jacobian(joint_val_post);
        MatrixXd Ide = MatrixXd::Identity(5, 5);
        // Updating the state through the Damped Least Square Method
        MatrixXd upd_step = (jacobian.transpose()*jacobian + (gamma*gamma)*Ide).inverse()*jacobian.transpose();
        if (iter_n > 10000)
        {
            break;
        }
        joint_val_post_aux = joint_val_post_aux + upd_step*(target_pose - current_pose);
        joint_val_post[0] = joint_val_post_aux(0); joint_val_post[1] = joint_val_post_aux(1); 
        joint_val_post[2] = joint_val_post_aux(2); joint_val_post[3] = joint_val_post_aux(3); 
        joint_val_post[4] = joint_val_post_aux(4); 
        pose_error = (target_pose - current_pose).norm();
        // Compute the variation between current and last state
        vari = abs(pose_error - pose_error_prev);
        iter_n++;
    }
    return joint_val_post;
}

/*double* youbot_kinematic::fix_limits(double joint_val[])
{
    bool limit_reached = false;
    for (int i = 0; i < 5; i++) 
    {
        if (joint_val[i] > joint_limit_max[i] || joint_val[i] < joint_limit_min[i]) 
            limit_reached = true;
            double interval = joint_limit_max[i] - joint_limit_min[i];
            double rand_num = ((double) rand() / (RAND_MAX));
            joint_val[i] = rand_num*interval + joint_limit_min[i];
    } 
    if (limit_reached){
        for (int i = 0; i < 5; i++){
            double interval = joint_limit_max[i] - joint_limit_min[i];
            double rand_num = ((double) rand() / (RAND_MAX));
            joint_val[i] = rand_num*interval + joint_limit_min[i];
        }
    }
}*/

double* youbot_kinematic::norm_angle(double joint_val[])
{
    /**
     * @brief Normalize the angles between -pi and pi
     * 
     * @param joint_val The angles of each joint in radians
     * 
     * @return The angles of each joint normalised in radians
     */
    double* new_joint = new double[5];
    for (int i = 0; i < 5; i++) 
    {
        int n_cycles = (int) joint_val[i] / (2*M_PI);
        new_joint[i] = joint_val[i] - ((double)n_cycles) * (M_PI);
        if (new_joint[i] > M_PI) 
        {
            new_joint[i] = new_joint[i] - 2*M_PI;
        }
        else if(new_joint[i] < -M_PI) 
        {
            new_joint[i] = new_joint[i] + 2*M_PI;
        }
    }
    return new_joint;
}

bool youbot_kinematic::check_singularity(double joint_val[])
{
    /**
     * @brief Check if the trajectory falls into a singularity
     * 
     * @param joint_val The angles of each joint in radians
     * 
     * @return Whether the joint angles of the manipulator fall into a singularity or not
     */
    MatrixXd jacobian = get_jacobian(joint_val);
    MatrixXd Ide = MatrixXd::Identity(5, 5);
    MatrixXd inverse = jacobian.transpose()*jacobian;
    return inverse.determinant() == 0;
}

double* youbot_kinematic::apply_offset(double joint_val[])
{
    /**
     * @brief Apply the offset for each joint
     * 
     * @param joint_val The angles of each joint in radians
     * 
     * @return The angles of each joint in radians plus the offset 
     */
    double *new_joint = new double[5];
    new_joint[0] = joint_offset[0] - joint_val[0]; 
    new_joint[1] = joint_val[1] + joint_offset[1];
    new_joint[2] = joint_val[2] + joint_offset[2];
    new_joint[3] = joint_val[3] + joint_offset[3];
    new_joint[4] = joint_val[4] + joint_offset[4];

    return new_joint;
}