#include "motion_controller/motion_control.h"

// MPC Class Implementation
MPC::MPC(int N, float dt, float u_max_x, float u_max_y, float w_max) : N_(N), dt_(dt), u_max_x_(u_max_x), u_max_y_(u_max_y), w_max_(w_max)
{
    std::vector<double> weights = {100, 100, 100, 0, 0, 0, 1, 1, 1, 100, 100, 100, 0, 0, 0}; // Q,R,P
    u_min_x_ = -u_max_x_;
    u_min_y_ = -u_max_y_;
    w_min_ = -w_max_;

    Q_ = DM::zeros(6, 6);
    R_ = DM::zeros(3, 3);
    P_ = DM::zeros(6, 6);

    setWeights(weights);
    system_dynamics_ = setSystemModel();
}

MPC::~MPC() {}

void MPC::setWeights(std::vector<double> weights)
{
    Q_(0, 0) = weights[0];
    Q_(1, 1) = weights[1];
    Q_(2, 2) = weights[2];
    Q_(3, 3) = weights[3];
    Q_(4, 4) = weights[4];
    Q_(5, 5) = weights[5];

    R_(0, 0) = weights[6];
    R_(1, 1) = weights[7];
    R_(2, 2) = weights[8];

    P_(0, 0) = weights[9];
    P_(1, 1) = weights[10];
    P_(2, 2) = weights[11];
    P_(3, 3) = weights[12];
    P_(4, 4) = weights[13];
    P_(5, 5) = weights[14];
}

Function MPC::setSystemModel()
{
    // Symbolic variables for states
    MX p_x = MX::sym("p_x");
    MX p_y = MX::sym("p_y");
    MX theta = MX::sym("theta");
    MX v_x = MX::sym("v_x");
    MX v_y = MX::sym("v_y");
    MX w = MX::sym("w");

    MX state_vars = MX::vertcat({p_x, p_y, theta, v_x, v_y, w});

    MX v_x_ref = MX::sym("v_x_ref");
    MX v_y_ref = MX::sym("v_y_ref");
    MX w_ref = MX::sym("w_ref");
    MX control_vars = MX::vertcat({v_x_ref, v_y_ref, w_ref});

    const double tau_vx = 0.1;
    const double tau_vy = 0.1;
    const double tau_w = 0.1;

    MX R_z = MX::vertcat({MX::horzcat({MX::cos(theta), -MX::sin(theta), MX::zeros(1, 1)}),
                          MX::horzcat({MX::sin(theta), MX::cos(theta), MX::zeros(1, 1)}),
                          MX::horzcat({MX::zeros(1, 1), MX::zeros(1, 1), MX::ones(1, 1)})});
    MX G = dt_ * R_z;

    const double a1 = std::exp(-dt_ / tau_vx);
    const double a2 = std::exp(-dt_ / tau_vy);
    const double a3 = std::exp(-dt_ / tau_w);

    MX A_v = MX::diag(MX::vertcat({a1, a2, a3}));

    MX A = MX::vertcat({MX::horzcat({MX::eye(3), G}),
                        MX::horzcat({MX::zeros(3, 3), A_v})});

    const double b1 = (1 - std::exp(-dt_ / tau_vx));
    const double b2 = (1 - std::exp(-dt_ / tau_vy));
    const double b3 = (1 - std::exp(-dt_ / tau_w));

    MX B_v = MX::diag(MX::vertcat({b1, b2, b3}));

    MX B = MX::vertcat({MX::zeros(3, 3),
                        B_v});

    MX rhs = mtimes(A, state_vars) + mtimes(B, control_vars);

    return Function("system_dynamics", {state_vars, control_vars}, {rhs});
}

bool MPC::optimize(Eigen::VectorXd current_states, Eigen::MatrixXd desired_states)
{
    const int n_states = 6;
    const int n_controls = 3;
    Opti opti = Opti();

    Slice all;

    MX cost = 0;
    // 6 states, N+1 time steps
    X = opti.variable(n_states, N_ + 1);
    // 3 inputs, N time steps
    U = opti.variable(n_controls, N_);

    // Extract state variables
    MX p_x = X(0, all);
    MX p_y = X(1, all);
    MX theta = X(2, all);
    MX v_x = X(3, all);
    MX v_y = X(4, all);
    MX w = X(5, all);

    // Extract input variables
    MX v_x_ref = U(0, all);
    MX v_y_ref = U(1, all);
    MX w_ref = U(2, all);

    // Reference trajectory and current state
    MX X_ref = opti.parameter(6, N_ + 1);
    MX X_cur = opti.parameter(6);

    // Safely convert current states to CasADi matrix
    if (current_states.size() != 6)
    {
        ROS_ERROR("Current states vector must be exactly 6 elements long!");
        return false;
    }

    // Create a vector of doubles from Eigen vector
    std::vector<double> x_tmp_v(current_states.data(), current_states.data() + current_states.size());
    DM x_tmp1 = x_tmp_v;

    opti.set_value(X_cur, x_tmp1);

    // Convert desired states to CasADi matrix
    if (desired_states.rows() != 6 || desired_states.cols() != N_ + 1)
    {
        ROS_ERROR("Desired states matrix must be 6x(N+1)!");
        return false;
    }

    std::vector<double> X_ref_v(desired_states.data(),
                                desired_states.data() + desired_states.size());
    DM X_ref_d(X_ref_v);
    X_ref = MX::reshape(X_ref_d, n_states, N_ + 1);

    // Cost function
    for (int i = 0; i < N_; ++i)
    {
        MX X_err = X(all, i) - X_ref(all, i);
        MX U_0 = U(all, i);

        // State error cost
        cost += MX::mtimes({X_err.T(), Q_, X_err});

        // Control input cost
        cost += MX::mtimes({U_0.T(), R_, U_0});
    }

    // Terminal cost
    cost += MX::mtimes({(X(all, N_) - X_ref(all, N_)).T(), P_,
                        X(all, N_) - X_ref(all, N_)});

    opti.minimize(cost);

    // Dynamics constraints
    for (int i = 0; i < N_; ++i)
    {
        std::vector<MX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);

        MX X_next = system_dynamics_(input)[0];
        opti.subject_to(X_next == X(all, i + 1));
    }

    // Initial state constraint
    opti.subject_to(X(all, 0) == X_cur);

    // Input constraints
    opti.subject_to(-u_max_x_ <= v_x_ref <= u_max_x_);
    opti.subject_to(-u_max_y_ <= v_y_ref <= u_max_y_);
    opti.subject_to(-w_max_ <= w_ref <= w_max_);

    // Solver configuration
    casadi::Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 10000;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 1e-6;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    opti.solver("ipopt", solver_opts);

    // Solve the optimization problem
    solution_ = std::make_unique<casadi::OptiSol>(opti.solve());

    return true;
}

std::vector<double> MPC::getPredictU()
{
    std::vector<double> u_res;
    auto vel_cmd = solution_->value(U);

    for (int i = 0; i < N_; ++i)
    {
        u_res.push_back(static_cast<double>(vel_cmd(0, i)));
        u_res.push_back(static_cast<double>(vel_cmd(1, i)));
        u_res.push_back(static_cast<double>(vel_cmd(2, i)));
    }

    return u_res;
}

std::vector<double> MPC::getPredictX()
{
    std::vector<double> res;
    auto predict_x = solution_->value(X);

    for (int i = 0; i <= N_; ++i)
    {
        res.push_back(static_cast<double>(predict_x(0, i)));
        res.push_back(static_cast<double>(predict_x(1, i)));
        res.push_back(static_cast<double>(predict_x(2, i)));
        res.push_back(static_cast<double>(predict_x(3, i)));
        res.push_back(static_cast<double>(predict_x(4, i)));
        res.push_back(static_cast<double>(predict_x(5, i)));
    }
    return res;
}

int MPC::getHorizon()
{
    return N_;
}

// PID Class Implementation
PID::PID(float kp, float ki, float kd, float dt, float max_output)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0), prev_output_(0.0), dt_(dt), max_output_(max_output) {}

float PID::update(float error)
{
    // TODO: ADD ANTI WIND-UP FOR INTEGRAL
    /*if (((prev_output_ > 0 && error > 0) || (prev_output_ < 0 && error < 0)) && (prev_output_ * prev_output_) >= max_output_sqr_)
    {
        integral_ = 0;
        ROS_ERROR("Integrator Wind-up!!");
    }
    else
    {
        integral_ += error;
    }*/

    integral_ += error * dt_;
    derivative_ = (error - prev_error_) / dt_;
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative_;
    prev_error_ = error;
    prev_output_ = output;
    if (output > max_output_){
        //ROS_ERROR("PID Clamped!");
        return max_output_;
    }
    else if(output < -max_output_){
        //ROS_ERROR("PID Clamped!");
        return -max_output_;
    }
    else{
        return output;
    }
    
}

// Controller Class Implementation
Controller::Controller() : vx_meas(0.0), vy_meas(0.0), wz_meas(0.0), px_meas(0.0), py_meas(0.0), theta_meas(0.0)
{
    VelCmdPub = nh.advertise<base_control::base_wheel_vel>("robot_wheel_command", 10);
    PosFeedbackSub = nh.subscribe("/rtabmap/odom", 10, &Controller::stateEstimatorCallback, this);
    prediction_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("predicted_trajectory", 1);

    trajectory_planner = std::make_unique<MPC>(MPC_PredictionHorizon, MPC_dt, u_max_x, u_max_y, w_max);

    if (readWaypoints(waypoints_path, waypoints))
    {
        printWaypoints(waypoints);
    }
    // Get desired trajectory with original timestamps.
    std::vector<TrajectoryPoint> desired_trajectory_tmp = generateTrajectory(waypoints, cruise_speed);

    // Resample and write to csv.
    desired_trajectory = interpolateTrajectory(desired_trajectory_tmp, MPC_dt);
    writeTrajectoryToCSV(desired_trajectory, trajectory_path);
}

void Controller::saveLogFile()
{   
    // Save logs to file when the controller is destroyed
    if (!state_history.empty() && !ref_history.empty()) {
        std::ofstream file(log_filename);
        if (file.is_open()) {
            // Write CSV header
            file << "idx,px,py,theta,vx,vy,wz,vx_ref,vy_ref,wz_ref\n";
            
            // Write data rows
            for (size_t i = 0; i < state_history.size(); ++i) {
                file << i;
                
                // Write states
                for (size_t j = 0; j < state_history[i].size(); ++j) {
                    file << "," << state_history[i][j];
                }
                
                // Write references
                for (size_t j = 0; j < ref_history[i].size(); ++j) {
                    file << "," << ref_history[i][j];
                }
                
                file << "\n";
            }
            
            file.close();
            ROS_INFO("Controller log saved to %s (%zu entries)", log_filename.c_str(), state_history.size());
        } else {
            ROS_ERROR("Could not open log file: %s", log_filename.c_str());
        }
    } else {
        ROS_WARN("No data to save to log file");
    }
    
    // Any other cleanup needed for the controller
}

void Controller::logStates(const std::vector<float>& reference, const std::vector<float>& states)
{
    // Validate input sizes
    if (reference.size() != 3 || states.size() != 6) {
        ROS_WARN("Invalid vector sizes: reference=%zu (expect 3), states=%zu (expect 6)", 
                reference.size(), states.size());
        return;
    }
    
    // Store the data
    ref_history.push_back(reference);
    state_history.push_back(states);
}

void Controller::stateEstimatorCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    /*
        // Store current time
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    
    // Calculate time difference and frequency
    double dt = (current_time - last_time).toSec();
    double frequency = (dt > 0) ? 1.0 / dt : 0.0;
    
    // Update last_time for the next call
    last_time = current_time;
    */
    px_meas = msg->pose.pose.position.x;
    py_meas = msg->pose.pose.position.y;

    double w_tmp = msg->pose.pose.orientation.w;
    double x_tmp = msg->pose.pose.orientation.x;
    double y_tmp = msg->pose.pose.orientation.y;
    double z_tmp = msg->pose.pose.orientation.z;

    theta_meas = std::atan2(2 * (w_tmp * z_tmp + x_tmp * y_tmp), 1 - 2 * (y_tmp * y_tmp + z_tmp * z_tmp));

    vx_meas = msg->twist.twist.linear.x;
    vy_meas = msg->twist.twist.linear.y;
    wz_meas = msg->twist.twist.angular.z;

    // ROS_INFO("Velocity -> V_x: %.2f, V_y: %.2f, w: %.2f", vx_meas, vy_meas, wz_meas);
    //ROS_INFO("Callback Frequency: %.2f Hz (dt = %.4f sec)", frequency, dt);
}

void Controller::update_wheel_speeds(float vx, float vy, float wz)
{
    base_control::base_wheel_vel wheel_speed_command;

    /*w1:Front left
      w2:Front Right
      w3:Rear Left
      w4:Rear Right*/
    const float Lx = 0.65, Ly = 0.65, Rw = 0.125, w_max_sqr = 400;

    float w1 = (vx - vy - (Lx + Ly) * wz) / Rw;
    float w2 = (vx + vy + (Lx + Ly) * wz) / Rw;
    float w3 = (vx + vy - (Lx + Ly) * wz) / Rw;
    float w4 = (vx - vy + (Lx + Ly) * wz) / Rw;

    float max_wheel_speed_curr = fmax(fmax(fabs(w1), fabs(w2)), fmax(fabs(w3), fabs(w4)));

    // Scale if necessary
    if (max_wheel_speed_curr > max_wheel_speed)
    {
        ROS_ERROR("Wheel speed limit exceeded!!\n\n Front Left Wheel: %.2f\n Front Right Wheel: %.2f\n Rear Left Wheel:%.2f\n Rear Right Wheel: %.2f\n", w1, w2, w3, w4);
        float scale_factor = max_wheel_speed / max_wheel_speed_curr;
        w1 *= scale_factor;
        w2 *= scale_factor;
        w3 *= scale_factor;
        w4 *= scale_factor;
    }

    // Set wheel speeds
    wheel_speed_command.w1 = w1;
    wheel_speed_command.w2 = w2;
    wheel_speed_command.w3 = w3;
    wheel_speed_command.w4 = w4;

    // Publish the wheel speeds
    VelCmdPub.publish(wheel_speed_command);
}

// New function to select the nearest point in trajectory
size_t Controller::selectNearestPoint(Eigen::MatrixXd &desired_states)
{
    // Find the trajectory point closest to current position (x,y)
    size_t currentIndex = 0;
    double closestDistanceSq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < desired_trajectory.size(); ++i)
    {
        double dx = desired_trajectory[i].x - px_meas;
        double dy = desired_trajectory[i].y - py_meas;
        double distanceSq = dx * dx + dy * dy; // Squared distance (no need for sqrt)

        if (distanceSq < closestDistanceSq)
        {
            closestDistanceSq = distanceSq;
            currentIndex = i;
        }
    }

    // Calculate lookahead index - start from closest point and look ahead
    // This prevents the robot from backtracking if it misses a point
    size_t startIndex = currentIndex;

    // To prevent oscillation between points, we can use a simple lookahead
    // If we've passed some point of the trajectory, we should follow the path forward
    if (startIndex + 1 < desired_trajectory.size())
    {
        // Check if we're past the closest point (dot product with path direction)
        double path_dx = desired_trajectory[startIndex + 1].x - desired_trajectory[startIndex].x;
        double path_dy = desired_trajectory[startIndex + 1].y - desired_trajectory[startIndex].y;
        double robot_dx = px_meas - desired_trajectory[startIndex].x;
        double robot_dy = py_meas - desired_trajectory[startIndex].y;

        double dot_product = path_dx * robot_dx + path_dy * robot_dy;

        // If dot product is positive, we're already progressing along the path
        // so we should target the next point
        if (dot_product > 0 && std::sqrt(path_dx * path_dx + path_dy * path_dy) > 0.01)
        {
            startIndex++;
        }
    }

    // Ensure we have enough points for the prediction horizon
    if (startIndex + trajectory_planner->getHorizon() >= desired_trajectory.size())
    {
        // We're near the end of the trajectory and don't have enough points
        // Use the available points and repeat the last point if necessary
        for (int i = 0; i <= trajectory_planner->getHorizon(); ++i)
        {
            size_t idx = std::min(startIndex + i, desired_trajectory.size() - 1);
            desired_states(0, i) = desired_trajectory[idx].x;
            desired_states(1, i) = desired_trajectory[idx].y;
            desired_states(2, i) = desired_trajectory[idx].theta;
            desired_states(3, i) = 0.0; // Desired velocity x
            desired_states(4, i) = 0.0; // Desired velocity y
            desired_states(5, i) = 0.0; // Desired angular velocity
        }
    }
    else
    {
        // We have enough points for the full horizon
        for (int i = 0; i <= trajectory_planner->getHorizon(); ++i)
        {
            desired_states(0, i) = desired_trajectory[startIndex + i].x;
            desired_states(1, i) = desired_trajectory[startIndex + i].y;
            desired_states(2, i) = desired_trajectory[startIndex + i].theta;
            desired_states(3, i) = 0.0; // Desired velocity x
            desired_states(4, i) = 0.0; // Desired velocity y
            desired_states(5, i) = 0.0; // Desired angular velocity
        }
    }

    return currentIndex;
}
void Controller::visualizePredictedTrajectory(const std::vector<double> &predicted_x)
{
    if (predicted_x.empty())
    {
        return;
    }

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker path_line_strip;
    path_line_strip.header.frame_id = "map";
    path_line_strip.header.stamp = ros::Time::now();
    path_line_strip.ns = "original_path";
    path_line_strip.id = 0;
    path_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    path_line_strip.action = visualization_msgs::Marker::ADD;
    path_line_strip.pose.orientation.w = 1.0;
    path_line_strip.scale.x = 0.02; // Line width
    // Red color for original path
    path_line_strip.color.r = 1.0;
    path_line_strip.color.g = 1.0;
    path_line_strip.color.a = 0.8;

    // Add points from the waypoints vector
    for (const auto &point : waypoints)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.05;
        path_line_strip.points.push_back(p);
    }

    marker_array.markers.push_back(path_line_strip);

    // 2. Create markers for each pose in the predicted trajectory (BLUE arrows)
    for (size_t i = 0; i < predicted_x.size(); i += 18) //instead of i+=6 for downsampling at 1/3rd rate
    {
        visualization_msgs::Marker pose_marker;
        pose_marker.header.frame_id = "map";
        pose_marker.header.stamp = ros::Time::now();
        pose_marker.ns = "predicted_poses";
        pose_marker.id = i / 6;
        pose_marker.type = visualization_msgs::Marker::ARROW;
        pose_marker.action = visualization_msgs::Marker::ADD;

        // Position
        pose_marker.pose.position.x = predicted_x[i];
        pose_marker.pose.position.y = predicted_x[i + 1];
        pose_marker.pose.position.z = 0.1;

        // Orientation (from theta)
        tf::Quaternion q;
        q.setRPY(0, 0, predicted_x[i + 2]);
        pose_marker.pose.orientation.x = q.x();
        pose_marker.pose.orientation.y = q.y();
        pose_marker.pose.orientation.z = q.z();
        pose_marker.pose.orientation.w = q.w();

        // Size
        pose_marker.scale.x = 0.2;  // Arrow length
        pose_marker.scale.y = 0.05; // Arrow width
        pose_marker.scale.z = 0.05; // Arrow height

        // Blue color
        pose_marker.color.b = 1.0;
        pose_marker.color.a = 0.7;

        marker_array.markers.push_back(pose_marker);
    }

    // Publish the marker array
    prediction_viz_pub.publish(marker_array);
}

std::vector<Eigen::Vector3d> Controller::interpolateVelocities(const std::vector<double> &predicted_u)
{
    std::vector<Eigen::Vector3d> interpolated_velocities;

    // Calculate interpolation factor (number of PID steps per MPC step)
    int interpolation_factor = static_cast<int>(MPC_dt / PID_dt);

    // Ensure we have sufficient predicted states
    if (predicted_u.empty() || predicted_u.size() < 3)
    {
        ROS_ERROR("Not enough inputs for interpolation!");
        return interpolated_velocities;
    }

    // Calculate how many complete MPC steps we can use
    int available_mpc_steps = std::min(
        MPC_PredictionHorizon,
        static_cast<int>(predicted_u.size() / 3) - 1 // -1 because we need pairs of points
    );

    for (int step = 0; step < available_mpc_steps; ++step)
    {
        // Extract current and next velocity states
        Eigen::Vector3d current_vel(
            predicted_u[step * 3 + 0], // vx at current step
            predicted_u[step * 3 + 1], // vy at current step
            predicted_u[step * 3 + 2]  // w at current step
        );

        Eigen::Vector3d next_vel(
            predicted_u[(step + 1) * 3 + 0], // vx at next step
            predicted_u[(step + 1) * 3 + 1], // vy at next step
            predicted_u[(step + 1) * 3 + 2]  // w at next step
        );

        // Create interpolated points
        for (int i = 0; i < interpolation_factor; ++i)
        {
            double alpha = static_cast<double>(i) / interpolation_factor;
            Eigen::Vector3d interpolated = current_vel + alpha * (next_vel - current_vel);
            interpolated_velocities.push_back(interpolated);
        }
    }

    // Add the final velocity point
    if (available_mpc_steps > 0)
    {
        interpolated_velocities.push_back(Eigen::Vector3d(
            predicted_u[available_mpc_steps * 3 + 0],
            predicted_u[available_mpc_steps * 3 + 1],
            predicted_u[available_mpc_steps * 3 + 2]));
    }

    return interpolated_velocities;
}

void Controller::systemIdentificationLoop()
{
    PID PID_Vx(0.05,0,0,PID_dt,u_max_x);
    PID PID_Vy(0.05,0,0,PID_dt,u_max_y);
    PID PID_w(0.04,0,0,PID_dt,w_max);


    ros::Rate rate(1 / PID_dt);
    float Vx_des, Vy_des, w_des;
    int loop_ctr=0;
    bool flag = false;
    while (ros::ok())
    {
        if (loop_ctr > 500){
            break;
        }

        if (loop_ctr % 100 == 0){
            if (flag == false){
                Vx_des=1;
                Vy_des=0;
                w_des=0;
                flag = true;
            }
            else{
                Vx_des=-1;
                Vy_des=0;
                w_des=0;
                flag = false;
            } 
        }

        //float Vx_ref = PID_Vx.update(Vx_des - vx_meas);
        //float Vy_ref = PID_Vy.update(Vy_des - vy_meas);
        //float w_ref = PID_w.update(w_des - wz_meas);

 
        //update_wheel_speeds(Vx_ref, Vy_ref, w_ref);
        update_wheel_speeds(Vx_des, Vy_des, w_des);

        //ROS_INFO("Vx_des: %f,Vx_ref: %f, Vx_meas: %f, ctr=%d\n ",Vx_des,Vx_ref,vx_meas,ctr);
        logStates({Vx_des, Vy_des, w_des},{px_meas,py_meas,theta_meas,vx_meas,vy_meas,wz_meas});
        loop_ctr++;
        ros::spinOnce();
        rate.sleep();
    }
    saveLogFile();
}




void Controller::startControlLoop()
{
    
    PID PID_Vx(0.5,0,0.4,PID_dt,u_max_x);
    PID PID_Vy(0.5,0,0.4,PID_dt,u_max_y);
    PID PID_w(0.4,0,0.2,PID_dt,w_max);

    // Get the end time of the trajectory
    double endTime = desired_trajectory.back().time;

    // Initialize matrices for desired states
    Eigen::MatrixXd desired_states(6, trajectory_planner->getHorizon() + 1);

    // Variables to store predicted control inputs and states
    std::vector<double> predicted_u;
    std::vector<double> predicted_x;
    //int current_u_index = 0;
    //bool have_valid_controls = false;

    ros::Rate rate(1 / Controller_dt);

    size_t Loop_Ctr = 0;
    std::cout << std::endl << GREEN_TEXT << "Motion control has started!" << RESET_COLOR << std::endl;
    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        
        Eigen::VectorXd current_state(6);
        current_state << px_meas, py_meas, theta_meas, vx_meas, vy_meas, wz_meas;

        if (Loop_Ctr % (static_cast<int>(MPC_ControlHorizon * MPC_dt / PID_dt)) == 0)
        {
            // Select nearest point and fill desired_states
            size_t currentIndex = selectNearestPoint(desired_states);
            
            // Run MPC optimization
            bool success = trajectory_planner->optimize(current_state, desired_states);
            if (success)
            {
                // Get all predicted control inputs and states
                predicted_u = trajectory_planner->getPredictU();
                predicted_x = trajectory_planner->getPredictX();

                // Interpolate velocities for PID control
                interpolated_velocities = interpolateVelocities(predicted_u);

                // Visualize predicted trajectory
                visualizePredictedTrajectory(predicted_x);
            }
            else
            {
                ROS_ERROR("MPC optimization failed!");
                update_wheel_speeds(0.0, 0.0, 0.0); // Safety stop on optimization failure
            }
            Loop_Ctr=0;
        }
        
        // Make sure we have valid interpolated velocities before using them
        if (!interpolated_velocities.empty() && Loop_Ctr < interpolated_velocities.size()) {
            float Vx_ref = PID_Vx.update(interpolated_velocities[Loop_Ctr][0] - vx_meas);
            float Vy_ref = PID_Vy.update(interpolated_velocities[Loop_Ctr][1] - vy_meas);
            float w_ref = PID_w.update(interpolated_velocities[Loop_Ctr][2] - wz_meas);

            // Log the applied velocities with timestamp
            //ros::Duration elapsed = ros::Time::now() - current_time;
            //ROS_INFO("Applied: Vx=%.3f, Vy=%.3f, w=%.3f | Target: Vx=%.3f, Vy=%.3f, w=%.3f | Current: Vx=%.3f, Vy=%.3f, w=%.3f",
            //        Vx_ref, Vy_ref, w_ref,
            //        interpolated_velocities[Loop_Ctr][0], interpolated_velocities[Loop_Ctr][1], interpolated_velocities[Loop_Ctr][2],
            //        vx_meas, vy_meas, wz_meas);
            
            update_wheel_speeds(Vx_ref, Vy_ref, w_ref);
            //logStates({Vx_ref, Vy_ref, w_ref}, {px_meas, py_meas, theta_meas, vx_meas, vy_meas, wz_meas});
            
        } else {
            ROS_WARN("No valid interpolated velocities available or Loop_Ctr out of bounds (%zu / %zu)", 
                    Loop_Ctr, interpolated_velocities.size());
            update_wheel_speeds(0.0, 0.0, 0.0);
        }

        Loop_Ctr++;
        
        // Process ROS callbacks and sleep until next cycle
        ros::spinOnce();
        rate.sleep();
    }
}


bool Controller::readWaypoints(const std::string &filePath, std::vector<Point> &waypoints)
{
    // Clear vectors in case they contain data
    waypoints.clear();

    // Open file
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        ROS_ERROR("Could not open waypoint file!");
        return false;
    }

    std::string line;
    bool headerProcessed = false;

    while (std::getline(file, line))
    {
        // Process the header row
        if (!headerProcessed)
        {
            headerProcessed = true;
            std::stringstream headerStream(line);
            std::string firstColumn;

            // Get the first column header
            if (std::getline(headerStream, firstColumn, ','))
            {
                // Check if it's a trajectory file
                if (firstColumn == "time")
                {
                    ROS_ERROR("This is a trajectory file, not a path file!");
                    file.close();
                    return false;
                }
                // Check if it's a valid path file
                else if (firstColumn != "X")
                {
                    ROS_ERROR("Invalid path file structure. The column headers should be in the [X, Y , Angle] order!");
                    file.close();
                    return false;
                }
                // Correct format, continue processing
                continue;
            }
        }

        std::stringstream ss(line);
        std::string cell;
        double val1, val2, val3;

        // Read first column (X)
        if (!std::getline(ss, cell, ','))
        {

            std::cerr << "Error: Failed to read X position!" << std::endl;
            continue;
        }
        try
        {
            val1 = std::stod(cell);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error converting '" << cell << "' to double: " << e.what() << std::endl;
            continue;
        }

        // Read second column (Y)
        if (!std::getline(ss, cell, ','))
        {
            std::cerr << "Error: Failed to read Y position!" << std::endl;
            continue;
        }
        try
        {
            val2 = std::stod(cell);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error converting '" << cell << "' to double: " << e.what() << std::endl;
            continue;
        }

        // Read third column (Angle)
        if (!std::getline(ss, cell))
        {
            std::cerr << "Error: Failed to read the rotation angle!" << std::endl;
            continue;
        }
        try
        {
            val3 = std::stod(cell);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error converting '" << cell << "' to double: " << e.what() << std::endl;
            continue;
        }

        // Store values in respective vectors
        waypoints.push_back({val1, val2, val3});
    }

    file.close();
    std::cout << "Successfully read " << waypoints.size() << " rows from CSV file." << std::endl;
    return true;
}

void Controller::printWaypoints(const std::vector<Point> &waypoints)
{

    // Print header
    std::cout << std::string(50, '-') << std::endl;
    std::cout << std::setw(6) << "Index" << " | "
              << std::setw(15) << "p_x (m)" << " | "
              << std::setw(15) << "p_y (m)" << " | "
              << std::setw(15) << "theta (rad)" << std::endl;
    std::cout << std::string(50, '-') << std::endl;

    // Set floating point precision for output
    std::cout << std::fixed << std::setprecision(4);

    // Print data rows
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        std::cout << std::setw(6) << i << " | "
                  << std::setw(15) << waypoints[i].x << " | "
                  << std::setw(15) << waypoints[i].y << " | "
                  << std::setw(15) << waypoints[i].theta << std::endl;
    }

    // Print footer
    std::cout << std::string(50, '-') << std::endl;
    std::cout << "Total entries: " << waypoints.size() << std::endl;
}

std::vector<TrajectoryPoint> Controller::generateTrajectory(const std::vector<Point> &waypoints, double linearSpeed)
{
    if (waypoints.size() < 2)
    {
        std::cerr << "Error: At least two waypoints are required for trajectory generation." << std::endl;
        return {};
    }

    std::vector<TrajectoryPoint> trajectory;

    // Add the first waypoint at time 0
    trajectory.push_back({waypoints[0].x, waypoints[0].y, waypoints[0].theta, 0.0});

    double cumulativeTime = 0.0;
    double totalDistance = 0.0;

    // Calculate distances between consecutive waypoints and add trajectory points
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
        // Calculate Euclidean distance between consecutive points
        double dx = waypoints[i].x - waypoints[i - 1].x;
        double dy = waypoints[i].y - waypoints[i - 1].y;
        double segmentDistance = std::sqrt(dx * dx + dy * dy);

        // Calculate time needed to travel this segment at constant speed
        double segmentTime = segmentDistance / linearSpeed;
        cumulativeTime += segmentTime;
        totalDistance += segmentDistance;

        // Create trajectory point with timing information
        trajectory.push_back({waypoints[i].x,
                              waypoints[i].y,
                              waypoints[i].theta,
                              cumulativeTime});
    }

    std::cout << "Trajectory generated with " << trajectory.size() << " points." << std::endl;
    std::cout << "Total distance: " << totalDistance << " meters." << std::endl;
    std::cout << "Total time: " << cumulativeTime << " seconds." << std::endl;

    return trajectory;
}

std::vector<TrajectoryPoint> Controller::interpolateTrajectory(
    const std::vector<TrajectoryPoint> &trajectory,
    double timeStep)
{
    if (trajectory.empty())
    {
        return {};
    }

    std::vector<TrajectoryPoint> interpolatedTrajectory;

    // Start time
    double startTime = trajectory.front().time;
    // End time
    double endTime = trajectory.back().time;

    // Generate points at regular intervals
    for (double t = startTime; t <= endTime; t += timeStep)
    {
        // Find the two points to interpolate between
        auto it = std::lower_bound(
            trajectory.begin(),
            trajectory.end(),
            t,
            [](const TrajectoryPoint &point, double time)
            {
                return point.time < time;
            });

        // Handle edge cases
        if (it == trajectory.begin())
        {
            interpolatedTrajectory.push_back(trajectory.front());
            continue;
        }

        if (it == trajectory.end())
        {
            interpolatedTrajectory.push_back(trajectory.back());
            continue;
        }

        // Get the two points for interpolation
        const TrajectoryPoint &p2 = *it;
        const TrajectoryPoint &p1 = *(it - 1);

        // Calculate interpolation factor
        double alpha = (t - p1.time) / (p2.time - p1.time);

        // Linear interpolation
        TrajectoryPoint interpolated;
        interpolated.x = p1.x + alpha * (p2.x - p1.x);
        interpolated.y = p1.y + alpha * (p2.y - p1.y);

        // Angular interpolation (be careful with angle wrapping)
        double angleDiff = p2.theta - p1.theta;
        // Normalize to [-pi, pi]
        if (angleDiff > M_PI)
            angleDiff -= 2 * M_PI;
        if (angleDiff < -M_PI)
            angleDiff += 2 * M_PI;

        interpolated.theta = p1.theta + alpha * angleDiff;
        interpolated.time = t;

        interpolatedTrajectory.push_back(interpolated);
    }

    return interpolatedTrajectory;
}

void Controller::writeTrajectoryToCSV(const std::vector<TrajectoryPoint> &trajectory, const std::string &trajectory_path)
{
    std::ofstream file(trajectory_path);
    if (!file.is_open())
    {
        ROS_ERROR("Could not open trajectory file!");
        return;
    }

    // Write header
    file << "time,x,y,theta\n";

    // Write data
    for (const auto &point : trajectory)
    {
        file << point.time << ","
             << point.x << ","
             << point.y << ","
             << point.theta << "\n";
    }

    file.close();
    std::cout << "Trajectory written to " << trajectory_path << std::endl;
}



// Main Function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_control");

    Controller controller;
    controller.startControlLoop();
    //controller.systemIdentificationLoop();
    //ros::spin();

    return 0;
}
