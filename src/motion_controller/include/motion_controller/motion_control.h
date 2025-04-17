#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <base_control/base_wheel_vel.h>
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <vector>
#include <memory>
#include <string>
#include <sstream>
#include <algorithm>
#include <fstream>  
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
//#include <signal.h>


#define GREEN_TEXT "\033[1;92m"
#define RESET_COLOR "\033[0m"
using namespace casadi;



struct Point {
    double x;
    double y;
    double theta;  // orientation in radians
};

struct TrajectoryPoint {
    double x;
    double y;
    double theta;
    double time;   // time to reach this point (seconds from start)
};


class MPC
{
public:
    MPC(int N,float dt,float u_max_x, float u_max_y, float w_max);
    ~MPC();
    Function setSystemModel();
    void setWeights(std::vector<double> weights);
    //bool solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states);
    bool optimize(Eigen::VectorXd current_states, Eigen::MatrixXd desired_states);
    std::vector<double> getPredictU();
    std::vector<double> getPredictX();
    int getHorizon();

private:
    int N_;     
    double dt_; 
    double u_max_x_, u_max_y_, w_max_, u_min_x_, u_min_y_, w_min_;

    // weights
    DM Q_, R_, P_;

    MX X;
    MX U;

    Function kinematic_equation_;
    Function system_dynamics_;
    std::unique_ptr<casadi::OptiSol> solution_;
};

class PID
{
private:
    float kp_, ki_, kd_, max_output_;
    float prev_error_, integral_, prev_output_;
    float derivative_;
    float dt_; // Time step

public:
    PID(float kp, float ki, float kd, float dt, float max_output);
    float update(float error);
};

class Controller
{
private:
    ros::NodeHandle nh;
    ros::Subscriber PosFeedbackSub;
    ros::Publisher VelCmdPub;
    ros::Publisher prediction_viz_pub;
    Eigen::VectorXd current_state;
    std::unique_ptr<MPC> trajectory_planner;
    std::string waypoints_path = "/home/prox/CSV_Listen/Ahmet_path.csv";
    std::string trajectory_path = "/home/prox/CSV_Listen/robot_trajectory.csv";
    //std::vector<double> p_x, p_y, theta;
    std::vector<Point> waypoints;
    std::vector<TrajectoryPoint> desired_trajectory, local_trajectory;
    std::vector<Eigen::Vector3d> interpolated_velocities;
    std::vector<std::vector<float>> state_history;  // Store state history [px, py, theta, vx, vy, wz]
    std::vector<std::vector<float>> ref_history;    // Store reference history [vx_ref, vy_ref, wz_ref]
    std::string log_filename = "/home/prox/CSV_Listen/controller_log.csv";  // Default filename


public:
    volatile float vx_meas, vy_meas, wz_meas, px_meas, py_meas, theta_meas;
    const float MPC_dt = 1.f / 10.f;
    const int MPC_PredictionHorizon = 50, MPC_ControlHorizon=5;
    const float PID_dt = 1.f / 100.f;
    const float Controller_dt=PID_dt;
    const float max_wheel_speed=50.0;
    const float cruise_speed=1.0;
    const float u_max_x=2, u_max_y=2, w_max=1.25;
    
    Controller();
    void stateEstimatorCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void update_wheel_speeds(float vx, float vy, float wz);
    void startControlLoop();
    bool readWaypoints(const std::string &filePath, std::vector<Point> &waypoints);
    void printWaypoints(const std::vector<Point>& waypoints);
    void writeTrajectoryToCSV(const std::vector<TrajectoryPoint> &trajectory, const std::string &trajectory_path);
    std::vector<TrajectoryPoint> interpolateTrajectory(const std::vector<TrajectoryPoint> &trajectory,double timeStep);
    std::vector<TrajectoryPoint> generateTrajectory(const std::vector<Point> &waypoints, double linearSpeed);
    size_t selectNearestPoint(Eigen::MatrixXd& desired_states);
    void visualizePredictedTrajectory(const std::vector<double>& predicted_x);
    std::vector<Eigen::Vector3d> interpolateVelocities(const std::vector<double>& predicted_u);
    void logStates(const std::vector<float>& reference, const std::vector<float>& states);
    void saveLogFile();


    //Temporary fcn
    void systemIdentificationLoop();
};



#endif // MPC_CONTROLLER_H