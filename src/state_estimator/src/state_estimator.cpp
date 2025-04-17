#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class KalmanFilterFusion
{
private:
    // ROS Node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers
    ros::Publisher fused_odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Subscribers
    ros::Subscriber rtabmap_odom_sub_;
    ros::Subscriber imu_sub_;
    
    // Kalman filter state
    Eigen::VectorXd x_; // State: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]'
    Eigen::MatrixXd P_; // State covariance
    Eigen::MatrixXd Q_; // Process noise covariance
    Eigen::MatrixXd R_odom_; // Odometry measurement noise covariance
    Eigen::MatrixXd R_imu_; // IMU measurement noise covariance
    
    // Last update time
    ros::Time last_update_time_;
    bool first_odom_received_;
    bool first_imu_received_;
    
    // Frame IDs
    std::string odom_frame_id_;
    std::string base_frame_id_;
    
    // Kalman filter parameters
    double odom_pos_noise_;
    double odom_rot_noise_;
    double imu_acc_noise_;
    double imu_gyro_noise_;
    double process_noise_pos_;
    double process_noise_rot_;
    double process_noise_vel_;
    double process_noise_accel_;
    
public:
    KalmanFilterFusion() : private_nh_("~"), first_odom_received_(false), first_imu_received_(false)
    {
        // Load parameters
        private_nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        
        private_nh_.param<double>("odom_pos_noise", odom_pos_noise_, 0.01);
        private_nh_.param<double>("odom_rot_noise", odom_rot_noise_, 0.01);
        private_nh_.param<double>("imu_acc_noise", imu_acc_noise_, 0.01);
        private_nh_.param<double>("imu_gyro_noise", imu_gyro_noise_, 0.005);
        private_nh_.param<double>("process_noise_pos", process_noise_pos_, 0.01);
        private_nh_.param<double>("process_noise_rot", process_noise_rot_, 0.01);
        private_nh_.param<double>("process_noise_vel", process_noise_vel_, 0.05);
        private_nh_.param<double>("process_noise_accel", process_noise_accel_, 0.05);
        
        // Initialize Kalman filter state (13 elements)
        x_ = Eigen::VectorXd::Zero(13);
        P_ = Eigen::MatrixXd::Identity(13, 13);
        
        // Initial covariance values
        P_.block<3, 3>(0, 0) *= 0.1; // Position
        P_.block<4, 4>(3, 3) *= 0.1; // Orientation (quaternion)
        P_.block<3, 3>(7, 7) *= 0.5; // Linear velocity
        P_.block<3, 3>(10, 10) *= 0.5; // Angular velocity
        
        // Process noise covariance
        Q_ = Eigen::MatrixXd::Identity(13, 13);
        Q_.block<3, 3>(0, 0) *= process_noise_pos_; // Position
        Q_.block<4, 4>(3, 3) *= process_noise_rot_; // Orientation
        Q_.block<3, 3>(7, 7) *= process_noise_vel_; // Linear velocity
        Q_.block<3, 3>(10, 10) *= process_noise_accel_; // Angular velocity
        
        // Measurement noise covariance for odometry
        R_odom_ = Eigen::MatrixXd::Identity(7, 7);
        R_odom_.block<3, 3>(0, 0) *= odom_pos_noise_; // Position
        R_odom_.block<4, 4>(3, 3) *= odom_rot_noise_; // Orientation
        
        // Measurement noise covariance for IMU
        R_imu_ = Eigen::MatrixXd::Identity(6, 6);
        R_imu_.block<3, 3>(0, 0) *= imu_acc_noise_; // Linear acceleration
        R_imu_.block<3, 3>(3, 3) *= imu_gyro_noise_; // Angular velocity
        
        // Publishers
        fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("fused_odom", 10);
        
        // Subscribers
        rtabmap_odom_sub_ = nh_.subscribe("/rtabmap/odom", 10, &KalmanFilterFusion::odomCallback, this);
        imu_sub_ = nh_.subscribe("/camera/imu", 10, &KalmanFilterFusion::imuCallback, this);
        
        ROS_INFO("Sensor fusion node initialized");
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        if (!first_odom_received_) {
            // Initialize state with first odometry message
            x_(0) = odom_msg->pose.pose.position.x;
            x_(1) = odom_msg->pose.pose.position.y;
            x_(2) = odom_msg->pose.pose.position.z;
            x_(3) = odom_msg->pose.pose.orientation.w;
            x_(4) = odom_msg->pose.pose.orientation.x;
            x_(5) = odom_msg->pose.pose.orientation.y;
            x_(6) = odom_msg->pose.pose.orientation.z;
            x_(7) = odom_msg->twist.twist.linear.x;
            x_(8) = odom_msg->twist.twist.linear.y;
            x_(9) = odom_msg->twist.twist.linear.z;
            
            last_update_time_ = odom_msg->header.stamp;
            first_odom_received_ = true;
            ROS_INFO("First odometry message received");
            return;
        }
        
        // Time delta for prediction
        double dt = (odom_msg->header.stamp - last_update_time_).toSec();
        
        if (dt > 0) {
            // Predict step
            predictState(dt);
            
            // Update with odometry measurement
            updateWithOdom(odom_msg);
            
            // Publish fused state
            publishFusedState(odom_msg->header.stamp);
            
            last_update_time_ = odom_msg->header.stamp;
        }
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
    {
        if (!first_odom_received_) {
            // Need odometry first to initialize state
            return;
        }
        
        if (!first_imu_received_) {
            // Initialize angular velocity from first IMU message
            x_(10) = imu_msg->angular_velocity.x;
            x_(11) = imu_msg->angular_velocity.y;
            x_(12) = imu_msg->angular_velocity.z;
            
            first_imu_received_ = true;
            ROS_INFO("First IMU message received");
            return;
        }
        
        // Update with IMU measurement
        updateWithImu(imu_msg);
    }
    
    void predictState(double dt)
{
    // Simple constant velocity model for prediction
    // Position update using velocity
    x_(0) += x_(7) * dt;
    x_(1) += x_(8) * dt;
    x_(2) += x_(9) * dt;
    
    // Orientation update using angular velocity
    // Extract angular velocity from state
    double wx = x_(10);
    double wy = x_(11);
    double wz = x_(12);
    
    // Quaternion from state
    Eigen::Quaterniond q(x_(3), x_(4), x_(5), x_(6));
    
    // Create angular velocity quaternion
    Eigen::Quaterniond q_dot(0, wx, wy, wz);
    
    // Update quaternion: q' = q + 0.5 * dt * (q * q_dot)
    // First calculate q * q_dot
    Eigen::Quaterniond q_omega = q * q_dot;
    
    // Then scale and add
    Eigen::Quaterniond q_update;
    q_update.w() = q.w() + 0.5 * dt * q_omega.w();
    q_update.x() = q.x() + 0.5 * dt * q_omega.x();
    q_update.y() = q.y() + 0.5 * dt * q_omega.y();
    q_update.z() = q.z() + 0.5 * dt * q_omega.z();
    
    // Normalize the updated quaternion
    q_update.normalize();
    
    // Update state quaternion
    x_(3) = q_update.w();
    x_(4) = q_update.x();
    x_(5) = q_update.y();
    x_(6) = q_update.z();
    
    // Update covariance: P' = F*P*F' + Q
    // Approximate update for simplicity
    // In a full implementation, compute the Jacobian F
    P_ += Q_ * dt;
}
    
    void updateWithOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        // Measurement vector: [x, y, z, qw, qx, qy, qz]
        Eigen::VectorXd z = Eigen::VectorXd(7);
        z(0) = odom_msg->pose.pose.position.x;
        z(1) = odom_msg->pose.pose.position.y;
        z(2) = odom_msg->pose.pose.position.z;
        z(3) = odom_msg->pose.pose.orientation.w;
        z(4) = odom_msg->pose.pose.orientation.x;
        z(5) = odom_msg->pose.pose.orientation.y;
        z(6) = odom_msg->pose.pose.orientation.z;
        
        // Measurement model Jacobian - maps state to measurement
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, 13);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Position mapping
        H.block<4, 4>(3, 3) = Eigen::Matrix4d::Identity(); // Orientation mapping
        
        // Predicted measurement
        Eigen::VectorXd z_pred = Eigen::VectorXd(7);
        z_pred.head(7) = x_.head(7); // Extract position and orientation from state
        
        // Innovation (measurement residual)
        Eigen::VectorXd innovation = z - z_pred;
        
        // Special handling for quaternion difference
        // Ensure we take the shortest path in quaternion space
        Eigen::Quaterniond q_state(x_(3), x_(4), x_(5), x_(6));
        Eigen::Quaterniond q_meas(z(3), z(4), z(5), z(6));
        
        if (q_state.dot(q_meas) < 0) {
            // Flip quaternion if dot product is negative
            z(3) = -z(3);
            z(4) = -z(4);
            z(5) = -z(5);
            z(6) = -z(6);
            innovation.tail(4) = z.tail(4) - z_pred.tail(4);
        }
        
        // Innovation covariance
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;
        
        // Kalman gain
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // Update state
        x_ += K * innovation;
        
        // Normalize quaternion
        Eigen::Quaterniond q_updated(x_(3), x_(4), x_(5), x_(6));
        q_updated.normalize();
        x_(3) = q_updated.w();
        x_(4) = q_updated.x();
        x_(5) = q_updated.y();
        x_(6) = q_updated.z();
        
        // Update covariance
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(13, 13);
        P_ = (I - K * H) * P_;
        
        // Ensure symmetry
        P_ = (P_ + P_.transpose()) / 2.0;
    }
    
    void updateWithImu(const sensor_msgs::Imu::ConstPtr& imu_msg)
    {
        // Measurement vector: [ax, ay, az, wx, wy, wz]
        Eigen::VectorXd z = Eigen::VectorXd(6);
        z(0) = imu_msg->linear_acceleration.x;
        z(1) = imu_msg->linear_acceleration.y;
        z(2) = imu_msg->linear_acceleration.z - 9.81; // Remove gravity
        z(3) = imu_msg->angular_velocity.x;
        z(4) = imu_msg->angular_velocity.y;
        z(5) = imu_msg->angular_velocity.z;
        
        // For IMU, we only update angular velocity directly
        // Linear acceleration would require more complex dynamics model
        
        // Measurement model Jacobian for angular velocity
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 13);
        H.block<3, 3>(0, 10) = Eigen::Matrix3d::Identity(); // Angular velocity mapping
        
        // Predicted measurement (only angular velocity part)
        Eigen::VectorXd z_pred = Eigen::VectorXd(3);
        z_pred = x_.segment(10, 3); // Extract angular velocity from state
        
        // Innovation (measurement residual)
        Eigen::VectorXd innovation = z.tail(3) - z_pred;
        
        // Innovation covariance
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_imu_.block<3, 3>(3, 3);
        
        // Kalman gain
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // Update state
        x_ += K * innovation;
        
        // Update covariance
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(13, 13);
        P_ = (I - K * H) * P_;
        
        // Ensure symmetry
        P_ = (P_ + P_.transpose()) / 2.0;
    }
    
    void publishFusedState(const ros::Time& timestamp)
    {
        // Create and publish odometry message
        nav_msgs::Odometry fused_odom;
        fused_odom.header.stamp = timestamp;
        fused_odom.header.frame_id = odom_frame_id_;
        fused_odom.child_frame_id = base_frame_id_;
        
        // Fill in position and orientation
        fused_odom.pose.pose.position.x = x_(0);
        fused_odom.pose.pose.position.y = x_(1);
        fused_odom.pose.pose.position.z = x_(2);
        fused_odom.pose.pose.orientation.w = x_(3);
        fused_odom.pose.pose.orientation.x = x_(4);
        fused_odom.pose.pose.orientation.y = x_(5);
        fused_odom.pose.pose.orientation.z = x_(6);
        
        // Fill in velocity
        fused_odom.twist.twist.linear.x = x_(7);
        fused_odom.twist.twist.linear.y = x_(8);
        fused_odom.twist.twist.linear.z = x_(9);
        fused_odom.twist.twist.angular.x = x_(10);
        fused_odom.twist.twist.angular.y = x_(11);
        fused_odom.twist.twist.angular.z = x_(12);
        
        // Fill in covariance matrices
        // Position & orientation covariance (6x6 upper left corner of P_)
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                // Need to transform quaternion cov to RPY cov for ROS format
                // Approximated by using position cov and orientation cov as is
                int p_i = (i < 3) ? i : i + 1;  // Skip quaternion.w in P_
                int p_j = (j < 3) ? j : j + 1;  // Skip quaternion.w in P_
                fused_odom.pose.covariance[i * 6 + j] = P_(p_i, p_j);
            }
        }
        
        // Twist covariance
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                // Linear velocity
                fused_odom.twist.covariance[i * 6 + j] = P_(i + 7, j + 7);
                // Angular velocity
                fused_odom.twist.covariance[(i + 3) * 6 + (j + 3)] = P_(i + 10, j + 10);
            }
        }
        
        // Publish the odometry message
        fused_odom_pub_.publish(fused_odom);
        
        // Broadcast transform
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = timestamp;
        transform.header.frame_id = odom_frame_id_;
        transform.child_frame_id = base_frame_id_;
        
        transform.transform.translation.x = x_(0);
        transform.transform.translation.y = x_(1);
        transform.transform.translation.z = x_(2);
        transform.transform.rotation.w = x_(3);
        transform.transform.rotation.x = x_(4);
        transform.transform.rotation.y = x_(5);
        transform.transform.rotation.z = x_(6);
        
        tf_broadcaster_.sendTransform(transform);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_estimator");
    KalmanFilterFusion fusion;
    ros::spin();
    return 0;
}