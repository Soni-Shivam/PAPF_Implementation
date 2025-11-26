#ifndef PAPF_NODE_HPP_
#define PAPF_NODE_HPP_

#include <vector>
#include <cmath>
#include <optional>
#include <unordered_map> // For Voxel Grid
#include <fstream>       // For file loading
#include <sstream>       // For string parsing
#include <limits>        // For std::numeric_limits

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "papf_planner/PAPF_Planner.h" 

// --- Includes for TF2 Transformations ---
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h" // For tf2::getYaw
// ----------------------------------------

// ... (VoxelHasher struct is unchanged) ...
struct VoxelHasher {
    std::size_t operator()(const std::pair<int, int>& k) const {
        return std::hash<int>()(k.first) ^ (std::hash<int>()(k.second) << 1);
    }
};


class PAPFNode : public rclcpp::Node
{
public:
    PAPFNode();

private:
    // Callbacks
    void start_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    // Helper functions
    void update_obstacles_from_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);    std::vector<Vector2D> voxel_grid_downsample(const std::vector<Vector2D>& points, double voxel_size);
    
    void publish_obstacle_circles();
    void plan_path(const geometry_msgs::msg::PoseStamped& current_start_pose);
    void publish_potential_field(const geometry_msgs::msg::PoseStamped& current_pose);
    
    

    // --- New functions for dynamic goal planning ---
    void load_centerline_from_csv();
    void publish_centerline();
    void publish_goal_marker();

    /**
     * @brief MODIFIED: Calculates and updates the dynamic goal pose.
     * * Now uses a "lookahead" logic based on the robot's current pose.
     * * @param scan_msg The incoming laser scan message (for future use).
     * * @param current_pose The robot's current pose in the global_frame.
     */
    void update_dynamic_goal(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
        const geometry_msgs::msg::PoseStamped& current_pose
    );

    /**
     * @brief MODIFIED: Now just checks for a goal and calls plan_path.
     * * The current_pose is now passed in from lidar_callback.
     * * @param current_pose The robot's current pose in the global_frame.
     */
    void try_to_plan_path(const geometry_msgs::msg::PoseStamped& current_pose);
    // ------------------------------------------------

    // ROS Publishers and Subscribers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_circles_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr potential_field_publisher_;
    std::string global_frame_ = "map";           // Planning frame

    // --- TF2 Members ---
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // -------------------

    // State variables
    std::optional<geometry_msgs::msg::PoseStamped> goal_pose_;
    std::vector<Obstacle> obstacle_circles_;

    // --- New state variables ---
    std::vector<Vector2D> centerline_points_; // Stores the loaded centerline
    bool planning_enabled_ = false;           // Flag to start planning, set by /initialpose
    std::string centerline_csv_path_;         // Path to the CSV file
    // ---------------------------
};

#endif // PAPF_NODE_HPP_