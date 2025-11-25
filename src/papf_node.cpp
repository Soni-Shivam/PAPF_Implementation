#include "papf_planner/papf_node.hpp"
#include "rclcpp/qos.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <sstream>
#include <limits> 

using std::placeholders::_1;

PAPFNode::PAPFNode() : Node("papf_node")
{
    // --- Parameters ---
    // this->declare_parameter<std::string>("centerline_csv_path", "centerline.csv");
    // this->get_parameter("centerline_csv_path", centerline_csv_path_);
    // RCLCPP_INFO(this->get_logger(), "Expecting centerline file at: %s", centerline_csv_path_.c_str());
    // // ------------------

    // --- NEW, ROBUST CODE ---
    std::string package_share_path;
    try {
        // Find the absolute path to your package's share directory
        package_share_path = ament_index_cpp::get_package_share_directory("papf_planner");
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Could not find package 'papf_planner': %s", e.what());
        // Handle the error appropriately
    }

    // You can still use a parameter for the *filename* itself, which is good
    this->declare_parameter<std::string>("centerline_filename", "centerline.csv");
    std::string csv_filename;
    this->get_parameter("centerline_filename", csv_filename);

    // Construct the full, correct path
    centerline_csv_path_ = package_share_path + "/racelines/" + csv_filename;
    
    RCLCPP_INFO(this->get_logger(), "Loading centerline from: %s", centerline_csv_path_.c_str());
    // ------------------


    // --- NEW: Declare all tunable parameters ---

    // Node-level (perception, goal, etc.)
    this->declare_parameter<double>("lookahead_distance", 4.0);
    this->declare_parameter<double>("voxel_size", 0.1);
    this->declare_parameter<double>("obstacle_point_radius", 0.05);
    this->declare_parameter<int>("virtual_wall_points", 10);
    this->declare_parameter<double>("virtual_wall_distance", 0.5);

    // Planner-level (from PAPF_Planner.h)
    this->declare_parameter<double>("papf.k_att", 100.0);
    this->declare_parameter<double>("papf.d_g", 4.0);
    this->declare_parameter<double>("papf.k_rep", 0.5);
    this->declare_parameter<double>("papf.d_o", 5.0);
    this->declare_parameter<int>("papf.n", 2);
    this->declare_parameter<double>("papf.k_prd", 5.0);
    this->declare_parameter<double>("papf.d_prd", 20.0);
    this->declare_parameter<double>("papf.max_turning_angle_deg", 20.0);
    this->declare_parameter<double>("papf.v_c", 1.0);
    this->declare_parameter<double>("papf.v_min", 0.5);
    this->declare_parameter<double>("papf.v_max", 2.0);
    this->declare_parameter<double>("papf.max_acceleration", 1.0);
    this->declare_parameter<double>("papf.max_deceleration", 1.0);
    this->declare_parameter<double>("papf.dt", 0.1);

    // Planning loop
    this->declare_parameter<int>("max_plan_steps", 500);
    this->declare_parameter<double>("goal_tolerance", 0.5);


    // Publishers
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    obstacle_circles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_circles", 10);
    
    centerline_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/centerline_path", rclcpp::QoS(1).transient_local());
    goal_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/dynamic_goal_marker", 10);
    // Subscribers
    start_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&PAPFNode::start_pose_callback, this, _1));
    
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        rclcpp::SensorDataQoS(),
        std::bind(&PAPFNode::lidar_callback, this, _1));

    // --- Initialize TF2 ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // ----------------------

    // --- Load and publish centerline ---
    load_centerline_from_csv();
    publish_centerline();
    // ---------------------------------

    RCLCPP_INFO(this->get_logger(), "PAPF Node has been started.");
}

void PAPFNode::start_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received start pose from RViz. Planning is now ENABLED.");
    planning_enabled_ = true;
    (void)msg; // Suppress "unused parameter" warning
}

// --- ADDED CONST ---
void PAPFNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 1. Update obstacles
    update_obstacles_from_scan(msg); 
    RCLCPP_INFO_ONCE(this->get_logger(), "--- LIDAR CALLBACK IS NOW RUNNING! ---");
    publish_obstacle_circles();
    
    // 2. Check if planning is even enabled. If not, don't do expensive calcs.
    if (!planning_enabled_) {
        return;
    }

    // 3. Get the robot's current pose from TF *first*
    geometry_msgs::msg::PoseStamped current_pose;
    std::string source_frame = msg->header.frame_id; // e.g., "ego_racecar/laser_model"
    try {
        // Get the transform from "map" to our "laser_frame"
        geometry_msgs::msg::TransformStamped t_map_to_source = tf_buffer_->lookupTransform(
            global_frame_, source_frame, tf2::TimePointZero);
        
        current_pose.header.frame_id = global_frame_;
        current_pose.header.stamp = t_map_to_source.header.stamp;
        current_pose.pose.position.x = t_map_to_source.transform.translation.x;
        current_pose.pose.position.y = t_map_to_source.transform.translation.y;
        current_pose.pose.orientation = t_map_to_source.transform.rotation;

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose (%s -> %s): %s",
            global_frame_.c_str(), source_frame.c_str(), ex.what());
        return; // Can't plan without a start pose
    }
    
    // 4. Update the dynamic goal using the new "lookahead" logic
    update_dynamic_goal(msg, current_pose); 

    publish_goal_marker();
    // 5. Try to plan a path from our current pose
    try_to_plan_path(current_pose);
}


/**
 * @brief NEW "Lookahead" logic for dynamic goal.
 * Finds the nearest point on the centerline to the robot,
 * then finds a new goal point a "lookahead_distance" *ahead*
 * of that point.
 */
// --- ADDED CONST ---
void PAPFNode::update_dynamic_goal(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
    const geometry_msgs::msg::PoseStamped& current_pose)
{
    if (centerline_points_.empty())
    {
        RCLCPP_WARN_ONCE(this->get_logger(), "Centerline is empty, cannot update dynamic goal.");
        return;
    }
    
    // --- Tunable Parameter ---
    // This is the "carrot" distance. 3-5m is usually good for F1TENTH.
    // const double LOOKAHEAD_DISTANCE = 4.0; // 4 meters
    const double LOOKAHEAD_DISTANCE = this->get_parameter("lookahead_distance").as_double(); // <-- ADD THIS
    // -------------------------

    Vector2D robot_pos(current_pose.pose.position.x, current_pose.pose.position.y);

    // --- Step 1: Find the nearest centerline point to the ROBOT ---
    double min_dist_sq = std::numeric_limits<double>::max();
    int nearest_index = -1;

    for (size_t i = 0; i < centerline_points_.size(); ++i) {
        double dist_sq = centerline_points_[i].distanceToSquared(robot_pos);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_index = i;
        }
    }
    
    if (nearest_index == -1) {
        return; // No points
    }

    // --- Step 2: Find the goal point "LOOKAHEAD_DISTANCE" away ---
    // We iterate forward from our nearest point until we find a
    // point that is *just past* the lookahead distance.
    int goal_index = nearest_index;
    for (size_t i = 0; i < centerline_points_.size(); ++i)
    {
        // Modulo arithmetic to wrap around the track
        int test_index = (nearest_index + i) % centerline_points_.size();
        
        double dist_from_robot = centerline_points_[test_index].distanceTo(robot_pos);
        
        if (dist_from_robot > LOOKAHEAD_DISTANCE) {
            goal_index = test_index;
            break; // Found our goal
        }
        
        // If we loop all the way around and never find a point
        // far enough away (e.g., small track), just take the
        // furthest one we found.
        goal_index = test_index;
    }


    // --- Step 3: Set this point as the new goal_pose_ ---
    const Vector2D& dynamic_goal = centerline_points_[goal_index];
    
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = global_frame_;
    goal_msg.header.stamp = this->get_clock()->now();
    goal_msg.pose.position.x = dynamic_goal.x;
    goal_msg.pose.position.y = dynamic_goal.y;
    goal_msg.pose.orientation.w = 1.0; 

    goal_pose_ = goal_msg; // Set the member variable
    
    // We no longer use the LaserScan message, but we keep it
    // in case we want to add obstacle-based logic later.
    (void)scan_msg;
}


// --- ADDED CONST ---
void PAPFNode::update_obstacles_from_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    // --- Tunable Parameters ---
    // const double VOXEL_SIZE = 0.1; // 10cm grid cells
    // const double OBSTACLE_POINT_RADIUS = 0.05; // 5cm safety radius
    const double VOXEL_SIZE = this->get_parameter("voxel_size").as_double(); // <-- ADD THIS
    const double OBSTACLE_POINT_RADIUS = this->get_parameter("obstacle_point_radius").as_double(); // <-- ADD THIS
// ...
    
    // --- Get Transform ---
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(
            global_frame_, scan_msg->header.frame_id,
            scan_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
            scan_msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
        return;
    }
    // ---------------------

    // --- Step 1: Convert LaserScan to raw Vector2D points (in laser_frame) ---
    std::vector<Vector2D> raw_points_local;
    const size_t num_measurements = scan_msg->ranges.size();
    const double angle_min = scan_msg->angle_min;
    const double angle_increment = scan_msg->angle_increment;
    const double range_min_valid = scan_msg->range_min;
    const double range_max_valid = scan_msg->range_max;

    for (size_t i = 0; i < num_measurements; ++i) 
    {
        const float r = scan_msg->ranges[i];
        if ((r >= range_min_valid) && (r <= range_max_valid) && std::isfinite(r)) {
            double angle = angle_min + i * angle_increment;
            raw_points_local.emplace_back(r * cos(angle), r * sin(angle));
        }
    }
    
    // --- Step 1b: Add a "Virtual Wall" behind the robot ---
    // const int VIRTUAL_WALL_POINTS = 10;
    // const double WALL_DISTANCE = 0.5; // 0.5 meters behind the robot (laser_frame)

    const int VIRTUAL_WALL_POINTS = this->get_parameter("virtual_wall_points").as_int(); // <-- ADD THIS
    const double WALL_DISTANCE = this->get_parameter("virtual_wall_distance").as_double(); // <-- ADD THIS

    const double arc_start_rad = 135.0 * M_PI / 180.0; // 2.356 rad
    const double arc_end_rad = 225.0 * M_PI / 180.0;   // 3.927 rad
    const double arc_increment = (arc_end_rad - arc_start_rad) / (VIRTUAL_WALL_POINTS - 1);

    for (int i = 0; i < VIRTUAL_WALL_POINTS; ++i) {
        double angle = arc_start_rad + i * arc_increment;
        raw_points_local.emplace_back(WALL_DISTANCE * cos(angle), WALL_DISTANCE * sin(angle));
    }
    // --- End of Virtual Wall ---

    if (raw_points_local.empty()) {
        obstacle_circles_.clear(); // Clear old obstacles
        return;
    }

    // --- Step 2: Transform raw points (real + virtual) to "map" frame ---
    std::vector<Vector2D> raw_points_map;
    for (const auto& local_point : raw_points_local) {
        geometry_msgs::msg::PointStamped point_in;
        point_in.header = scan_msg->header;
        point_in.point.x = local_point.x;
        point_in.point.y = local_point.y;
        point_in.point.z = 0.0; // Assume 2D

        geometry_msgs::msg::PointStamped point_out;
        tf2::doTransform(point_in, point_out, tf_stamped);
        
        raw_points_map.emplace_back(point_out.point.x, point_out.point.y);
    }

    // --- Step 3: "Merge dense points" in the map frame ---
    std::vector<Vector2D> downsampled_points = voxel_grid_downsample(raw_points_map, VOXEL_SIZE);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, // Log every 5s
        "Downsampled %zu raw points to %zu obstacles.", 
        raw_points_map.size(), downsampled_points.size());

    // --- Step 4: Create final obstacle list ---
    obstacle_circles_.clear(); 
    for (const auto& point : downsampled_points) {
        obstacle_circles_.emplace_back(Obstacle{point, OBSTACLE_POINT_RADIUS});
    }
}


std::vector<Vector2D> PAPFNode::voxel_grid_downsample(const std::vector<Vector2D>& points, double voxel_size)
{
    // <VoxelIndex, <SumOfPoints, PointCount>>
    std::unordered_map<std::pair<int, int>, std::pair<Vector2D, int>, VoxelHasher> voxel_map;

    for (const auto& point : points) {
        // Calculate which voxel (grid cell) this point belongs to
        int voxel_x = static_cast<int>(floor(point.x / voxel_size));
        int voxel_y = static_cast<int>(floor(point.y / voxel_size));
        std::pair<int, int> voxel_index = {voxel_x, voxel_y};

        // Add the point to the sum for that voxel
        voxel_map[voxel_index].first = voxel_map[voxel_index].first + point;
        voxel_map[voxel_index].second++;
    }

    // Now, average all the points in each voxel to get the final list
    std::vector<Vector2D> downsampled_points;
    for (auto const& [index, pair] : voxel_map) {
        const Vector2D& point_sum = pair.first;
        const int count = pair.second;
        downsampled_points.push_back(point_sum * (1.0 / count));
    }

    return downsampled_points;
}

void PAPFNode::publish_obstacle_circles()
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto& obs : obstacle_circles_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_; // <-- Use global_frame_
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "obstacle_circles";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = obs.center.x;
        marker.pose.position.y = obs.center.y;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2 * obs.radius;
        marker.scale.y = 2 * obs.radius;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.a = 0.5;
        marker_array.markers.push_back(marker);
    }
    obstacle_circles_publisher_->publish(marker_array);
}


/**
 * @brief MODIFIED: Now much simpler.
 * The pose is already calculated and passed in from lidar_callback.
 * We just check if we have a goal and then call plan_path.
 */
void PAPFNode::try_to_plan_path(const geometry_msgs::msg::PoseStamped& current_pose)
{
    // planning_enabled_ is already checked in lidar_callback.
    // The pose is already validated in lidar_callback.
    
    if (!goal_pose_) {
        // This is normal if the centerline hasn't loaded or
        // update_dynamic_goal hasn't run yet.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "No goal set yet, skipping plan.");
        return;
    }
    
    // We only print this message if all checks and transforms have passed.
    RCLCPP_INFO_ONCE(this->get_logger(), "All inputs received, planning path...");

    plan_path(current_pose); // Pass the pose we already found
}

void PAPFNode::plan_path(const geometry_msgs::msg::PoseStamped& current_start_pose)
{
    // PAPF_Planner::Params params;
    // You can now set params as needed, e.g., params.dt = 0.1;

    // --- NEW: Load all planner params from the ROS 2 server ---
    PAPF_Planner::Params params;
    params.k_att = this->get_parameter("papf.k_att").as_double();
    params.d_g = this->get_parameter("papf.d_g").as_double();
    params.k_rep = this->get_parameter("papf.k_rep").as_double();
    params.d_o = this->get_parameter("papf.d_o").as_double();
    params.n = this->get_parameter("papf.n").as_int();
    params.k_prd = this->get_parameter("papf.k_prd").as_double();
    params.d_prd = this->get_parameter("papf.d_prd").as_double();
    params.max_turning_angle_deg = this->get_parameter("papf.max_turning_angle_deg").as_double();
    params.v_c = this->get_parameter("papf.v_c").as_double();
    params.v_min = this->get_parameter("papf.v_min").as_double();
    params.v_max = this->get_parameter("papf.v_max").as_double();
    params.max_acceleration = this->get_parameter("papf.max_acceleration").as_double();
    params.max_deceleration = this->get_parameter("papf.max_deceleration").as_double();
    params.dt = this->get_parameter("papf.dt").as_double();
    // --- End of new param loading ---
    
    PAPF_Planner planner(params);

    // --- Initialize USV state from the current robot pose ---
    USV usv;
    usv.position.x = current_start_pose.pose.position.x;
    usv.position.y = current_start_pose.pose.position.y;
    // Extract yaw from the quaternion
    usv.yaw = tf2::getYaw(current_start_pose.pose.orientation);
    usv.velocity = 0.0; // Assuming start velocity is 0 for planning
    // --------------------------------------------------------

    Vector2D goal(goal_pose_->pose.position.x, goal_pose_->pose.position.y);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = global_frame_;

    // const int MAX_STEPS = 500;
    // const double GOAL_TOLERANCE = 0.5;
    const int MAX_STEPS = this->get_parameter("max_plan_steps").as_int();             // <-- ADD THIS
    const double GOAL_TOLERANCE = this->get_parameter("goal_tolerance").as_double();  // <-- ADD THIS


    for (int i = 0; i < MAX_STEPS; ++i) {
        geometry_msgs::msg::PoseStamped pose_in_path;
        pose_in_path.header = path_msg.header;
        pose_in_path.pose.position.x = usv.position.x;
        pose_in_path.pose.position.y = usv.position.y;
        // You could also set orientation here if needed
        path_msg.poses.push_back(pose_in_path);

        if (usv.position.distanceTo(goal) < GOAL_TOLERANCE) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached in plan!");
            break;
        }

        // This is now safe: usv, goal, and obstacles are all in the "map" frame
        planner.computeStep(usv, goal, obstacle_circles_);
    }

    path_publisher_->publish(path_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, // Log every 1s
        "Published planned path with %zu poses.", path_msg.poses.size());

    // --- DO NOT RESET POSES ---
    // We want to keep planning on the next cycle
}


// --- NEW Function Implementations ---

void PAPFNode::load_centerline_from_csv()
{
    std::ifstream file(centerline_csv_path_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open centerline file: %s", 
            centerline_csv_path_.c_str());
        return;
    }

    centerline_points_.clear();
    std::string line;
    
    // --- FIX 1: Skip the header line ---
    std::getline(file, line); 
    RCLCPP_INFO(this->get_logger(), "Skipped header line: %s", line.c_str());

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str;
        double x, y;

        // --- FIX 2: More robust parsing ---
        
        // Get the first value (x)
        if (!std::getline(ss, x_str, ',')) {
            //RCLCPP_WARN(this->get_logger(), "Skipping malformed line (no x): %s", line.c_str());
            continue; // Skip empty lines or lines without a comma
        }
        
        // Get the second value (y)
        if (!std::getline(ss, y_str, ',')) {
            //RCLCPP_WARN(this->get_logger(), "Skipping malformed line (no y): %s", line.c_str());
            continue; // Skip lines without a second value
        }
        
        // Try to convert the strings to doubles
        try {
            x = std::stod(x_str);
            y = std::stod(y_str);
        } catch (const std::invalid_argument& e) {
            RCLCPP_WARN(this->get_logger(), "Skipping non-numeric line: %s. Error: %s", line.c_str(), e.what());
            continue; // Skip lines where x or y are not valid numbers
        }
        
        centerline_points_.emplace_back(x, y);
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu points from centerline file.", 
        centerline_points_.size());
    file.close();
}

void PAPFNode::publish_centerline()
{
    if (centerline_points_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot publish centerline, no points loaded.");
        return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = global_frame_;

    for (const auto& point : centerline_points_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    centerline_publisher_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Published centerline path to /centerline_path");
}

// --- NEW: Implementation of the goal marker publisher ---
void PAPFNode::publish_goal_marker()
{
    // Only publish if we have a valid goal
    if (!goal_pose_) {
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "dynamic_goal";
    marker.id = 0; // We only have one goal marker
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose from our calculated goal_pose_
    marker.pose = goal_pose_->pose;

    // Set the scale (0.5m diameter sphere)
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color (bright green)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // Fully opaque

    // Set a short lifetime so it disappears if the node stops
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    goal_marker_publisher_->publish(marker);
}

// --- main function is unchanged ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PAPFNode>());
    rclcpp::shutdown();
    return 0;
}