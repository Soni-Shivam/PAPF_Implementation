#include "papf_planner/papf_node.hpp"
#include "rclcpp/qos.hpp"
#include <fstream>
#include <sstream>
#include <limits> 

using std::placeholders::_1;

PAPFNode::PAPFNode() : Node("papf_node")
{

    this->declare_parameter<std::string>("centerline_csv_path", "centerline.csv");
    this->get_parameter("centerline_csv_path", centerline_csv_path_);
    RCLCPP_INFO(this->get_logger(), "Expecting centerline file at: %s", centerline_csv_path_.c_str());

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    obstacle_circles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_circles", 10);

    centerline_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/centerline_path", rclcpp::QoS(1).transient_local());
    goal_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/dynamic_goal_marker", 10);

    start_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&PAPFNode::start_pose_callback, this, _1));

    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        rclcpp::SensorDataQoS(),
        std::bind(&PAPFNode::lidar_callback, this, _1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    load_centerline_from_csv();
    publish_centerline();

    RCLCPP_INFO(this->get_logger(), "PAPF Node has been started.");
}

void PAPFNode::start_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received start pose from RViz. Planning is now ENABLED.");
    planning_enabled_ = true;
    (void)msg; 
}

void PAPFNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

    update_obstacles_from_scan(msg); 
    RCLCPP_INFO_ONCE(this->get_logger(), "--- LIDAR CALLBACK IS NOW RUNNING! ---");
    publish_obstacle_circles();

    if (!planning_enabled_) {
        return;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    std::string source_frame = msg->header.frame_id; 
    try {

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
        return; 
    }

    update_dynamic_goal(msg, current_pose); 

    publish_goal_marker();

    try_to_plan_path(current_pose);
}

void PAPFNode::update_dynamic_goal(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
    const geometry_msgs::msg::PoseStamped& current_pose)
{
    if (centerline_points_.empty())
    {
        RCLCPP_WARN_ONCE(this->get_logger(), "Centerline is empty, cannot update dynamic goal.");
        return;
    }

    const double LOOKAHEAD_DISTANCE = 4.0; 

    Vector2D robot_pos(current_pose.pose.position.x, current_pose.pose.position.y);

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
        return; 
    }

    int goal_index = nearest_index;
    for (size_t i = 0; i < centerline_points_.size(); ++i)
    {

        int test_index = (nearest_index + i) % centerline_points_.size();

        double dist_from_robot = centerline_points_[test_index].distanceTo(robot_pos);

        if (dist_from_robot > LOOKAHEAD_DISTANCE) {
            goal_index = test_index;
            break; 
        }

        goal_index = test_index;
    }

    const Vector2D& dynamic_goal = centerline_points_[goal_index];

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = global_frame_;
    goal_msg.header.stamp = this->get_clock()->now();
    goal_msg.pose.position.x = dynamic_goal.x;
    goal_msg.pose.position.y = dynamic_goal.y;
    goal_msg.pose.orientation.w = 1.0; 

    goal_pose_ = goal_msg; 

    (void)scan_msg;
}

void PAPFNode::update_obstacles_from_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{

    const double VOXEL_SIZE = 0.1; 
    const double OBSTACLE_POINT_RADIUS = 0.05; 

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

    const int VIRTUAL_WALL_POINTS = 10;
    const double WALL_DISTANCE = 0.5; 
    const double arc_start_rad = 135.0 * M_PI / 180.0; 
    const double arc_end_rad = 225.0 * M_PI / 180.0;   
    const double arc_increment = (arc_end_rad - arc_start_rad) / (VIRTUAL_WALL_POINTS - 1);

    for (int i = 0; i < VIRTUAL_WALL_POINTS; ++i) {
        double angle = arc_start_rad + i * arc_increment;
        raw_points_local.emplace_back(WALL_DISTANCE * cos(angle), WALL_DISTANCE * sin(angle));
    }

    if (raw_points_local.empty()) {
        obstacle_circles_.clear(); 
        return;
    }

    std::vector<Vector2D> raw_points_map;
    for (const auto& local_point : raw_points_local) {
        geometry_msgs::msg::PointStamped point_in;
        point_in.header = scan_msg->header;
        point_in.point.x = local_point.x;
        point_in.point.y = local_point.y;
        point_in.point.z = 0.0; 

        geometry_msgs::msg::PointStamped point_out;
        tf2::doTransform(point_in, point_out, tf_stamped);

        raw_points_map.emplace_back(point_out.point.x, point_out.point.y);
    }

    std::vector<Vector2D> downsampled_points = voxel_grid_downsample(raw_points_map, VOXEL_SIZE);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "Downsampled %zu raw points to %zu obstacles.", 
        raw_points_map.size(), downsampled_points.size());

    obstacle_circles_.clear(); 
    for (const auto& point : downsampled_points) {
        obstacle_circles_.emplace_back(Obstacle{point, OBSTACLE_POINT_RADIUS});
    }
}

std::vector<Vector2D> PAPFNode::voxel_grid_downsample(const std::vector<Vector2D>& points, double voxel_size)
{

    std::unordered_map<std::pair<int, int>, std::pair<Vector2D, int>, VoxelHasher> voxel_map;

    for (const auto& point : points) {

        int voxel_x = static_cast<int>(floor(point.x / voxel_size));
        int voxel_y = static_cast<int>(floor(point.y / voxel_size));
        std::pair<int, int> voxel_index = {voxel_x, voxel_y};

        voxel_map[voxel_index].first = voxel_map[voxel_index].first + point;
        voxel_map[voxel_index].second++;
    }

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
        marker.header.frame_id = global_frame_; 
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

void PAPFNode::try_to_plan_path(const geometry_msgs::msg::PoseStamped& current_pose)
{

    if (!goal_pose_) {

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "No goal set yet, skipping plan.");
        return;
    }

    RCLCPP_INFO_ONCE(this->get_logger(), "All inputs received, planning path...");

    plan_path(current_pose); 
}

void PAPFNode::plan_path(const geometry_msgs::msg::PoseStamped& current_start_pose)
{
    PAPF_Planner::Params params;

    PAPF_Planner planner(params);

    USV usv;
    usv.position.x = current_start_pose.pose.position.x;
    usv.position.y = current_start_pose.pose.position.y;

    usv.yaw = tf2::getYaw(current_start_pose.pose.orientation);
    usv.velocity = 0.0; 

    Vector2D goal(goal_pose_->pose.position.x, goal_pose_->pose.position.y);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = global_frame_;

    const int MAX_STEPS = 500;
    const double GOAL_TOLERANCE = 0.5;
    for (int i = 0; i < MAX_STEPS; ++i) {
        geometry_msgs::msg::PoseStamped pose_in_path;
        pose_in_path.header = path_msg.header;
        pose_in_path.pose.position.x = usv.position.x;
        pose_in_path.pose.position.y = usv.position.y;

        path_msg.poses.push_back(pose_in_path);

        if (usv.position.distanceTo(goal) < GOAL_TOLERANCE) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached in plan!");
            break;
        }

        planner.computeStep(usv, goal, obstacle_circles_);
    }

    path_publisher_->publish(path_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Published planned path with %zu poses.", path_msg.poses.size());

}

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

    std::getline(file, line); 
    RCLCPP_INFO(this->get_logger(), "Skipped header line: %s", line.c_str());

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str;
        double x, y;

        if (!std::getline(ss, x_str, ',')) {

            continue; 
        }

        if (!std::getline(ss, y_str, ',')) {

            continue; 
        }

        try {
            x = std::stod(x_str);
            y = std::stod(y_str);
        } catch (const std::invalid_argument& e) {
            RCLCPP_WARN(this->get_logger(), "Skipping non-numeric line: %s. Error: %s", line.c_str(), e.what());
            continue; 
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

void PAPFNode::publish_goal_marker()
{

    if (!goal_pose_) {
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "dynamic_goal";
    marker.id = 0; 
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = goal_pose_->pose;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; 

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    goal_marker_publisher_->publish(marker);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PAPFNode>());
    rclcpp::shutdown();
    return 0;
}