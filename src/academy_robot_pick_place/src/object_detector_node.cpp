#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "academy_robot_interfaces/srv/detect_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/PolygonMesh.h>

struct ColoredPoint
{
  float x;
  float y;
  float z;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct FieldLayout
{
  int x_offset{-1};
  int y_offset{-1};
  int z_offset{-1};
  int color_offset{-1};

  bool valid() const
  {
    return x_offset >= 0 && y_offset >= 0 && z_offset >= 0 && color_offset >= 0;
  }
};

class ObjectDetectorNode : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;
  using DetectObject = academy_robot_interfaces::srv::DetectObject;

  ObjectDetectorNode()
  : Node("object_detector_node"),
    frame_count_(0),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    latest_pose_valid_(false)
  {
    min_depth_m_ = this->declare_parameter<double>("min_depth_m", 0.0);
    max_depth_m_ = this->declare_parameter<double>("max_depth_m", 10.0);

    roi_u_min_ratio_ = this->declare_parameter<double>("roi_u_min_ratio", 0.20);
    roi_u_max_ratio_ = this->declare_parameter<double>("roi_u_max_ratio", 0.80);
    roi_v_min_ratio_ = this->declare_parameter<double>("roi_v_min_ratio", 0.35);
    roi_v_max_ratio_ = this->declare_parameter<double>("roi_v_max_ratio", 1.00);

    seed_depth_margin_m_ = this->declare_parameter<double>("seed_depth_margin_m", 0.06);
    cluster_radius_m_ = this->declare_parameter<double>("cluster_radius_m", 0.10);
    region_radius_px_ = this->declare_parameter<int>("region_radius_px", 140);
    min_points_for_object_ = this->declare_parameter<int>("min_points_for_object", 20);

    reference_stl_path_ = this->declare_parameter<std::string>(
      "reference_stl_path",
      "/home/user/ros2_ws/src/academy_robot_simulation/academy_robot_gazebo_ignition/meshes/socket_cap_screw.stl");

    target_frame_ = this->declare_parameter<std::string>("target_frame", "robot_odom");
    reference_scale_ = this->declare_parameter<double>("reference_scale", 0.001);
    voxel_leaf_size_m_ = this->declare_parameter<double>("voxel_leaf_size_m", 0.0015);
    icp_max_corr_m_ = this->declare_parameter<double>("icp_max_corr_m", 0.03);
    icp_max_iterations_ = this->declare_parameter<int>("icp_max_iterations", 80);
    icp_fitness_threshold_ = this->declare_parameter<double>("icp_fitness_threshold", 0.02);
    tf_lookup_timeout_s_ = this->declare_parameter<double>("tf_lookup_timeout_s", 1.0);
    pose_publish_period_ms_ = this->declare_parameter<int>("pose_publish_period_ms", 500);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/front_rgbd_camera/depth/color/points",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&ObjectDetectorNode::pointCloudCallback, this, std::placeholders::_1));

    isolated_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/isolated_object_cloud",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    detected_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/detected_object_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    detect_object_srv_ = this->create_service<DetectObject>(
      "/detect_object",
      std::bind(
        &ObjectDetectorNode::handleDetectObject,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    pose_republish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(pose_publish_period_ms_),
      std::bind(&ObjectDetectorNode::republishLatestPose, this));

    reference_cloud_model_ = std::make_shared<CloudT>();
    reference_cloud_down_ = std::make_shared<CloudT>();

    reference_loaded_ = loadReferenceCloud();

    RCLCPP_INFO(
      this->get_logger(),
      "T2/T3/T4/T11 detector started | depth_range=[%.2f, %.2f] m | roi_u=[%.2f, %.2f] | roi_v=[%.2f, %.2f] | seed_depth_margin=%.3f m | cluster_radius=%.3f m | region_radius_px=%d",
      min_depth_m_, max_depth_m_,
      roi_u_min_ratio_, roi_u_max_ratio_,
      roi_v_min_ratio_, roi_v_max_ratio_,
      seed_depth_margin_m_, cluster_radius_m_, region_radius_px_);

    RCLCPP_INFO(
      this->get_logger(),
      "T3 params | reference_stl=%s | target_frame=%s | reference_scale=%.6f | voxel_leaf=%.4f m | icp_max_corr=%.3f m | icp_max_iter=%d | icp_fitness_thresh=%.5f | tf_timeout=%.2f s",
      reference_stl_path_.c_str(),
      target_frame_.c_str(),
      reference_scale_,
      voxel_leaf_size_m_,
      icp_max_corr_m_,
      icp_max_iterations_,
      icp_fitness_threshold_,
      tf_lookup_timeout_s_);

    RCLCPP_INFO(
      this->get_logger(),
      "T4/T11 interfaces ready | topic=/detected_object_pose | service=/detect_object");
  }

private:
  static FieldLayout getFieldLayout(const sensor_msgs::msg::PointCloud2 & msg)
  {
    FieldLayout layout;

    for (const auto & field : msg.fields) {
      if (field.name == "x") {
        layout.x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y") {
        layout.y_offset = static_cast<int>(field.offset);
      } else if (field.name == "z") {
        layout.z_offset = static_cast<int>(field.offset);
      } else if (field.name == "rgb" || field.name == "rgba") {
        layout.color_offset = static_cast<int>(field.offset);
      }
    }

    return layout;
  }

  static float readFloat32(const uint8_t * point_ptr, int offset)
  {
    float value = 0.0f;
    std::memcpy(&value, point_ptr + offset, sizeof(float));
    return value;
  }

  static void readPackedRgb(
    const uint8_t * point_ptr,
    int color_offset,
    uint8_t & r,
    uint8_t & g,
    uint8_t & b)
  {
    std::uint32_t packed = 0;
    std::memcpy(&packed, point_ptr + color_offset, sizeof(std::uint32_t));

    r = static_cast<uint8_t>((packed >> 16) & 0xFF);
    g = static_cast<uint8_t>((packed >> 8) & 0xFF);
    b = static_cast<uint8_t>(packed & 0xFF);
  }

  void handleDetectObject(
    const std::shared_ptr<DetectObject::Request> request,
    std::shared_ptr<DetectObject::Response> response)
  {
    (void)request;

    if (latest_pose_valid_) {
      response->success = true;
      response->detected_pose = latest_pose_global_;

      RCLCPP_INFO(
        this->get_logger(),
        "T11 service reply | success=true | frame=%s | xyz=(%.4f, %.4f, %.4f)",
        response->detected_pose.header.frame_id.c_str(),
        response->detected_pose.pose.position.x,
        response->detected_pose.pose.position.y,
        response->detected_pose.pose.position.z);
    } else {
      response->success = false;
      response->detected_pose = geometry_msgs::msg::PoseStamped();
      response->detected_pose.header.frame_id = target_frame_;

      RCLCPP_WARN(
        this->get_logger(),
        "T11 service reply | success=false | no valid pose cached yet");
    }
  }

  void publishFilteredCloud(
    const std_msgs::msg::Header & header,
    const std::vector<ColoredPoint> & points)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header = header;
    output.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(output, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(output, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(output, "b");

    for (const auto & p : points) {
      *iter_x = p.x;
      *iter_y = p.y;
      *iter_z = p.z;
      *iter_r = p.r;
      *iter_g = p.g;
      *iter_b = p.b;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }

    isolated_cloud_pub_->publish(output);
  }

  CloudT::Ptr downsampleCloud(const CloudT::Ptr & input) const
  {
    auto output = std::make_shared<CloudT>();
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(
      static_cast<float>(voxel_leaf_size_m_),
      static_cast<float>(voxel_leaf_size_m_),
      static_cast<float>(voxel_leaf_size_m_));
    voxel.filter(*output);
    return output;
  }

  bool loadReferenceCloud()
  {
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(reference_stl_path_, mesh) == 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to load reference STL: %s",
        reference_stl_path_.c_str());
      return false;
    }

    pcl::fromPCLPointCloud2(mesh.cloud, *reference_cloud_model_);

    if (reference_cloud_model_->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Reference cloud derived from STL is empty.");
      return false;
    }

    for (auto & pt : reference_cloud_model_->points) {
      pt.x *= static_cast<float>(reference_scale_);
      pt.y *= static_cast<float>(reference_scale_);
      pt.z *= static_cast<float>(reference_scale_);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*reference_cloud_model_, centroid);

    for (auto & pt : reference_cloud_model_->points) {
      pt.x -= centroid[0];
      pt.y -= centroid[1];
      pt.z -= centroid[2];
    }

    reference_cloud_down_ = downsampleCloud(reference_cloud_model_);

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded reference STL cloud | raw_points=%zu | downsampled_points=%zu",
      reference_cloud_model_->size(),
      reference_cloud_down_->size());

    return !reference_cloud_down_->empty();
  }

  bool transformPoseToTarget(
    const geometry_msgs::msg::PoseStamped & pose_camera,
    geometry_msgs::msg::PoseStamped & pose_global)
  {
    try {
      const bool available = tf_buffer_.canTransform(
        target_frame_,
        pose_camera.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_lookup_timeout_s_));

      if (!available) {
        RCLCPP_WARN(
          this->get_logger(),
          "TF not available yet from %s to %s",
          pose_camera.header.frame_id.c_str(),
          target_frame_.c_str());
        return false;
      }

      const auto tf = tf_buffer_.lookupTransform(
        target_frame_,
        pose_camera.header.frame_id,
        tf2::TimePointZero);

      tf2::doTransform(pose_camera, pose_global, tf);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Pose estimated in camera frame, but tf2 transform to %s failed: %s",
        target_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  bool estimatePoseFromCluster(
    const std::vector<ColoredPoint> & isolated_points,
    const std_msgs::msg::Header & cloud_header,
    geometry_msgs::msg::PoseStamped & pose_camera,
    geometry_msgs::msg::PoseStamped & pose_global,
    double & fitness)
  {
    if (!reference_loaded_ || reference_cloud_down_->empty()) {
      RCLCPP_WARN(this->get_logger(), "Reference STL cloud is not loaded, cannot run T3.");
      return false;
    }

    auto observed_cloud = std::make_shared<CloudT>();
    observed_cloud->points.reserve(isolated_points.size());

    for (const auto & p : isolated_points) {
      observed_cloud->points.emplace_back(p.x, p.y, p.z);
    }

    observed_cloud->width = static_cast<std::uint32_t>(observed_cloud->points.size());
    observed_cloud->height = 1;
    observed_cloud->is_dense = false;

    auto observed_down = downsampleCloud(observed_cloud);

    if (observed_down->size() < 10) {
      RCLCPP_WARN(
        this->get_logger(),
        "Observed object cloud too small after downsampling: %zu points",
        observed_down->size());
      return false;
    }

    Eigen::Vector4f ref_centroid;
    Eigen::Vector4f obs_centroid;
    pcl::compute3DCentroid(*reference_cloud_down_, ref_centroid);
    pcl::compute3DCentroid(*observed_down, obs_centroid);

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess(0, 3) = obs_centroid[0] - ref_centroid[0];
    initial_guess(1, 3) = obs_centroid[1] - ref_centroid[1];
    initial_guess(2, 3) = obs_centroid[2] - ref_centroid[2];

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(reference_cloud_down_);
    icp.setInputTarget(observed_down);
    icp.setMaxCorrespondenceDistance(static_cast<float>(icp_max_corr_m_));
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-8);

    CloudT aligned;
    icp.align(aligned, initial_guess);

    if (!icp.hasConverged()) {
      RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
      return false;
    }

    fitness = icp.getFitnessScore();
    if (fitness > icp_fitness_threshold_) {
      RCLCPP_WARN(
        this->get_logger(),
        "ICP converged but fitness is weak: %.6f",
        fitness);
    }

    const Eigen::Matrix4f transform = icp.getFinalTransformation();
    const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    quat.normalize();

    pose_camera.header = cloud_header;
    pose_camera.pose.position.x = transform(0, 3);
    pose_camera.pose.position.y = transform(1, 3);
    pose_camera.pose.position.z = transform(2, 3);
    pose_camera.pose.orientation.x = quat.x();
    pose_camera.pose.orientation.y = quat.y();
    pose_camera.pose.orientation.z = quat.z();
    pose_camera.pose.orientation.w = quat.w();

    return transformPoseToTarget(pose_camera, pose_global);
  }

  void republishLatestPose()
  {
    if (!latest_pose_valid_) {
      return;
    }

    latest_pose_global_.header.stamp = this->now();
    detected_pose_pub_->publish(latest_pose_global_);
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++frame_count_;

    const FieldLayout layout = getFieldLayout(*msg);
    if (!layout.valid()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Incoming cloud is missing x/y/z/rgb(or rgba) fields.");
      return;
    }

    if (msg->point_step == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Incoming cloud has point_step=0.");
      return;
    }

    const std::size_t num_points = msg->data.size() / msg->point_step;
    const std::size_t width = static_cast<std::size_t>(msg->width);
    const std::size_t height = static_cast<std::size_t>(msg->height);

    if (width == 0 || height == 0 || num_points == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Incoming cloud is empty.");
      return;
    }

    const bool organized = (width * height == num_points);

    int u_min = 0;
    int u_max = static_cast<int>(width) - 1;
    int v_min = 0;
    int v_max = static_cast<int>(height) - 1;

    if (organized) {
      u_min = std::clamp(
        static_cast<int>(roi_u_min_ratio_ * static_cast<double>(width)),
        0, static_cast<int>(width) - 1);
      u_max = std::clamp(
        static_cast<int>(roi_u_max_ratio_ * static_cast<double>(width)),
        0, static_cast<int>(width) - 1);
      v_min = std::clamp(
        static_cast<int>(roi_v_min_ratio_ * static_cast<double>(height)),
        0, static_cast<int>(height) - 1);
      v_max = std::clamp(
        static_cast<int>(roi_v_max_ratio_ * static_cast<double>(height)),
        0, static_cast<int>(height) - 1);

      if (u_min > u_max) std::swap(u_min, u_max);
      if (v_min > v_max) std::swap(v_min, v_max);
    }

    bool have_seed = false;
    int seed_u = -1;
    int seed_v = -1;
    float seed_x = 0.0f;
    float seed_y = 0.0f;
    float seed_z = std::numeric_limits<float>::infinity();

    std::size_t raw_valid_points = 0;
    std::size_t depth_pass_points = 0;

    if (organized) {
      for (int v = v_min; v <= v_max; ++v) {
        for (int u = u_min; u <= u_max; ++u) {
          const std::size_t idx =
            static_cast<std::size_t>(v) * width + static_cast<std::size_t>(u);
          const uint8_t * point_ptr = &msg->data[idx * msg->point_step];

          const float x = readFloat32(point_ptr, layout.x_offset);
          const float y = readFloat32(point_ptr, layout.y_offset);
          const float z = readFloat32(point_ptr, layout.z_offset);

          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
          }

          ++raw_valid_points;

          if (z < min_depth_m_ || z > max_depth_m_) {
            continue;
          }

          ++depth_pass_points;

          if (z < seed_z) {
            have_seed = true;
            seed_u = u;
            seed_v = v;
            seed_x = x;
            seed_y = y;
            seed_z = z;
          }
        }
      }
    } else {
      for (std::size_t idx = 0; idx < num_points; ++idx) {
        const uint8_t * point_ptr = &msg->data[idx * msg->point_step];

        const float x = readFloat32(point_ptr, layout.x_offset);
        const float y = readFloat32(point_ptr, layout.y_offset);
        const float z = readFloat32(point_ptr, layout.z_offset);

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        ++raw_valid_points;

        if (z < min_depth_m_ || z > max_depth_m_) {
          continue;
        }

        ++depth_pass_points;

        if (z < seed_z) {
          have_seed = true;
          seed_x = x;
          seed_y = y;
          seed_z = z;
        }
      }
    }

    if (!have_seed) {
      publishFilteredCloud(msg->header, {});
      if (frame_count_ == 1 || frame_count_ % 30 == 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "T2 weak detection | no seed found | raw_valid=%zu | depth_pass=%zu",
          raw_valid_points,
          depth_pass_points);
      }
      return;
    }

    std::vector<ColoredPoint> isolated_points;
    isolated_points.reserve(256);

    const double cluster_radius_sq = cluster_radius_m_ * cluster_radius_m_;
    const float max_allowed_z = seed_z + static_cast<float>(seed_depth_margin_m_);

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    if (organized) {
      const int local_u_min = std::max(0, seed_u - region_radius_px_);
      const int local_u_max = std::min(static_cast<int>(width) - 1, seed_u + region_radius_px_);
      const int local_v_min = std::max(0, seed_v - region_radius_px_);
      const int local_v_max = std::min(static_cast<int>(height) - 1, seed_v + region_radius_px_);

      for (int v = local_v_min; v <= local_v_max; ++v) {
        for (int u = local_u_min; u <= local_u_max; ++u) {
          const std::size_t idx =
            static_cast<std::size_t>(v) * width + static_cast<std::size_t>(u);
          const uint8_t * point_ptr = &msg->data[idx * msg->point_step];

          const float x = readFloat32(point_ptr, layout.x_offset);
          const float y = readFloat32(point_ptr, layout.y_offset);
          const float z = readFloat32(point_ptr, layout.z_offset);

          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
          }

          if (z < min_depth_m_ || z > max_depth_m_) {
            continue;
          }

          if (z > max_allowed_z) {
            continue;
          }

          const double dx = static_cast<double>(x - seed_x);
          const double dy = static_cast<double>(y - seed_y);
          const double dz = static_cast<double>(z - seed_z);
          const double dist_sq = dx * dx + dy * dy + dz * dz;

          if (dist_sq > cluster_radius_sq) {
            continue;
          }

          uint8_t r = 0;
          uint8_t g = 0;
          uint8_t b = 0;
          readPackedRgb(point_ptr, layout.color_offset, r, g, b);

          isolated_points.push_back({x, y, z, r, g, b});
          sum_x += x;
          sum_y += y;
          sum_z += z;
        }
      }
    } else {
      for (std::size_t idx = 0; idx < num_points; ++idx) {
        const uint8_t * point_ptr = &msg->data[idx * msg->point_step];

        const float x = readFloat32(point_ptr, layout.x_offset);
        const float y = readFloat32(point_ptr, layout.y_offset);
        const float z = readFloat32(point_ptr, layout.z_offset);

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        if (z < min_depth_m_ || z > max_depth_m_) {
          continue;
        }

        if (z > max_allowed_z) {
          continue;
        }

        const double dx = static_cast<double>(x - seed_x);
        const double dy = static_cast<double>(y - seed_y);
        const double dz = static_cast<double>(z - seed_z);
        const double dist_sq = dx * dx + dy * dy + dz * dz;

        if (dist_sq > cluster_radius_sq) {
          continue;
        }

        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        readPackedRgb(point_ptr, layout.color_offset, r, g, b);

        isolated_points.push_back({x, y, z, r, g, b});
        sum_x += x;
        sum_y += y;
        sum_z += z;
      }
    }

    publishFilteredCloud(msg->header, isolated_points);

    if (isolated_points.size() >= static_cast<std::size_t>(min_points_for_object_)) {
      const double cx = sum_x / static_cast<double>(isolated_points.size());
      const double cy = sum_y / static_cast<double>(isolated_points.size());
      const double cz = sum_z / static_cast<double>(isolated_points.size());

      if (frame_count_ == 1 || frame_count_ % 30 == 0) {
        if (organized) {
          RCLCPP_INFO(
            this->get_logger(),
            "T2 OK | raw_valid=%zu | depth_pass=%zu | isolated=%zu | seed_px=(%d,%d) | seed_xyz=(%.4f, %.4f, %.4f) | centroid=(%.4f, %.4f, %.4f)",
            raw_valid_points,
            depth_pass_points,
            isolated_points.size(),
            seed_u, seed_v,
            seed_x, seed_y, seed_z,
            cx, cy, cz);
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "T2 OK | raw_valid=%zu | depth_pass=%zu | isolated=%zu | seed_xyz=(%.4f, %.4f, %.4f) | centroid=(%.4f, %.4f, %.4f)",
            raw_valid_points,
            depth_pass_points,
            isolated_points.size(),
            seed_x, seed_y, seed_z,
            cx, cy, cz);
        }
      }

      geometry_msgs::msg::PoseStamped pose_camera;
      geometry_msgs::msg::PoseStamped pose_global;
      double icp_fitness = std::numeric_limits<double>::quiet_NaN();

      const bool pose_ok = estimatePoseFromCluster(
        isolated_points,
        msg->header,
        pose_camera,
        pose_global,
        icp_fitness);

      if (pose_ok) {
        latest_pose_global_ = pose_global;
        latest_pose_global_.header.stamp = this->now();
        latest_pose_valid_ = true;
        detected_pose_pub_->publish(latest_pose_global_);
      }

      if (frame_count_ == 1 || frame_count_ % 30 == 0) {
        if (pose_ok) {
          RCLCPP_INFO(
            this->get_logger(),
            "T3 OK | camera_frame=%s | target_frame=%s | fitness=%.6f | camera_xyz=(%.4f, %.4f, %.4f) | global_xyz=(%.4f, %.4f, %.4f)",
            pose_camera.header.frame_id.c_str(),
            pose_global.header.frame_id.c_str(),
            icp_fitness,
            pose_camera.pose.position.x,
            pose_camera.pose.position.y,
            pose_camera.pose.position.z,
            pose_global.pose.position.x,
            pose_global.pose.position.y,
            pose_global.pose.position.z);

          RCLCPP_INFO(
            this->get_logger(),
            "T4 OK | published latest pose on /detected_object_pose and cached it for /detect_object");
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "T3 weak detection | registration succeeded or partially succeeded, but transform to target frame is not available yet");
        }
      }
    } else {
      if (frame_count_ == 1 || frame_count_ % 30 == 0) {
        if (organized) {
          RCLCPP_WARN(
            this->get_logger(),
            "T2 weak detection | raw_valid=%zu | depth_pass=%zu | isolated=%zu | need_at_least=%d | seed_px=(%d,%d) | seed_xyz=(%.4f, %.4f, %.4f)",
            raw_valid_points,
            depth_pass_points,
            isolated_points.size(),
            min_points_for_object_,
            seed_u, seed_v,
            seed_x, seed_y, seed_z);
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "T2 weak detection | raw_valid=%zu | depth_pass=%zu | isolated=%zu | need_at_least=%d | seed_xyz=(%.4f, %.4f, %.4f)",
            raw_valid_points,
            depth_pass_points,
            isolated_points.size(),
            min_points_for_object_,
            seed_x, seed_y, seed_z);
        }
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr isolated_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detected_pose_pub_;
  rclcpp::Service<DetectObject>::SharedPtr detect_object_srv_;
  rclcpp::TimerBase::SharedPtr pose_republish_timer_;

  std::size_t frame_count_;

  double min_depth_m_;
  double max_depth_m_;

  double roi_u_min_ratio_;
  double roi_u_max_ratio_;
  double roi_v_min_ratio_;
  double roi_v_max_ratio_;

  double seed_depth_margin_m_;
  double cluster_radius_m_;
  int region_radius_px_;
  int min_points_for_object_;

  std::string reference_stl_path_;
  std::string target_frame_;
  double reference_scale_;
  double voxel_leaf_size_m_;
  double icp_max_corr_m_;
  int icp_max_iterations_;
  double icp_fitness_threshold_;
  double tf_lookup_timeout_s_;
  int pose_publish_period_ms_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  CloudT::Ptr reference_cloud_model_;
  CloudT::Ptr reference_cloud_down_;
  bool reference_loaded_{false};

  geometry_msgs::msg::PoseStamped latest_pose_global_;
  bool latest_pose_valid_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
