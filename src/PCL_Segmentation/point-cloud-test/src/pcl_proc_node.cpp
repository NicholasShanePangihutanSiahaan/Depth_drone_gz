#include <algorithm>
#include <deque>
#include <limits>
#include <mutex>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/console/print.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pcl_cstm_msg/msg/point_cloud_array.hpp"
#include "pcl_cstm_msg/msg/v_cylinders_fit.hpp"
#include "pcl_cstm_msg/msg/tracked_cylinder_array.hpp"
#include "point-cloud-test/pcl_processor.h"
#include "point-cloud-test/pcl_filter.h"
#include "point-cloud-test/global_cylinder_manager.hpp"

namespace point_cloud_test
{

  struct CloudOdomPair
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  };

  class PclProcNode : public rclcpp::Node
  {
  public:
    PclProcNode()
        : Node("pcl_proc_node")
    {
      // A physical ZED ROS wrapper publishes optical axes (Z forward), while
      // Gazebo Sim's RGB-D PointCloudPacked uses sensor axes (X forward).
      // Keep this explicit instead of silently rotating every source.
      input_is_optical_frame_ = declare_parameter<bool>(
          "input_is_optical_frame", true);
      max_pose_time_error_ = declare_parameter<double>(
          "max_pose_time_error", 0.20);
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose", rclcpp::SensorDataQoS(),
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
          {
            std::lock_guard<std::mutex> lock(swap_mutex_);
            pose_history_.push_back(msg);
            while (pose_history_.size() > 200)
            {
              pose_history_.pop_front();
            }
          });
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "/input_cloud", rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
          {
            cloud_callback(msg);
          });

      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/output_cloud", rclcpp::SensorDataQoS());

      cluster_pub_ = create_publisher<pcl_cstm_msg::msg::PointCloudArray>(
          "/clusters", rclcpp::SensorDataQoS());

      cylinder_pub_ = create_publisher<pcl_cstm_msg::msg::VCylindersFit>(
          "/cylinders", rclcpp::SensorDataQoS());

      global_cylinder_pub_ = create_publisher<pcl_cstm_msg::msg::TrackedCylinderArray>(
          "/global/cylinders", rclcpp::SensorDataQoS());

      timer_ = create_wall_timer(
          std::chrono::milliseconds(500),
          [this]()
          { timer_callback(); });
    }

  private:
    void cloud_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
    {
      std::lock_guard<std::mutex> lock(swap_mutex_);
      if (pose_history_.empty())
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Point cloud diterima tetapi /pose belum tersedia; frame diabaikan");
        return;
      }
      // Jangan biarkan frame menumpuk ketika backend PCL sedang sibuk.
      // Cukup simpan dua pasangan cloud-odometry terbaru.
      constexpr std::size_t kMaxBufferedPairs = 2;

      const double cloud_time = stamp_seconds(cloud_msg->header.stamp);
      const double newest_pose_time = stamp_seconds(
          pose_history_.back()->header.stamp);
      auto closest = pose_history_.back();
      double closest_error = 0.0;
      const bool different_clock_domains =
          std::abs(newest_pose_time - cloud_time) > 60.0;
      if (!different_clock_domains)
      {
        closest_error = std::numeric_limits<double>::infinity();
        for (const auto &pose : pose_history_)
        {
          const double error = std::abs(
              stamp_seconds(pose->header.stamp) - cloud_time);
          if (error < closest_error)
          {
            closest_error = error;
            closest = pose;
          }
        }
      }
      if (closest_error > max_pose_time_error_)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Cloud ditolak: selisih timestamp cloud-pose %.3f s", closest_error);
        return;
      }
      write_buffer_->push_back({cloud_msg, closest});

      if (write_buffer_->size() > kMaxBufferedPairs)
      {
        write_buffer_->erase(
            write_buffer_->begin(),
            write_buffer_->end() - kMaxBufferedPairs);
      }
    }

    void timer_callback()
    {
      auto timer_cb_start = std::chrono::high_resolution_clock::now();
      std::shared_ptr<std::vector<CloudOdomPair>> to_process;
      {
        std::lock_guard<std::mutex> lock(swap_mutex_);
        std::swap(write_buffer_, process_buffer_);
        to_process = process_buffer_;
      }

      if (to_process->empty())
      {
        return;
      }

      // Reduce ke 700k jika lebih dari 1,2 juta point cloud

      int total_point_clouds = 0;
      for (const auto &pair : *to_process)
      {
        total_point_clouds += static_cast<int>(
            pair.cloud->width * pair.cloud->height);
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);

      auto time_filter_start = std::chrono::high_resolution_clock::now();

      for (const auto &pair : *to_process)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pair.cloud, *cloud);

        // Gazebo RGB-D point clouds use NaN for pixels without a valid depth.
        // Its bridge may nevertheless mark the message as dense. PCL skips
        // finite-point removal for dense clouds, so do not trust that metadata.
        cloud->is_dense = false;
        // KdTree / normal estimation asserts when one of those points reaches
        // radiusSearch(), so sanitize every frame before any PCL operation.
        std::vector<int> finite_indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, finite_indices);
        if (cloud->empty())
        {
          continue;
        }
        pcl::RandomSample<pcl::PointXYZ> random_sampler;

        if (total_point_clouds > 1200000)
        {
          //RCLCPP_INFO(get_logger(), "Resampling");
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
          unsigned int samples_per_pair = (600000 / to_process->size());
          random_sampler.setInputCloud(cloud);
          random_sampler.setSample(samples_per_pair);

          random_sampler.filter(*cloud_sampled);
          cloud = cloud_sampled;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        // 0.30 m terlalu kasar untuk batang beradius 0.35--0.55 m dan dapat
        // menyisakan terlalu sedikit sampel untuk cylinder fitting.
        voxel.setLeafSize(0.15f, 0.15f, 0.15f);
        voxel.filter(*filtered);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(filtered);
        sor.setMeanK(50);            // Default often used is 50
        sor.setStddevMulThresh(1.0); // Default often used is 1.0
        sor.filter(*filtered);

        const auto &pos = pair.pose->pose.position;
        const auto &q = pair.pose->pose.orientation;
        Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
        Eigen::Matrix3f R = rotation.toRotationMatrix();
        Eigen::Vector3f t(pos.x, pos.y, pos.z);

        for (const auto &pt : filtered->points)
        {
          if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
              !std::isfinite(pt.z))
          {
            continue;
          }
          Eigen::Vector3f sensor_pt(pt.x, pt.y, pt.z);
          Eigen::Vector3f robot_pt = input_is_optical_frame_
              ? R_optical_to_robot_ * sensor_pt
              : sensor_pt;
          Eigen::Vector3f global_pt = R * robot_pt + t;
          pcl::PointXYZ p;
          p.x = global_pt.x();
          p.y = global_pt.y();
          p.z = global_pt.z();
          if (std::isfinite(p.x) && std::isfinite(p.y) &&
              std::isfinite(p.z))
          {
            merged_cloud->push_back(p);
          }
        }
      }

      auto time_filter_end = std::chrono::high_resolution_clock::now();
      double time_filter_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(time_filter_end - time_filter_start)
              .count() /
          1000.0;
      RCLCPP_INFO(get_logger(), "Total Time Filter: %lf ms, array size: %ld", time_filter_ms, to_process->size());

      merged_cloud->width = merged_cloud->size();
      merged_cloud->height = 1;
      merged_cloud->is_dense = true;

      if (merged_cloud->empty())
      {
        RCLCPP_WARN(get_logger(), "No finite points available after filtering");
        age_and_publish_tracks();
        to_process->clear();
        return;
      }

      auto time_ransac_start = std::chrono::high_resolution_clock::now();
      auto non_ground = processRANSAC(merged_cloud);
      if (!non_ground || non_ground->empty())
      {
        RCLCPP_WARN(get_logger(), "Ground removal failed");
        age_and_publish_tracks();
        to_process->clear();
        return;
      }
      auto time_ransac_end = std::chrono::high_resolution_clock::now();
      double time_ransac_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(time_ransac_end - time_ransac_start)
              .count() /
          1000.0;
      RCLCPP_INFO(get_logger(), "Total Time RANSAC: %lf ms", time_ransac_ms);

      auto trunk_filter = removeNonNormals(non_ground);

      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*trunk_filter, output_msg);
      output_msg.header.stamp = now();
      output_msg.header.frame_id = "plantation";

      cloud_pub_->publish(output_msg);

      auto time_cluster_start = std::chrono::high_resolution_clock::now();
      auto clusters = clusterTrees_RegionGrowing(trunk_filter, 15, 1);
      auto time_cluster_end = std::chrono::high_resolution_clock::now();

      double time_cluster_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(time_cluster_end - time_cluster_start)
              .count() /
          1000.0;
      RCLCPP_INFO(get_logger(), "Total Time Cluster: %lf ms, total_clusters: %ld", time_cluster_ms, clusters.size());

      if (!clusters.empty())
      {
        pcl_cstm_msg::msg::PointCloudArray cluster_msg;
        cluster_msg.header.stamp = now();
        cluster_msg.header.frame_id = "plantation";

        pcl_cstm_msg::msg::VCylindersFit cyl_array_msg;
        cyl_array_msg.header.stamp = now();
        cyl_array_msg.header.frame_id = "plantation";

        std::vector<CylinderParams> params_vec;
        params_vec.reserve(clusters.size());

        int is_valid = 0;
        int is_point = 0;
        auto time_fit_start = std::chrono::high_resolution_clock::now();
        for (const auto &cluster : clusters)
        {
          sensor_msgs::msg::PointCloud2 cluster_cloud;
          pcl::toROSMsg(*cluster, cluster_cloud);
          cluster_cloud.header.stamp = now();
          cluster_cloud.header.frame_id = "plantation";
          cluster_msg.clouds.push_back(std::move(cluster_cloud));

          auto params = fitCylinderZAxis(cluster);
          params_vec.push_back(params);

          pcl_cstm_msg::msg::CylinderFit cyl_msg;
          cyl_msg.header.stamp = now();
          cyl_msg.header.frame_id = "plantation";
          cyl_msg.radius = params.radius;
          cyl_msg.height = params.height;
          cyl_msg.confidence = params.confidence;

          cyl_msg.pose.position.x = params.center_x;
          cyl_msg.pose.position.y = params.center_y;
          cyl_msg.pose.position.z = params.center_z;

          Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
          Eigen::Vector3f cylinder_axis(params.dir_x, params.dir_y, params.dir_z);
          if (cylinder_axis.z() < 0)
          {
            cylinder_axis = -cylinder_axis;
          }

          Eigen::Quaternionf q;
          q.setFromTwoVectors(z_axis, cylinder_axis);

          cyl_msg.pose.orientation.x = q.x();
          cyl_msg.pose.orientation.y = q.y();
          cyl_msg.pose.orientation.z = q.z();
          cyl_msg.pose.orientation.w = q.w();

          cyl_msg.is_valid = params.isValid;

          if (params.isValid)
          {
            is_valid++;
          }
          if (params.clouds && !params.clouds->points.empty())
          {
            // Convert the point cloud
            is_point++;
            pcl::toROSMsg(*params.clouds, cyl_msg.clouds);

            cyl_msg.clouds.header = cyl_msg.header;
          }

          cyl_array_msg.cylinders.push_back(std::move(cyl_msg));
        }
        auto time_fit_end = std::chrono::high_resolution_clock::now();

        double time_fit_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(time_fit_end - time_fit_start)
                .count() /
            1000.0;
        RCLCPP_INFO(get_logger(), "Total Fit: %lf ms, valid: %d %d", time_fit_ms, is_valid, is_point);

        cluster_pub_->publish(cluster_msg);
        if (!cyl_array_msg.cylinders.empty())
        {
          cylinder_pub_->publish(cyl_array_msg);
        }

        std::vector<TrackedCylinder> tracked;

        auto time_manager_start = std::chrono::high_resolution_clock::now();
        global_manager_.process(params_vec, tracked);
        auto time_manager_end = std::chrono::high_resolution_clock::now();

        double time_manager_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(time_manager_end - time_manager_start)
                .count() /
            1000.0;
        RCLCPP_INFO(get_logger(), "Total Manager: %lf ms", time_manager_ms);

        publish_tracks(tracked);
      }
      else
      {
        // An empty perception frame is still evidence. Previously the global
        // manager was never called here, so stale cylinders could live forever
        // and keep driving the mapper/FSM toward a non-existent tree.
        std::vector<CylinderParams> no_detections;
        std::vector<TrackedCylinder> tracked;
        global_manager_.process(no_detections, tracked);

        publish_tracks(tracked);
      }

      to_process->clear();
      auto timer_cb_end = std::chrono::high_resolution_clock::now();

      double timer_cb_ms =
          std::chrono::duration_cast<std::chrono::microseconds>(timer_cb_end - timer_cb_start)
              .count() /
          1000.0;
      RCLCPP_INFO(get_logger(), "Total Point Processed: %d", total_point_clouds);
      RCLCPP_INFO(get_logger(), "Total Times: %lf ms", timer_cb_ms);
    }

    static double stamp_seconds(const builtin_interfaces::msg::Time &stamp)
    {
      return static_cast<double>(stamp.sec) +
             static_cast<double>(stamp.nanosec) * 1e-9;
    }

    void age_and_publish_tracks()
    {
      std::vector<CylinderParams> empty;
      std::vector<TrackedCylinder> tracked;
      global_manager_.process(empty, tracked);
      publish_tracks(tracked);
    }

    void publish_tracks(const std::vector<TrackedCylinder> &tracked)
    {
      pcl_cstm_msg::msg::TrackedCylinderArray msg;
      msg.header.stamp = now();
      msg.header.frame_id = "odom";
      for (const auto &t : tracked)
      {
        pcl_cstm_msg::msg::TrackedCylinder item;
        item.id = t.id;
        item.seen_count = t.seen_count;
        item.missed_count = t.missed_count;
        item.cylinder.header = msg.header;
        item.cylinder.radius = t.radius;
        item.cylinder.height = t.height;
        item.cylinder.confidence = t.confidence;
        item.cylinder.pose.position.x = t.center_x;
        item.cylinder.pose.position.y = t.center_y;
        item.cylinder.pose.position.z = t.center_z;
        item.cylinder.pose.orientation.w = 1.0;
        item.cylinder.is_valid = true;
        msg.cylinders.push_back(std::move(item));
      }
      global_cylinder_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::deque<geometry_msgs::msg::PoseStamped::ConstSharedPtr> pose_history_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::PointCloudArray>::SharedPtr cluster_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::VCylindersFit>::SharedPtr cylinder_pub_;
    rclcpp::Publisher<pcl_cstm_msg::msg::TrackedCylinderArray>::SharedPtr global_cylinder_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<std::vector<CloudOdomPair>> write_buffer_{
        std::make_shared<std::vector<CloudOdomPair>>()};
    std::shared_ptr<std::vector<CloudOdomPair>> process_buffer_{
        std::make_shared<std::vector<CloudOdomPair>>()};
    std::mutex swap_mutex_;
    bool input_is_optical_frame_{true};
    double max_pose_time_error_{0.20};

    Eigen::Matrix3f R_optical_to_robot_{
        (Eigen::Matrix3f() << 0, 0, 1,
         -1, 0, 0,
         0, -1, 0)
            .finished()};

    GlobalCylinderManager global_manager_{2.0f};
  };

} // namespace point_cloud_test

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<point_cloud_test::PclProcNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
