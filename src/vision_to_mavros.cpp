#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mavros_msgs/msg/landing_target.hpp>

#include <string.h>

using std::placeholders::_1;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("vision_to_mavros");

  //////////////////////////////////////////////////
  // Variables for precision navigation
  //////////////////////////////////////////////////
  auto camera_pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("vision_pose", 10);
  auto body_path_pubisher = node->create_publisher<nav_msgs::msg::Path>("body_frame/path", 1);

  std::unique_ptr<tf2_ros::Buffer> tfBuffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  // TransformStamped is returned by lookupTransform. We need to convert this msg to tf2::Transform before using it
  // like it was used in ros1
  tf2::Stamped<tf2::Transform> transform;

  geometry_msgs::msg::TransformStamped tf;

  geometry_msgs::msg::PoseStamped msg_body_pose;

  nav_msgs::msg::Path body_path;

  std::string target_frame_id = "camera_odom_frame";

  std::string source_frame_id = "camera_link";

  double output_rate = 20, roll_cam = 0, pitch_cam = 0, yaw_cam = 1.5707963, gamma_world = -1.5707963;

  // Read parameters from launch file, including: target_frame_id, source_frame_id, output_rate
  {
    // The frame in which we find the transform into, the original "world" frame
    node->declare_parameter<std::string>("target_frame_id", target_frame_id.c_str());
    if (node->get_parameter("target_frame_id", target_frame_id))
    {
      RCLCPP_INFO(node->get_logger(), "Get target_frame_id parameter: %s", target_frame_id.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default target_frame_id: %s", target_frame_id.c_str());
    }

    // The frame for which we find the tranform to target_frame_id, the original "camera" frame
    node->declare_parameter<std::string>("source_frame_id", source_frame_id.c_str());
    if (node->get_parameter("source_frame_id", source_frame_id))
    {
      RCLCPP_INFO(node->get_logger(), "Get source_frame_id parameter: %s", source_frame_id.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default source_frame_id: %s", source_frame_id.c_str());
    }

    // The rate at which we wish to publish final pose data
    node->declare_parameter<double>("output_rate", output_rate);
    if (node->get_parameter("output_rate", output_rate))
    {
      RCLCPP_INFO(node->get_logger(), "Get output_rate parameter: %f", output_rate);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default output_rate: %f", output_rate);
    }

    // The rotation around z axis between original world frame and target world frame, assuming the z axis needs not to
    // be changed In this case, target world frame has y forward, x to the right and z upwards (ENU as ROS dictates)
    node->declare_parameter<double>("gamma_world", gamma_world);
    if (node->get_parameter("gamma_world", gamma_world))
    {
      RCLCPP_INFO(node->get_logger(), "Get gamma_world parameter: %f", gamma_world);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default gamma_world: %f", gamma_world);
    }

    // The roll angle around camera's own axis to align with body frame
    node->declare_parameter<double>("roll_cam", roll_cam);
    if (node->get_parameter("roll_cam", roll_cam))
    {
      RCLCPP_INFO(node->get_logger(), "Get roll_cam parameter: %f", roll_cam);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default roll_cam: %f", roll_cam);
    }

    // The pitch angle around camera's own axis to align with body frame
    node->declare_parameter<double>("pitch_cam", pitch_cam);
    if (node->get_parameter("pitch_cam", pitch_cam))
    {
      RCLCPP_INFO(node->get_logger(), "Get pitch_cam parameter: %f", pitch_cam);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default pitch_cam: %f", pitch_cam);
    }

    // The yaw angle around camera's own axis to align with body frame
    node->declare_parameter<double>("yaw_cam", yaw_cam);
    if (node->get_parameter("yaw_cam", yaw_cam))
    {
      RCLCPP_INFO(node->get_logger(), "Get yaw_cam parameter: %f", yaw_cam);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default yaw_cam: %f", yaw_cam);
    }
  }

  //////////////////////////////////////////////////
  // Variables for precision landing (optional)
  //////////////////////////////////////////////////
  bool enable_precland = true;

  std::string precland_target_frame_id = "36h11:1";

  std::string precland_camera_frame_id = "camera_frame";

  rclcpp::Publisher<mavros_msgs::msg::LandingTarget>::SharedPtr precland_msg_publisher;

  node->declare_parameter<bool>("enable_precland", enable_precland);
  if (node->get_parameter("enable_precland", enable_precland))
  {
    RCLCPP_INFO(node->get_logger(), "Precision landing: %s", enable_precland ? "enabled" : "disabled");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Precision landing disabled by default");
  }

  if (enable_precland)
  {
    // The frame of the landing target in the camera frame
    node->declare_parameter<std::string>("precland_target_frame_id", precland_target_frame_id.c_str());
    if (node->get_parameter("precland_target_frame_id", precland_target_frame_id))
    {
      RCLCPP_INFO(node->get_logger(), "Get precland_target_frame_id parameter: %s", precland_target_frame_id.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default precland_target_frame_id: %s", precland_target_frame_id.c_str());
    }
    node->declare_parameter<std::string>("precland_camera_frame_id", precland_camera_frame_id.c_str());
    if (node->get_parameter("precland_camera_frame_id", precland_camera_frame_id))
    {
      RCLCPP_INFO(node->get_logger(), "Get precland_camera_frame_id parameter: %s", precland_camera_frame_id.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Using default precland_camera_frame_id: %s", precland_camera_frame_id.c_str());
    }

    precland_msg_publisher = node->create_publisher<mavros_msgs::msg::LandingTarget>("landing_raw", 10);
  }

  //////////////////////////////////////////////////
  // Wait for the first transform to become available.
  //////////////////////////////////////////////////
  tfBuffer->canTransform(target_frame_id, source_frame_id, node->get_clock()->now(), rclcpp::Duration(3.0));
  // tf_listener.waitForTransform(target_frame_id, source_frame_id, node->get_clock()->now(), rclcpp::Duration(3.0));

  rclcpp::Time last_tf_time = node->get_clock()->now();
  rclcpp::Time last_precland_tf_time = node->get_clock()->now();

  // Limit the rate of publishing data, otherwise the other telemetry port might be flooded
  rclcpp::Rate rate(output_rate);

  while (rclcpp::ok())
  {
    //////////////////////////////////////////////////
    // Publish vision_position_estimate message if transform is available
    //////////////////////////////////////////////////
    try
    {
      //    will give the transfrom from frame_1 to frame_2
      tf = tfBuffer->lookupTransform(target_frame_id, source_frame_id, tf2::TimePointZero);
      // returns geometry_msgs/transform_stamped, so we need to convert to this to tf2::Stamped<tf2::Transform>
      // alternative: tf2::convert(in, out)
      tf2::fromMsg(tf, transform);

      // Only publish pose messages when we have new transform data.
      // TODO: maybe we can go back to transform.stamp_ here?
      if (last_tf_time < tf.header.stamp)
      {
        last_tf_time = tf.header.stamp;

        static tf2::Vector3 position_orig, position_body;

        static tf2::Quaternion quat_cam, quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z, quat_rot_z,
            quat_body;

        // 1) Rotation from original world frame to world frame with y forward.
        // See the full rotation matrix at https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        position_orig = transform.getOrigin();

        position_body.setX(cos(gamma_world) * position_orig.getX() + sin(gamma_world) * position_orig.getY());
        position_body.setY(-sin(gamma_world) * position_orig.getX() + cos(gamma_world) * position_orig.getY());
        position_body.setZ(position_orig.getZ());

        // 2) Rotation from camera to body frame.
        quat_cam = transform.getRotation();

        quat_cam_to_body_x.setRPY(roll_cam, 0, 0);
        quat_cam_to_body_y.setRPY(0, pitch_cam, 0);
        quat_cam_to_body_z.setRPY(0, 0, yaw_cam);

        // 3) Rotate body frame 90 degree (align body x with world y at launch)
        quat_rot_z.setRPY(0, 0, -gamma_world);

        quat_body = quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
        quat_body.normalize();

        // Create PoseStamped message to be sent
        msg_body_pose.header.stamp = tf.header.stamp;
        msg_body_pose.header.frame_id = transform.frame_id_;
        msg_body_pose.pose.position.x = position_body.getX();
        msg_body_pose.pose.position.y = position_body.getY();
        msg_body_pose.pose.position.z = position_body.getZ();
        msg_body_pose.pose.orientation.x = quat_body.getX();
        msg_body_pose.pose.orientation.y = quat_body.getY();
        msg_body_pose.pose.orientation.z = quat_body.getZ();
        msg_body_pose.pose.orientation.w = quat_body.getW();

        // Publish pose of body frame in world frame
        camera_pose_publisher->publish(msg_body_pose);

        // Publish trajectory path for visualization
        body_path.header.stamp = msg_body_pose.header.stamp;
        body_path.header.frame_id = msg_body_pose.header.frame_id;
        body_path.poses.push_back(msg_body_pose);
        body_path_pubisher->publish(body_path);
      }
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(node->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    //////////////////////////////////////////////////
    // Publish landing_target message if option is enabled and transform is available
    //////////////////////////////////////////////////
    if (enable_precland)
    {
      if (tfBuffer->canTransform(precland_camera_frame_id, precland_target_frame_id, tf2::TimePointZero))
      // if (tf_listener.canTransform(precland_camera_frame_id, precland_target_frame_id, now))
      {
        //    will give the transfrom from frame_1 to frame_2
        tf = tfBuffer->lookupTransform(precland_camera_frame_id, precland_target_frame_id, tf2::TimePointZero);
        // returns geometry_msgs/transform_stamped, so we need to convert to this to tf2::Stamped<tf2::Transform>
        // alternative: tf2::convert(in, out)
        tf2::fromMsg(tf, transform);

        // Only publish when we have new data
        if (last_precland_tf_time < tf.header.stamp)
        {
          last_precland_tf_time = tf.header.stamp;

          mavros_msgs::msg::LandingTarget msg_landing_target;

          // Setup the landing target message according to the relative protocol:
          // https://mavlink.io/en/services/landing_target.html#camera_image_relative
          msg_landing_target.header.frame_id = transform.frame_id_;
          msg_landing_target.header.stamp = tf.header.stamp;
          msg_landing_target.target_num = 0;
          msg_landing_target.frame = mavros_msgs::msg::LandingTarget::LOCAL_NED;
          msg_landing_target.type = mavros_msgs::msg::LandingTarget::VISION_FIDUCIAL;

          msg_landing_target.angle[0] = std::atan(transform.getOrigin().getX() / transform.getOrigin().getZ());
          msg_landing_target.angle[1] = std::atan(transform.getOrigin().getY() / transform.getOrigin().getZ());
          msg_landing_target.distance = transform.getOrigin().length();

          // Publish the message
          precland_msg_publisher->publish(msg_landing_target);

          RCLCPP_INFO(node->get_logger(), "Landing target detected");
        }
      }
    }

    //////////////////////////////////////////////////
    // Repeat
    //////////////////////////////////////////////////
    rate.sleep();
  }
  return 0;
}