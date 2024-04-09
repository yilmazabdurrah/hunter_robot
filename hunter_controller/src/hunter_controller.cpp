// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Hariharan Arunachalam
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>

#include "hunter_controller/hunter_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <sys/socket.h>
#include <arpa/inet.h>
// #include "std_msgs/msg/string.hpp"

#include <netinet/in.h> 
using json = nlohmann::json;
using namespace nlohmann::literals;

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace hunter_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

HunterController::HunterController() : controller_interface::ControllerInterface() {}

const char * HunterController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn HunterController::on_init()
{ 
  std::cout<< "==========init controller plugin========" << std::endl;
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // nlohmann::json_abi_v3_11_2::json message =  {{"stamp",'2023-09-08T16:14:29.0000000+01:00'},{"velocity", {{"x",0.0} , {"y",0.0}, {"z",0.0}}} , {"angulary",{{"x",0.0} ,{"y",0.0} , {"z",0.0}}} , {"eStop",false}};
  // struct sockaddr_in hostaddr , robotaddr;
  // memset(&hostaddr , 0 , sizeof(hostaddr));
  memset(&hostaddr , 0 , sizeof(hostaddr));
  memset(&robotaddr , 0 , sizeof(robotaddr));
  memset(&recaddr , 0 ,  sizeof(recaddr) );
  // memset(&robotaddr , 0 , sizeof(robotaddr));
  // int sockfd = socket(AF_INET , SOCK_DGRAM  ,0 );
  // bool connected_flag =  false;
  // char buffer[1024];
  // publisher_ = get_node()->create_publisher<std_msgs::msg::String>("/hunter/battery_status", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration HunterController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  // for (const auto & joint_name : params_.left_wheel_names)
  // {
  //   conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  // }
  // for (const auto & joint_name : params_.right_wheel_names)
  // {
  //   conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  // }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration HunterController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  // for (const auto & joint_name : params_.left_wheel_names)
  // {
  //   conf_names.push_back(joint_name + "/" + feedback_type());
  // }
  // for (const auto & joint_name : params_.right_wheel_names)
  // {
  //   conf_names.push_back(joint_name + "/" + feedback_type());
  // }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

void HunterController::update_time()
{

  // std::cout<< "============1" << std::endl;
  time_t now = time(0);
  tm *ltm = localtime(&now);
  std::string year_str = add_zero(ltm->tm_year+1900);
  std::string month_str = add_zero(ltm->tm_mon);
  std::string date_str = add_zero(ltm->tm_mday);
  std::string hour_str = add_zero(ltm->tm_hour);
  std::string min_str = add_zero(ltm->tm_min);
  std::string sec_str = add_zero(ltm->tm_sec);
  message["stamp"] = year_str + "-" + month_str + "-" + date_str + "T" + hour_str + ":" + min_str + ":" + sec_str + ".0000000+01:00";
}

std::string HunterController::add_zero(int value){
    std::string value_str;
    // cout << value;
    if(value < 10){
        // cout << value;
        value_str = std::to_string(value);
        value_str = "0" + value_str;
        // cout << value_str;
    }
    else{
        value_str = std::to_string(value);
        // cout << value_str;
    }
    return value_str;
}

controller_interface::return_type HunterController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }
  // std::cout << is_halted << std::endl;
  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);
  // std::cout << last_command_msg << std::endl;
  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  // std::cout << age_of_last_command << std::endl; 
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_command_msg;
  double & linear_command = command.twist.linear.x;
  double & angular_command = command.twist.angular.z;

  previous_update_timestamp_ = time;

  // Apply (possibly new) multipliers:
  // const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  // const double left_wheel_radius = params_.left_wheel_radius_multiplier * params_.wheel_radius;
  // const double right_wheel_radius = params_.right_wheel_radius_multiplier * params_.wheel_radius;
  // std::cout << params_.position_feedback << std::endl;

  // NEED TO CHECK FOR ODOMETRY UPDATION PROCESS

  // if (params_.open_loop)
  // {
  //   odometry_.updateOpenLoop(linear_command, angular_command, time);
  // }
  // else
  // {
  //   if (params_.position_feedback)
  //   {
  //     odometry_.update(linear_command, angular_command, time);
  //   }
  //   else
  //   {
  //     odometry_.updateFromVelocity(
  //       linear_command * period.seconds(),
  //       angular_command * period.seconds(), time);
  //   }
  // }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  bool should_publish = false;
  try
  {
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  }
  catch (const std::runtime_error &)
  {
    // Handle exceptions when the time source changes and initialize publish timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  // if (should_publish)
  {
    // if (realtime_odometry_publisher_->trylock())
    {len = sizeof(recaddr);
       n = recvfrom(sockfd, (char *)buffer, 1024,  
                MSG_WAITALL, ( struct sockaddr *) &recaddr, 
                &len); 
      // n = recvfrom(sockfd , (char *)buffer  , 1024 , MSG_WAITALL , ( struct sockaadr *) , &recaddr , &len);
      buffer[n] = '\0';
      // std::string rec_s = buffer;
      // rec_s.erase(std::remove(rec_s.begin(), rec_s.end(), '\n'), rec_s.cend());
      // rec_s.erase(std::remove(rec_s.begin(), rec_s.end(), ' ') , rec_s.cend());
      // rec_s.erase(std::remove(rec_s.begin(), rec_s.end(), '\\') , rec_s.cend());
      json received_data = json::parse(buffer);
      // std::cout << received_data["pose"]["position"] <<std::endl;
      // json rec_message = std::string(rec_s);
      // json rec_message =  ; 
      // json rec_message = json::array(rec_s);
      // json rec_message = rec_s;
      // std::cout <<rec_message<< std::endl;
      // std::cout << "TRYING TO PUBLISH IN REALTIME " << std::endl;
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      // std::cout << received_data["pose"] << std::endl;
      tf2::Quaternion q;
      // auto message = std_msgs::msg::String();
      // message.data = static_cast<std::string>(received_data);
      // publisher_->publish(message);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      q.setRPY(0, 0,received_data["pose"]["orientation"]["z"] );
      odometry_message.pose.pose.position.x = received_data["pose"]["position"]["x"];//odometry_.getX();
      odometry_message.pose.pose.position.y = received_data["pose"]["position"]["y"];//odometry_.getY();
      // Eigen::Quaterniond quat = Eigen::AngleAxisd(received_data["pose"]["orientation"]["x"], Eigen::Vector3d::UnitX())
                            // * Eigen::AngleAxisd(received_data["pose"]["orientation"]["y"], Eigen::Vector3d::UnitY())
                            // * Eigen::AngleAxisd(received_data["pose"]["orientation"]["z"], Eigen::Vector3d::UnitZ());
      odometry_message.pose.pose.orientation.x = q.x(); //orientation.x();
      odometry_message.pose.pose.orientation.y = q.y();//orientation.y();
      odometry_message.pose.pose.orientation.z = q.z(); //orientation.z();
      odometry_message.pose.pose.orientation.w = q.w(); //orientation.w();
      odometry_message.twist.twist.linear.x =received_data["twist"]["linear"]["x"];// odometry_.getLinear();
      odometry_message.twist.twist.angular.z = received_data["twist"]["angular"]["z"];//odometry_.getAngular();
      // std::cout<< odometry_message.twist.twist.linear.x  << std::endl;
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(
    linear_command, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }
  // float x = x_value;
  // float z = z_value;
  message["velocity"]["x"] = linear_command ;
  // if ((linear_command >= 0.0 && linear_command < 0.05)  || (linear_command <= 0.0 && linear_command > -0.05)){
  //   message["angulary"]["z"] = 0.0; }
  // else {
    message["angulary"]["z"] = angular_command;//}
    // std::cout<< "============before_update_time" << std::endl;

  update_time();
  // std::cout << message << std::endl;
  std::string s = message.dump();
  // std::cout << s <<std::endl;
  // std::cout << "==========Time Updated" << std::endl;
  socklen_t len;
  sendto(sockfd1 , s.c_str() , s.length(), 0 , (struct sockaddr*)&robotaddr , sizeof(robotaddr));
  // std::cout <<"Data Sent to Robot" << std::endl;
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn HunterController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

  cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
  publish_limited_velocity_ = params_.publish_limited_velocity;
  use_stamped_vel_ = params_.use_stamped_vel;

  limiter_linear_ = SpeedLimiter(
    params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
    params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.min_acceleration, params_.linear.x.max_acceleration, params_.linear.x.min_jerk,
    params_.linear.x.max_jerk);

  limiter_angular_ = SpeedLimiter(
    params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
    params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
    params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
    params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  params_.wheels_per_side = params_.left_wheel_names.size();

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(
              get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();
        });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  // Append the tf prefix if there is one
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable)
  {
    if (params_.tf_frame_prefix != "")
    {
      tf_prefix = params_.tf_frame_prefix;
    }
    else
    {
      tf_prefix = std::string(get_node()->get_namespace());
    }

    if (tf_prefix == "/")
    {
      tf_prefix = "";
    }
    else
    {
      tf_prefix = tf_prefix + "/";
    }
  }

  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id = base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HunterController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::cout<< "=============== Activate On By User ============" <<std::endl;
  is_halted = false;
  subscriber_is_active_ = true;
  int host_port  = 12347;
  int robot_port = 12345;
  int rec_port = 12346;
  std::string ip_address = "127.0.0.1";
  // int sockfd;

  hostaddr.sin_family    = AF_INET; //IPv4 
  hostaddr.sin_addr.s_addr = inet_addr(ip_address.c_str()); 
  hostaddr.sin_port = htons(host_port); 
  robotaddr.sin_family = AF_INET;
  robotaddr.sin_addr.s_addr = inet_addr(ip_address.c_str());
  robotaddr.sin_port = htons(robot_port);
  recaddr.sin_family = AF_INET;
  recaddr.sin_addr.s_addr = inet_addr(ip_address.c_str());
  recaddr.sin_port = htons(rec_port);
  bind(sockfd , (const struct sockaddr *)&recaddr , sizeof(recaddr));
  bind(sockfd1, (const struct sockaddr *)&hostaddr , sizeof(hostaddr));
  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher, and sockets are now active.");
  
  std::cout <<"==================== Plugin Activated ========================" << std::endl;
  return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::CallbackReturn HunterController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted)
  {
    halt();
    is_halted = true;
  }
  close(sockfd);
  close(sockfd1);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HunterController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HunterController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool HunterController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);


  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn HunterController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void HunterController::halt()
{

}

controller_interface::CallbackReturn HunterController::configure_side(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace hunter_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  hunter_controller::HunterController, controller_interface::ControllerInterface)