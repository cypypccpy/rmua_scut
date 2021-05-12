/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <unistd.h>
#include "armor_detection_node.h"

namespace roborts_detection {

ArmorDetectionNode::ArmorDetectionNode():
    node_state_(roborts_common::IDLE),
    demensions_(3),
    initialized_(false),
    detected_enemy_(false),
    undetected_count_(0),
    as_(nh_, "armor_detection_node_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false) {
  initialized_ = false;
  enemy_nh_ = ros::NodeHandle();
  if (Init().IsOK()) {
    initialized_ = true;
    node_state_ = roborts_common::IDLE;
  } else {
    ROS_ERROR("armor_detection_node initalized failed!");
    node_state_ = roborts_common::FAILURE;
  }
  as_.start();
}

ErrorInfo ArmorDetectionNode::Init() {
  enemy_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
  gimbal_info_sub_ = enemy_nh_.subscribe<roborts_msgs::GimbalInfo>("cmd_gimbal_info", 100, &ArmorDetectionNode::GimbalInfoCallback, this);
  shoot_client_=enemy_nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
  fric_client_=enemy_nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");

  ArmorDetectionAlgorithms armor_detection_param;

  std::string file_name = ros::package::getPath("roborts_detection") + "/armor_detection/config/armor_detection.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
  if (!read_state) {
    ROS_ERROR("Cannot open %s", file_name.c_str());
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }
  gimbal_control_.Init(armor_detection_param.camera_gimbal_transform().offset_x(),
                       armor_detection_param.camera_gimbal_transform().offset_y(),
                       armor_detection_param.camera_gimbal_transform().offset_z(),
                       armor_detection_param.camera_gimbal_transform().offset_pitch(),
                       armor_detection_param.camera_gimbal_transform().offset_yaw(), 
                       armor_detection_param.projectile_model_info().init_v(),
                       armor_detection_param.projectile_model_info().init_k()
                       );
  
  //create the selected algorithms
  std::string selected_algorithm = armor_detection_param.selected_algorithm();
  // create image receiver
  cv_toolbox_ =std::make_shared<CVToolbox>(armor_detection_param.camera_name());
  // create armor detection algorithm
  armor_detector_ = roborts_common::AlgorithmFactory<ArmorDetectionBase,std::shared_ptr<CVToolbox>>::CreateAlgorithm
      (selected_algorithm, cv_toolbox_);

  undetected_armor_delay_ = armor_detection_param.undetected_armor_delay();
  if (armor_detector_ == nullptr) {
    ROS_ERROR("Create armor_detector_ pointer failed!");
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  } else
    return ErrorInfo(ErrorCode::OK);
}

void ArmorDetectionNode::ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data) {
  roborts_msgs::ArmorDetectionFeedback feedback;
  roborts_msgs::ArmorDetectionResult result;
  bool undetected_msg_published = false;

  if(!initialized_){
    feedback.error_code = error_info_.error_code();
    feedback.error_msg  = error_info_.error_msg();
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_INFO("Initialization Failed, Failed to execute action!");
    return;
  }

  switch (data->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  }
  
  ros::Rate rate(25);
  while(ros::ok()) {

    if(as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }

    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (undetected_count_ != 0) {
        feedback.detected = true;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_pos.header.frame_id = "camera0";
        feedback.enemy_pos.header.stamp    = ros::Time::now();

        feedback.enemy_pos.pose.position.x = x_;
        feedback.enemy_pos.pose.position.y = y_;
        feedback.enemy_pos.pose.position.z = z_;
        feedback.enemy_pos.pose.orientation.w = 1;
        as_.publishFeedback(feedback);
        undetected_msg_published = false;
      } else if(!undetected_msg_published) {
        feedback.detected = false;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_pos.header.frame_id = "camera0";
        feedback.enemy_pos.header.stamp    = ros::Time::now();

        feedback.enemy_pos.pose.position.x = 0;
        feedback.enemy_pos.pose.position.y = 0;
        feedback.enemy_pos.pose.position.z = 0;
        feedback.enemy_pos.pose.orientation.w = 1;
        as_.publishFeedback(feedback);
        undetected_msg_published = true;
      }
    }
    rate.sleep();
  }
}

void ArmorDetectionNode::GimbalInfoCallback(const roborts_msgs::GimbalInfo::ConstPtr& data_) {
  std::lock_guard<std::mutex> guard(mutex_);
  gimbal_info_ = *data_;
}

void ArmorDetectionNode::ExecuteLoop() {
  undetected_count_ = undetected_armor_delay_;
  int mode = 0;
  int delay = 0;
  float last_position_ = 0;
  fric_wheel_.request.open=0;
  
  while(running_) {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) {
      cv::Point3f target_3d;
      ErrorInfo error_info = armor_detector_->DetectArmor(detected_enemy_, target_3d);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        x_ = target_3d.x;
        y_ = target_3d.y;
        z_ = target_3d.z;
        error_info_ = error_info;
      }
      //std::cout << "gimbal_info_.pitch_rate: " << gimbal_info_.pitch_rate << std::endl;

      if(detected_enemy_) {
        float pitch, yaw;
        float pitch_rate = gimbal_info_.pitch_rate;
        float yaw_rate = gimbal_info_.yaw_rate;
        gimbal_control_.Transform(target_3d, pitch, yaw);
        gimbal_control_.Predict(pitch_rate, yaw_rate, pitch, yaw);
        
        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        gimbal_angle_.yaw_angle = yaw * 0.35;
        gimbal_angle_.pitch_angle = pitch;

        //send shoot_info
        shoot_cmd_.request.mode=1;
        shoot_cmd_.request.number=1;
        shoot_cmd_.request.shoot_freq=10;
        
        fric_wheel_.request.open=1;
        //fric_client_.call(fric_wheel_);
        //shoot_client_.call(shoot_cmd_);

        std::lock_guard<std::mutex> guard(mutex_);
        undetected_count_ = undetected_armor_delay_;
        last_position_ = gimbal_angle_.pitch_angle;

        PublishMsgs();
      } else if(undetected_count_ != 0) {

        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        gimbal_angle_.yaw_angle = 0;
        gimbal_angle_.pitch_angle = last_position_;

        undetected_count_--;
        PublishMsgs();
        /*
        if(undetected_count_ == 0) {
          fric_wheel_.request.open=0;
          fric_client_.call(fric_wheel_);
        }
        */
      } else if(!detected_enemy_) {
        float pitch, yaw;
        gimbal_control_.Transform(target_3d, pitch, yaw);

        gimbal_angle_.yaw_mode = true;
        gimbal_angle_.pitch_mode = false;
        if(mode == 0) {
          gimbal_angle_.yaw_angle = 0.12;
        } else {
          gimbal_angle_.yaw_angle = -0.12;
        }
        gimbal_angle_.pitch_angle = pitch;
        delay += 1;

        std::lock_guard<std::mutex> guard(mutex_);
        if(delay == 100) {
          mode = mode == 0 ? 1 : 0;
          delay = 0;
        }     
        PublishMsgs(); 
      }
    } else if (node_state_ == NodeState::PAUSE) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
}

void ArmorDetectionNode::PublishMsgs() {
  enemy_info_pub_.publish(gimbal_angle_);
}

void ArmorDetectionNode::StartThread() {
  //ROS_INFO("Armor detection node started!");
  running_ = true;
  armor_detector_->SetThreadState(true);
  if(node_state_ == NodeState::IDLE) {
    armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ArmorDetectionNode::PauseThread() {
  ROS_INFO("Armor detection thread paused!");
  node_state_ = NodeState::PAUSE;
}

void ArmorDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  armor_detector_->SetThreadState(false);
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

ArmorDetectionNode::~ArmorDetectionNode() {
  StopThread();
}
} //namespace roborts_detection

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "armor_detection_node", ros::init_options::NoSigintHandler);
  roborts_detection::ArmorDetectionNode armor_detection;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  armor_detection.StopThread();
  return 0;
}


