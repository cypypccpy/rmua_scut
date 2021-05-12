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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/
#include <cmath>
#include <stdio.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "gimbal_control.h"

namespace roborts_detection {

void GimbalContrl::Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k) {
  offset_.x = x;
  offset_.y = y;
  offset_.z = z;
  offset_pitch_ = pitch;
  offset_yaw_ = yaw;
  init_v_ = init_v;
  init_k_ = init_k;
  predict_init_ = false;
}

//air friction is considered
float GimbalContrl::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
  float t, y;
  t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
  y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return y;
}

//x:distance , y: height
float GimbalContrl::GetPitch(float x, float y, float v) {
  float y_temp, y_actual, dy;
  float a;
  y_temp = y;
  // by iteration
  for (int i = 0; i < 20; i++) {
    a = (float) atan2(y_temp, x);
    y_actual = BulletModel(x, v, a);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    if (fabsf(dy) < 0.0001) {
      break;
    }
    //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,y_temp,dy);
  }
  return a;
}

float GimbalContrl::GyroOptimizer(float angle_) {
    if (angle_ > 0) {
        return 160. * sqrtf(angle_) /
               (1 + powf(e, -0.3 * (angle_ - 25.)));
    }
    else {
        angle_ = -angle_;
        return -160. * sqrtf(angle_) /
               (1 + powf(e, -0.3 * (angle_ - 25.)));
    }
}

void GimbalContrl::Transform(cv::Point3f &postion, float &pitch, float &yaw) {
  pitch =
      -GetPitch((postion.z + offset_.z) / 1000, -(postion.y + offset_.y) / 1000, init_v_) + (float)(offset_pitch_ * 3.1415926535 / 180);
  //yaw positive direction :anticlockwise
  yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z)) + (float)(offset_yaw_ * 3.1415926535 / 180);
  }

void GimbalContrl::Predict(float &pitch_rate, float &yaw_rate, float &pitch, float &yaw) {
  if (predict_init_) {
  cv::Point2f gyro_speed, pic_speed, actual_speed;

  auto actual_time = std::chrono::duration<double, std::ratio<1, 1>>(std::chrono::high_resolution_clock::now() - time_begin_).count();
  time_begin_ = std::chrono::high_resolution_clock::now();

  gyro_speed.x = yaw_rate;
  gyro_speed.y = pitch_rate;
  pic_speed.x = (yaw - last_center.x) / actual_time * 0.1;
  pic_speed.y = (pitch - last_center.y) / actual_time * 0.1;
  actual_speed = pic_speed - gyro_speed * 0.02;
  std::cout << "actual_speed.x: " << actual_speed.x << std::endl;
  std::cout << "actual_speed.y:" << actual_speed.y << std::endl;
  std::cout << "actual_time:" << actual_time << std::endl;

  if (actual_speed.x < 5) {
    actual_speed.x = 0;
  }
  if (actual_speed.y < 5) {
    actual_speed.y = 0;
  }
  last_center.x = yaw;
  last_center.y = pitch;

  yaw += actual_speed.x * actual_time;
  pitch += actual_speed.x * actual_time;
  }
  if (!predict_init_) {
    predict_init_ = true;
    time_begin_ = std::chrono::high_resolution_clock::now();
    last_center.x = yaw;
    last_center.y = pitch;
  }
}

} // roborts_detection



