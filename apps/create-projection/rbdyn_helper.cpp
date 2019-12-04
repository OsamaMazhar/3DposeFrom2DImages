/*      File: rbdyn_helper.cpp
*       This file is part of the program pose-estimation
*       Program description : TODO:inputashortdescriptionofpackagepose-estimationutilityhere
*       Copyright (C) 2018 -  osama (). All Right reserved.
*
*       This software is free software: you can redistribute it and/or modify
*       it under the terms of the CeCILL license as published by
*       the CEA CNRS INRIA, either version 2.1
*       of the License, or (at your option) any later version.
*       This software is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*       CeCILL License for more details.
*
*       You should have received a copy of the CeCILL License
*       along with this software. If not, it can be found on the official website
*       of the CeCILL licenses family (http://www.cecill.info/index.en.html).
*/
#include <iostream>
#include <functional>

#include "rbdyn_helper.h"

#include <RBDyn/Body.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Joint.h>
#include <RBDyn/Jacobian.h>

#include <rbdyn-parsers/urdf.h>

#include <nlopt.hpp>

#include <pid/rpath.h>
#include <yaml-cpp/yaml.h>
#include <chrono>


RbdynHelper::RbdynHelper(const std::string& urdf_file) {
  std::tie(mb, mbc, mbg) = rbd::RBDynFromURDF(urdf_file, rbd::ParserInput::File);
  getAllBodiesName();
  getAllJointNames();
  getJointIndex();
  init_FK();
  load_camera_parameters();
  initialize_joints();
}

void RbdynHelper::initialize_joints()
{
    auto config = YAML::LoadFile(PID_PATH("app_config/camera-projection.yaml"));
    // index = config["optim"]["upper_body_joints"].as<std::vector<double>>();
    joints_taken = config["optim"]["joints_taken"].as<std::vector<std::string>>();
    Upper_params = std::vector<double>(joints_taken.size(), 0);
    // std::cout << "[Test] Size of joints taken: " << joints_taken.size() << std::endl;
  }

namespace {
size_t _count = 0;

double pose_func(const std::vector<double> &vals_inp, std::vector<double> &grad, void *obj)
{
  RbdynHelper* this_obj = reinterpret_cast<RbdynHelper*>(obj);
  auto compute_cost =
  [this_obj](const std::vector<double> &vals_inp) -> double {
    this_obj->Update_FK(vals_inp); // Upper_params
    cv::Mat new_image_points = this_obj->Camera_Projection(this_obj->cv3points);
    _count++;

    return cv::norm(new_image_points, this_obj->pose2d, cv::NORM_L2);
  };

  if(not grad.empty()) {
    auto f0 = compute_cost(vals_inp);
    std::vector<double> x_plus_eps;
    for (size_t i = 0; i < vals_inp.size(); ++i) {
      x_plus_eps = vals_inp;
      x_plus_eps[i] += 1e-12;
      auto f1 = compute_cost(x_plus_eps);
      grad[i] = (f1 - f0) / 1e-12;
    }
  }

  return compute_cost(vals_inp);
}

}

void RbdynHelper::optim_prob(){
  // >>> Optim
  auto config = YAML::LoadFile(PID_PATH("app_config/camera-projection.yaml"));

  std::vector<double> lower_bounds_all = config["optim"]["lower_bounds"].as<std::vector<double>>();
  std::vector<double> upper_bounds_all = config["optim"]["upper_bounds"].as<std::vector<double>>();
  std::vector<double> lowerbounds, upperbounds;

  for (int i = 0; i < joints_taken.size(); i++){
    ptrdiff_t pos = find(joint_names.begin(), joint_names.end(), joints_taken[i]) - joint_names.begin();
    lowerbounds.push_back(lower_bounds_all[pos]);
    upperbounds.push_back(upper_bounds_all[pos]);
    // std::cout << "Joint Index by name: " << joints_taken[i] << " : " << pos << std::endl;
  }

  using namespace std::placeholders;
  // bool success = optim::de(x_1, std::bind(&RbdynHelper::bukin_fn, this, _1, _2, _3), nullptr, settings);

    nlopt::opt opt(nlopt::LD_SLSQP, joints_taken.size());
  	opt.set_xtol_rel(1e-6);
  	// opt.set_ftol_rel(1e-6);
  	opt.set_min_objective(pose_func, this);
    opt.set_lower_bounds(lowerbounds);
    opt.set_upper_bounds(upperbounds);

  	try{
  		double min_func;
      _count = 0;
      auto start_time = std::chrono::high_resolution_clock::now();
  		auto result = opt.optimize(Upper_params, min_func);
      auto time_elap = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> Time_Elapsed = time_elap - start_time;
      count = _count;

      std::cout << "nlopt result: " << result << std::endl;
      std::cout << "Time Elapsed in optimization: " << (int)Time_Elapsed.count() << " ms" << std::endl;
      // for (int i = 0; i < joints_taken.size(); i++)
      //   std::cout << Upper_params[i] << std::endl;
  	}
  	catch(std::exception &e) {
  		std::stringstream ss;
  		ss << "Cannot estimate the joint angles (nlopt: " << e.what() << ")";
  		throw std::runtime_error(ss.str());
  	}

  // <<< Optim
}

sva::PTransformd RbdynHelper::relativeTransform(const std::string & object, const std::string & relativeTo) const {
   // X_rel_o = X_0_o * X_rel_0
   return  mbc.bodyPosW[mb.bodyIndexByName(object)] *
           mbc.bodyPosW[mb.bodyIndexByName(relativeTo)].inv();
}

double& RbdynHelper::getAngle(const std::string& name) {
  return mbc.q[mb.jointIndexByName(name)][0];
}

double& RbdynHelper::getAngle(size_t index) {
  return mbc.q[index][0];
}

void RbdynHelper::getJointIndex() {
  for(int i = 0; i < joint_names.size(); i++)
    Joint_Indices.push_back(mb.jointIndexByName(joint_names[i]));
  // for(int i = 0; i < joint_names.size(); i++)
  //   std::cout << "Joint - " << joint_names[i] << ": " << Joint_Indices[i] << std::endl;
}

const std::vector<sva::PTransformd>& RbdynHelper::getAllBodiesPose() const {
    return mbc.bodyPosW;
}

void RbdynHelper::init_FK(void){

  for (int i = 0 ; i < body_names.size() ; i ++)
    Body_Poses.push_back(mbc.bodyPosW[mb.bodyIndexByName(body_names[i])]);
  // Body_Poses = getAllBodiesPose();
  rbd::forwardKinematics(mb, mbc);
  for (int i = 0 ; i < body_names.size() ; i ++){
    positions.push_back(Body_Poses[i].translation());
    rotations.push_back(Body_Poses[i].rotation());
    cv::Mat Temp_Mat(3,1,CV_64FC1, positions[i].data());
    cv3points.push_back(cv::Point3d(Temp_Mat));
  }
  // for (int i = 0 ; i < body_names.size() ; i ++){
  //   std::cout << "Body[" << i << "] position vector: \n" << positions[i] << std::endl;
  // }
}

void RbdynHelper::Update_FK(std::vector<double> Upper_params){
  // New temporary change
  // for (int i = 0; i < Joint_Indices.size(); i++)

  for (int i = 0; i < static_cast<int>(Upper_params.size()); i++)
    getAngle(joints_taken[i]) = Upper_params[i];
  rbd::forwardKinematics(mb, mbc);
  for (int i = 0 ; i < body_names.size() ; i ++){
    Body_Poses[i] = mbc.bodyPosW[mb.bodyIndexByName(body_names[i])];
    positions[i] = Body_Poses[i].translation();
    rotations[i] = Body_Poses[i].rotation();
    cv::Mat Temp_Mat(3,1,CV_64FC1, positions[i].data());
    cv3points[i] = cv::Point3d(Temp_Mat);
    }
  }

void RbdynHelper::Update_FK(){
  rbd::forwardKinematics(mb, mbc);
  for (int i = 0 ; i < body_names.size() ; i ++){
    Body_Poses[i] = mbc.bodyPosW[mb.bodyIndexByName(body_names[i])];
    positions[i] = Body_Poses[i].translation();
    rotations[i] = Body_Poses[i].rotation();
    cv::Mat Temp_Mat(3,1,CV_64FC1, positions[i].data());
    cv3points[i] = cv::Point3d(Temp_Mat);
    }
}

void RbdynHelper::load_camera_parameters(void){
  auto calibration_data = YAML::LoadFile(PID_PATH("app_config/Kinect.yaml"));
  auto K_Matrix_vector = calibration_data["K"]["data"].as<std::vector<double>>();
  auto D_Matrix_vector = calibration_data["D"]["data"].as<std::vector<double>>();

  memcpy(K_Mat.data, K_Matrix_vector.data(), K_Matrix_vector.size()*sizeof(double));
  memcpy(D_Mat.data, D_Matrix_vector.data(), D_Matrix_vector.size()*sizeof(double));

  rVec.at<double>(0) = 0;
  rVec.at<double>(1) = -1.570796;
  rVec.at<double>(2) = 0;

  // vertical distance of the camera (from trunk)
  tVec.at<double>(0) = 0.3477; // set at neck level (orignal value of K(1,3) was 9.5926478925672473e+02)
  // horizontal distance
  tVec.at<double>(1) = 0;
  //scale how far
  tVec.at<double>(2) = 1.4;
}

cv::Mat RbdynHelper::Camera_Projection(std::vector<cv::Point3d> objectPoints){
  cv::projectPoints(objectPoints, rVec, tVec, K_Mat, D_Mat, imagePoints);
  cv::Mat image_points_mat = cv::Mat(15,2,CV_64F,imagePoints.data());
  return image_points_mat;
}

void RbdynHelper::draw_skeleton(cv::Mat image_for_skeleton, cv::Mat * image_with_skeleton, std::vector<cv::Point2d> imagePoints)
{
  for (unsigned int i = 0; i < imagePoints.size(); ++i)
    cv::circle(image_for_skeleton, cv::Point(imagePoints[i].y, imagePoints[i].x), 5, cv::Scalar(0,0,255), -1);
  // >>> *
  unsigned int joint_indices[14] = {1, 2, 6 ,10, 4, 8, 3, 11, 5, 12, 7, 13, 9, 14};
  unsigned int count = 0;
  bool one_time = true;
  for (unsigned int i = 0; i < 10; ++i){
    if(count < 4)
      i = 0;
    if(i == 2 && one_time == true){
      i = 1;
      one_time = false;
    }
    cv::line(image_for_skeleton, cv::Point(imagePoints[i].y, imagePoints[i].x), cv::Point(imagePoints[joint_indices[count]].y,  imagePoints[joint_indices[count]].x),  cv::Scalar(0,255,0), 2, -1);
    count ++;
  }

  *image_with_skeleton = image_for_skeleton;
}

void RbdynHelper::Display_Projection(std::vector<cv::Point2d> imagePoints){
  cv::Mat Image_plane = cv::Mat::zeros( 1080, 1920, CV_8UC3 );
  cv::Mat Image_with_skeleton;
  draw_skeleton(Image_plane, &Image_with_skeleton, imagePoints);

  cv::namedWindow("Image Plane", cv::WINDOW_NORMAL);
  cv::resizeWindow("Image Plane", 600,600);
  cv::imshow("Image Plane", Image_with_skeleton);
  cv::waitKey(1);
}

void RbdynHelper::getAllBodiesName() {
  body_names.push_back("main_neck");           // 0
  body_names.push_back("main_trunk");          // 1
  body_names.push_back("main_left_shoulder");  //  2
  body_names.push_back("main_left_elbow");     //  3
  body_names.push_back("main_left_hip");       //  4
  body_names.push_back("main_left_knee");      //  5
  body_names.push_back("main_right_shoulder"); //  6
  body_names.push_back("main_right_elbow");    //  7
  body_names.push_back("main_right_hip");      //  8
  body_names.push_back("main_right_knee");     //  9
  body_names.push_back("nose");                //  10
  body_names.push_back("left_wrist_base");     //  11
  body_names.push_back("left_ankle_base");     //  12
  body_names.push_back("right_wrist_base");    //  13
  body_names.push_back("right_ankle_base");    //  14
}

// 6, 8, 9, 12, 15
// 7, 9, 10, 13, 16
void RbdynHelper::getAllJointNames() {
  joint_names.push_back("hip_base_x");       // 0
  joint_names.push_back("hip_base_y");       // 1
  joint_names.push_back("hip_base_z");       // 2
  joint_names.push_back("neck_x");           // 3
  joint_names.push_back("neck_y");           // 4
  joint_names.push_back("neck_z");           // 5   // skip it
  joint_names.push_back("left_shoulder_x");  // 6
  joint_names.push_back("left_shoulder_y");  // 7
  joint_names.push_back("left_shoulder_z");  // 8
  joint_names.push_back("left_elbow_x");     // 9
  joint_names.push_back("left_elbow_y");     // 10  // skip it
  joint_names.push_back("left_elbow_z");     // 11  // skip it
  joint_names.push_back("right_shoulder_x"); // 12
  joint_names.push_back("right_shoulder_y"); // 13
  joint_names.push_back("right_shoulder_z"); // 14
  joint_names.push_back("right_elbow_x");    // 15
  joint_names.push_back("right_elbow_y");    // 16  // skip it
  joint_names.push_back("right_elbow_z");    // 17  // skip it
  joint_names.push_back("left_hip_x");       // 18
  joint_names.push_back("left_hip_y");       // 19
  joint_names.push_back("left_hip_z");       // 21
  joint_names.push_back("right_hip_x");      // 23
  joint_names.push_back("right_hip_y");      // 20
  joint_names.push_back("right_hip_z");      // 22
  joint_names.push_back("left_knee_x");      // 24
  joint_names.push_back("left_knee_y");      // 25
  joint_names.push_back("left_knee_z");      // 27
  joint_names.push_back("right_knee_x");     // 29
  joint_names.push_back("right_knee_y");     // 26
  joint_names.push_back("right_knee_z");     // 28

//   LSX_0 = 0.785398;
// LSZ_0 = -0.785398;
// RSX_0 = -0.785398;
// REX_0 = 0.785398;
}
