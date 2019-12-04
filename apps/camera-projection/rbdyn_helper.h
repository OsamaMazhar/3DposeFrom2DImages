/*      File: rbdyn_helper.h
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
#pragma once

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg.h>

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/MultiBodyConfig.h>

// opencv libraries
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/contrib/contrib.hpp>

// #include <opencv2/opencv.hpp>

// Optim Optimization Library
// #include <optim/optim.hpp>

class RbdynHelper {
public:
  RbdynHelper(const std::string& urdf_file, const int mode);
  ~RbdynHelper() = default;

  sva::PTransformd relativeTransform(const std::string & object, const std::string & relativeTo) const;

  double& getAngle(const std::string& name);
  double& getAngle(size_t index);
  const std::vector<sva::PTransformd>& getAllBodiesPose() const;
  void Update_FK(std::vector<double>);
  void Update_FK();
  void Display_Projection(std::vector<cv::Point2d>);
  void draw_skeleton(cv::Mat, cv::Mat *, std::vector<cv::Point2d>);
  // double pose_func(const std::vector<double> &vals_inp, std::vector<double> &grad, void *my_func_data);
  std::vector<cv::Point2d> reposition_skeleton(std::vector<cv::Point2d>, std::vector<cv::Point2d>);

  void optim_prob();

  cv::Mat Camera_Projection(std::vector<cv::Point3d>, int mode_of_operation);

  std::vector<std::string> body_names;
  std::vector<std::string> joint_names;
  std::vector<cv::Point3d> cv3points;
  cv::Mat pose2d; // reference (from openpose) vector

  std::vector<double> Upper_params;
  std::vector<int> Joint_Indices;
  std::vector<std::string> temp_joints_taken;
  std::vector<std::string> joints_taken;
  std::vector<cv::Point2d> imagePoints;
  std::vector<cv::Point2d> inputPoints;
  int mode_of_operation = 1;

private:
  std::vector<double> index;
  double count = 0;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;

  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Matrix3d> rotations;

  std::vector<sva::PTransformd> Body_Poses;

  cv::Mat K_Mat = cv::Mat(3, 3, cv::DataType<double>::type);
  cv::Mat D_Mat = cv::Mat(5, 1, cv::DataType<double>::type);   // Distortion vector
  cv::Mat rVec  = cv::Mat(3, 1, cv::DataType<double>::type); // Rotation vector
  cv::Mat tVec  = cv::Mat(3, 1, cv::DataType<double>::type); // Translation vector

  void init_FK(void);
  void load_camera_parameters(void);
  void getAllJointNames(void);
  void getAllBodiesName(void);
  void getJointIndex(void);
  void initialize_joints(void);
};
