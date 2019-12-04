/*      File: main.cpp
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
#include "rbdyn_helper.h"

// RBDyn-urdf
#include <rbdyn-parsers/urdf.h>

#include <pid/rpath.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace Eigen;
using namespace sva;
using namespace rbd;


int main()
{
  RbdynHelper projection(PID_PATH("urdf_models/human.urdf"));
  auto config = YAML::LoadFile(PID_PATH("app_config/camera-projection.yaml"));

  std::string pid_path = PID_PATH("pose_files");
  std::string frame_base_name = "frame"; // same as folder name
  std::string frame_template = frame_base_name + "_%d";
  char frame_number[80];
  cv::FileStorage fs_read(pid_path + "/Pose_sequence.yaml", cv::FileStorage::WRITE);

  std::vector<std::vector<cv::Point2d>> array_imagePoints;

  cv::Mat Image_plane = cv::Mat::zeros( 1080, 1920, CV_8UC3 );
  cv::Mat Image_with_skeleton;

  cv::namedWindow("Image Plane", cv::WINDOW_NORMAL);
  cv::resizeWindow("Image Plane", 600,600);
  float pi = 3.14159265359;

    // >>> Writing the Yaml file
  for (int i = 0; i < 10; i++)
    {
      sprintf(frame_number, frame_template.c_str(), i);

      projection.getAngle("neck_x") =  0 + i*(0.1);

      // projection.getAngle("left_shoulder_x") =  0 + i*(0.2);
      // projection.getAngle("left_shoulder_y") =   0 + i*(0.2); // tick //reversed it axes and rotated
      projection.getAngle("left_shoulder_z") =  -(0 + i*(0.2)); // tick
      // projection.getAngle("left_elbow_x") =   1.5708; // tick

      // projection.getAngle("right_shoulder_x") = 0.785398 + i*(0.2);
      // projection.getAngle("right_shoulder_y") = (0 + i*(0.2));
      projection.getAngle("right_shoulder_z") = (0 + i*(0.2));
      // projection.getAngle("right_elbow_x") =    1.5708;
      // projection.getAngle("hip_base_z") =    (0 + i*(0.2));
      projection.getAngle("hip_base_x") =    (0 + i*(0.2));
      projection.getAngle("hip_base_y") =    (0 + i*(0.2));
      //
      // projection.getAngle("left_hip_x") =    0 + i*(0.2);
      // projection.getAngle("left_hip_y") =    0 + i*(0.1);
      // projection.getAngle("left_hip_z") =    0 + i*(0.1);
      // projection.getAngle("left_knee_y") =   0 + i*(0.1);
      // //
      // // projection.getAngle("right_hip_x") =    0 + i*(0.2);
      // projection.getAngle("right_hip_y") =    0 + i*(0.1);
      // projection.getAngle("right_hip_z") =    0 + i*(0.1);
      // projection.getAngle("right_knee_y") =   0 + i*(0.1);
      // projection.getAngle("right_knee_y") =    1.17939;

      std::cout << "--------------------------------------" << std::endl;
      std::cout << "Lower Angles: " << i*(0.1) * 180 / pi << std::endl;
      std::cout << "Upper Angles: " << (0.5+i*(0.1)) * 180 / pi<< std::endl;
      std::cout << "--------------------------------------" << std::endl;

      projection.Update_FK();


      float rotx = 0;
      float roty = 0 * pi / 180;;
      float rotz = 90 * pi / 180;

      float cx = cosf(rotx), sx = sinf(rotx);
      float cy = cosf(roty), sy = sinf(roty);
      float cz = cosf(rotz), sz = sinf(rotz);

      cv::Mat Rot = cv::Mat(3, 3, CV_64F);

      Rot.at<double>(0) = cz * cy;
      Rot.at<double>(1) = cz * sy * sx - sz * cx;
      Rot.at<double>(2) = cz * sy * cx + sz * sx;
      Rot.at<double>(3) = sz * cy;
      Rot.at<double>(4) = sz * sy * sx + cz * cx;
      Rot.at<double>(5) = sz * sy * cx - cz * sx;
      Rot.at<double>(6) = -sy;
      Rot.at<double>(7) = cy * sx;
      Rot.at<double>(8) = cy * cx;

      std::cout << "Rotation Matrix is: " << Rot << std::endl;

      cv::transform(projection.cv3points, projection.cv3points, Rot);

      projection.Camera_Projection(projection.cv3points);

      // array_imagePoints.push_back(projection.imagePoints);

      fs_read << frame_number << projection.imagePoints;

      projection.draw_skeleton(Image_plane, &Image_with_skeleton, projection.imagePoints);
      cv::imshow("Image Plane", Image_with_skeleton);
      cv::waitKey(0);
    }
  fs_read.release();

  // <<< Writing the Yaml file


  std::cout << "Done execution of the program!!!" << std::endl;
return(0);
}
