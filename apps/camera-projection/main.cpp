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
#include <pid/rpath.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include <nnxx/message.h>
#include <nnxx/socket.h>
#include <nnxx/reqrep.h>
#include <nnxx/testing>
#include <nnxx/pubsub.h>

#include "rbdyn_helper.h"

#include "point2d_skeleton_generated.h"

constexpr size_t RX_MAX_PACKET_SIZE = 1000;     // 1 Kb in decimal

// enum Foo {
//   Foo1,
//   Foo2
// }
extern "C" {
    #include "extApi.h"
}

int main()
{
  int mode = 1;
  std::string input;

  do
    {
      std::cout << "Specify Estimation Mode [1: Upper body (Default), 2: Full body, 3: Lower body]: (1,2, or 3)? ";
      std::getline( std::cin, input );
      if ( !input.empty() ) {
        std::istringstream stream( input );
        stream >> mode;
      }
    } while(mode < 1 || mode > 3);

  RbdynHelper rbdyn(PID_PATH("urdf_models/human.urdf"), mode);

  auto config = YAML::LoadFile(PID_PATH("app_config/camera-projection.yaml"));
  std::string filename = config["optim"]["Pose_file"].as<std::string>();
  std::cout << "name: " << filename << std::endl;

  nnxx::socket Coordinate_rep_socket { nnxx::SP, nnxx::REP };
  const char *Coordinate_rep_addr = "ipc:///home/osama/Programs/OpenKinect/Coordinate_req_offline.ipc";
  auto Coordinate_rep_socket_id = Coordinate_rep_socket.bind(Coordinate_rep_addr);

  std::cout << "Socket ID: (" << Coordinate_rep_socket_id << ")\n";

  std::vector<std::string> considered_joints = rbdyn.joints_taken;

  auto pose = YAML::LoadFile(PID_PATH(filename));
  std::string frame_base_name = "frame"; // same as folder name
  std::string frame_template = frame_base_name + "_%d";
  char frame_number[80];

  // rbdyn.optim_prob();
  // Angles[5] = -Angles[5];

  int clientID=simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
  if (clientID!=-1)
    {
        printf("Connected to remote API server\n");

        // Now try to retrieve data in a blocking fashion (i.e. a service call):
        int objectCount;
        int* objectHandles;
        int ret=simxGetObjects(clientID,sim_handle_all,&objectCount,&objectHandles,simx_opmode_blocking);
        if (ret==simx_return_ok)
            printf("Number of objects in the scene: %d\n",objectCount);
        else
            printf("Remote API function call returned with error code: %d\n",ret);

        std::vector<int> joint_handles;
        for(int i; i < considered_joints.size(); i++)
        {
          int temp;
          int ret_code = simxGetObjectHandle(clientID, considered_joints[i].c_str(), &temp, simx_opmode_oneshot_wait);
          if(ret_code != simx_return_ok) {
        	std::cout << "can't get appro_proximity_sensor handle (error " << ret_code << ")" << std::endl;
        	return false;
        	 }
          joint_handles.push_back(temp);
        }
        simxSynchronous(clientID,1); // enable the synchronous mode (client side). The server side (i.e. V-REP) also needs to be enabled.
        simxStartSimulation(clientID,simx_opmode_oneshot); // start the simulation

        // >>> Nanomsg skeleton
        std::vector<cv::Point2d> temp_coordinate_vector, skeletal_coordinates;
        cv::Point2d temp_coordinate;
        cv::Point current_point;
        double scale = 1;
        auto buffer = new unsigned char[RX_MAX_PACKET_SIZE];
        cv::Point2f hip_base;
        for(;;)
          {

            auto co_bytes_received = Coordinate_rep_socket.recv(buffer, RX_MAX_PACKET_SIZE);
            cv::Mat blank_with_skeleton = cv::Mat::zeros(1080, 1920, CV_8UC3);
            // std::cout << "Loop Running!" << std::endl;
            if(co_bytes_received > 0)
              {
                auto coordinate_request = my_skeleton::Getskeleton2d(buffer);
                auto name = coordinate_request->name()->c_str();
                auto number_of_points = coordinate_request->number_of_points();
                for(int i = 0; i < (int)number_of_points; i++)
                {
                  auto point = coordinate_request->coordinates()->Get(i);
                  temp_coordinate.x = scale * point->x();
                  temp_coordinate.y = scale * point->y();
                  temp_coordinate_vector.push_back(temp_coordinate);
                }
            hip_base = (temp_coordinate_vector[8] + temp_coordinate_vector[11]) * .5;
            temp_coordinate_vector.push_back(hip_base);
            skeletal_coordinates = temp_coordinate_vector;
            temp_coordinate_vector.clear();
            std::cout << "Skeleton coordinate size: " << skeletal_coordinates.size() << std::endl;
            // rbdyn.inputPoints = skeletal_coordinates;
            rbdyn.Camera_Projection(rbdyn.cv3points, mode);
            // std::vector<cv::Point2d> repositionedPoints = rbdyn.reposition_skeleton(skeletal_coordinates, rbdyn.imagePoints);
            rbdyn.inputPoints = rbdyn.reposition_skeleton(skeletal_coordinates, rbdyn.imagePoints);
            std::vector<cv::Point2d> temp = rbdyn.inputPoints;
            cv::Mat points_mat = cv::Mat(15,2,CV_64F, temp.data());
            if(mode == 1)
              {
                cv::Mat selected_points = points_mat(cv::Range(0, 9), cv::Range::all()).clone();
                rbdyn.pose2d = selected_points;
              }
            else
              rbdyn.pose2d = points_mat;
            rbdyn.optim_prob();
            std::vector<double> Angles = rbdyn.Upper_params;

            std::cout << "--------------------------------------" << std::endl;
            for(int i = 0; i < Angles.size(); i++)
              std::cout << "Angle [" << considered_joints[i] << "]: " << Angles[i] << std::endl;
            std::cout << "--------------------------------------" << std::endl;
            for(int i = 0; i < joint_handles.size(); i++)
            {
              simxSetJointPosition(clientID,joint_handles[i],Angles[i],simx_opmode_oneshot); // set the joint to 90 degrees
              simxSynchronousTrigger(clientID); // trigger next simulation step. Above commands will be applied
            }
            rbdyn.Display_Projection(rbdyn.imagePoints);
            // for(int i = 0; i < skeletal_coordinates.size(); i++)
            //   {
            //     current_point = skeletal_coordinates[i];
            //     if(current_point.x == 0 && current_point.y == 0)
            //       continue;
            //     cv::circle(blank_with_skeleton, current_point, 4, cv::Scalar(0,0,255), -1);
            //   }
            // rbdyn.Display_Projection(rbdyn.imagePoints);

            // rbdyn.Display_Projection(rbdyn.inputPoints);
            // cv::namedWindow("Skeleton on Kinect 2 Size Image", cv::WINDOW_NORMAL);
            // cv::resizeWindow("Skeleton on Kinect 2 Size Image", 800, 600);
            // cv::imshow("Skeleton on Kinect 2 Size Image", blank_with_skeleton);
            // int key = cv::waitKey(1);
            // if(key > 0 && ((key & 0xFF) == 27)) // Escape Key to exit
              // break;
            }
        // <<< Nanomsg skeleton
            // sprintf(frame_number, frame_template.c_str(), i);
            // auto points = pose[frame_number].as<std::vector<double>>();
            //
            // cv::Mat points_mat = cv::Mat(15,2,CV_64F,points.data());
            // rbdyn.pose2d = points_mat;
            // cv::waitKey(0);
          }

        simxAddStatusbarMessage(clientID,"Hello V-REP!",simx_opmode_oneshot);

        // Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        int pingTime;
        simxGetPingTime(clientID,&pingTime);

        // Now close the connection to V-REP:
        simxFinish(clientID);
    }
    else
    {printf("Cannot connect to remote API server\n");}
    return(0);

}
