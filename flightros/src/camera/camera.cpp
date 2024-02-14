/* Getting the first data set */

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <stdio.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

#include "flightlib/bridges/py_bridge.hpp"

#define PI 3.1415926535897932384626433832795028841971

using namespace flightlib;

/*
Would be nicer to have an iterative loop that creates these instances
  instantiation before while loop
  separate loop in while loop

Most of the cameras and references are just 1D, there are always 6 cameras coupled with one vehicle
*/

int main(int argc, char *argv[]) {
  std::cout << "Starting up program" << std::endl;
  // declaring the functions (this should be moved to a separate header ideally)
  void SetupCameraIntrinsics(int pixelResolution, Vector<3> translation, Matrix<3, 3> rotMatrix, std::shared_ptr<Quadrotor> quadPointer, std::shared_ptr<RGBCamera> cameraPointer);
  Matrix<3,3> rotMatrixXrounded(float angle);
  Matrix<3,3> rotMatrixYrounded(float angle);
  Matrix<3,3> rotMatrixZrounded(float angle);

  std::cout << "Defining some variables" << std::endl;
  // number of agents
  int numberOfAgents = 1;
  // number of cameras that need to be setup per agent
  int numberOfCameras = 6;
  // total number of cameras
  int totalNumberOfCameras = numberOfAgents*numberOfCameras;
  // pixel resolution of individual perspective cameras
  int pixelResolution = 1024;
  // references to the rotation matrices {front, back, left, right, top, bottom}
  Matrix<3,3> rotMatrices[numberOfCameras] = {rotMatrixXrounded(0),rotMatrixZrounded(PI),rotMatrixZrounded(PI/2),rotMatrixZrounded(-PI/2),rotMatrixXrounded(PI/2),rotMatrixXrounded(-PI/2)};

  std::cout << "Initialize ROS nodes" << std::endl;
  // initialize ROS
  ros::init(argc, argv, "camera_example");
  ros::start();

  std::cout << "Initialize quadcopters" << std::endl;
  // unity quadrotor
  std::shared_ptr<Quadrotor> quads[numberOfAgents];

  //std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
  // define quadsize scale (for unity visualization only)
  std::cout << "Setting the size of the quadcopters" << std::endl;
  Vector<3> quad_size(0.5, 0.5, 0.5);
  std::cout << "Attaching the size to the quadcopters" << std::endl;
  for (int i = 0; i < numberOfAgents; i++) {
    quads[i] = std::make_shared<Quadrotor>();
    quads[i]->setSize(quad_size);
  }

  std::cout << "Initialize quad states" << std::endl;
  QuadState quad_states[numberOfAgents];

  // camera pointers
  // references to camera pointers
  std::cout << "Initialize camera pointers" << std::endl;
  std::shared_ptr<RGBCamera> cameras[totalNumberOfCameras];

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  SceneID scene_id{UnityScene::OWN};
  bool unity_ready{false};

  std::shared_ptr<PyBridge> py_bridge_ptr = PyBridge::getInstance();

  // Flightmare
  // relative position translation of camera
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  
  std::cout << "Getting the quads set up" << std::endl;
  // setting up the agents
  for (int i = 0; i < numberOfAgents; i++) {
    // defining the camera intrinsics per camera for 1 agent
    for (int j = 0; j < numberOfCameras; j++) {
      cameras[j+numberOfCameras*i] = std::make_shared<RGBCamera>();
      cameras[j+numberOfCameras*i]->setFOV(90);
      cameras[j+numberOfCameras*i]->setWidth(pixelResolution);
      cameras[j+numberOfCameras*i]->setHeight(pixelResolution);
      cameras[j+numberOfCameras*i]->setRelPose(B_r_BC, rotMatrices[j]);
      cameras[j+numberOfCameras*i]->setPostProcesscing(
        std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
      quads[i]->addRGBCamera(cameras[j+numberOfCameras*i]);
    }
  }

  std::cout << "Making the quad grid" << std::endl;
  // initialization
  // initial offsets in order to start at left corner of environment (first quad on first row)
  float offsetX = 538.0;//-150;   // assuming this is in meters
  float offsetY = 573.0;//60;
  float quadRowOffset = 0;
  float gridDist = 1;
  //float gridDist = atof(argv[1]);
  int quadsPerDim = 10;
  // loading the tree positions through matrices in text files
  cv::Mat terrHeight;
  cv::Mat treePos;
  std::string pkg_path = ros::package::getPath("flightros"); 
  //cv::FileStorage fs_terrHeight(pkg_path + "/src/camera/required_mats/heightmap.xml",cv::FileStorage::READ);
  //cv::FileStorage fs_treePos(pkg_path + "/src/camera/required_mats/treepositions.xml",cv::FileStorage::READ);
  //fs_terrHeight["data"] >> terrHeight;
  //fs_treePos["data"] >> treePos;
  //std::cout << terrHeight.rows << ", " << terrHeight.cols << std::endl;
  // positioning quadcopters starting from the first row on the left and working backward 
  for (int i = 0; i < numberOfAgents; i++) {
    quad_states[i].setZero();
    quad_states[i].x[QS::POSX] += gridDist*i + offsetX;
    quad_states[i].x[QS::POSY] += offsetY + quadRowOffset;
    quad_states[i].x[QS::POSZ] = 317.0;
    quads[i]->reset(quad_states[i]);
    unity_bridge_ptr->addQuadrotor(quads[i]);

    if ((i+1)%quadsPerDim == 0) {
      quadRowOffset += gridDist;
      offsetX -= gridDist*quadsPerDim;
    }
  }

  // connect unity
  unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  std::cout << "Setting up omni-directional picture construction" << std::endl;
  // loading the required matrices through xml files
  cv::Mat img_index;
  cv::Mat p_x_RGB;
  cv::Mat p_y_RGB;
  //std::string pkg_path = ros::package::getPath("flightros");
  cv::FileStorage fs_img_index(pkg_path + "/src/camera/required_mats/img_index.xml",cv::FileStorage::READ);
  cv::FileStorage fs_p_x_RGB(pkg_path + "/src/camera/required_mats/p_x_RGB.xml",cv::FileStorage::READ);
  cv::FileStorage fs_p_y_RGB(pkg_path + "/src/camera/required_mats/p_y_RGB.xml",cv::FileStorage::READ);
  fs_img_index["data"] >> img_index;
  fs_p_x_RGB["data"] >> p_x_RGB;
  fs_p_y_RGB["data"] >> p_y_RGB;
  cv::transpose(img_index,img_index);
  cv::transpose(p_x_RGB,p_x_RGB);
  cv::transpose(p_y_RGB,p_y_RGB);

  

  // Parameters for spiral
  float theta;
  float spiralParam1 = 0;       // extra offset for spiral radius
  float spiralParam2 = 0.25;       // scaling of theta in radians for spiral radius
  int pointsPerRotation = 25;    // number of points to draw for each full rotation (fidelity)
  int numRotations = 4;           // number of total turns

  // Constructing a list to hold the different values of theta for the spiral
  float theta_lst[pointsPerRotation*numRotations];
  int theta_idx = 0;
  for (theta=0;theta<numRotations*(2*PI);theta+=(2*PI)/pointsPerRotation) {
    theta_lst[theta_idx] = theta;
    theta_idx++;
  }

  float xpos_lst[pointsPerRotation*numRotations];   // *(int)16
  float ypos_lst[pointsPerRotation*numRotations];
  for (int pos = 0; pos < pointsPerRotation*numRotations; pos++) {
    float spiral_radius = spiralParam1 + spiralParam2*theta_lst[pos];
    xpos_lst[pos] = spiral_radius*cos(theta_lst[pos]);
    ypos_lst[pos] = spiral_radius*sin(theta_lst[pos]); 
  }

  //for (int i = 0; i < pointsPerRotation*numRotations; i++) {
    //xpos_lst[i] = gridDist*i + offsetX - 5.0;
    //ypos_lst[i] = offsetY + quadRowOffset - 5.0;

    //if ((i+1)%quadsPerDim == 0) {
      //quadRowOffset += gridDist;
      //offsetX -= gridDist*quadsPerDim;
    //}
  //}

  //int counter = 0;
  //for (float y = -5.0; y < 5.0; y++) {
    //for (float x = -5.0; x < 5.0; x++) {
      //xpos_lst[counter] = x;
      //ypos_lst[counter] = y;
      //counter++;
    //}
  //}
  
  std::cout << "Rendering cubemap images" << std::endl;
  FrameID frame_id = 0;
  int num_frames = 202;
  int frame_id_delay = 100;
  int msg_delay = 0;    // discripancy between requested and received frame_id
  
  while (unity_ready && (int)frame_id < num_frames) {
    std::cout << "====================================================" << std::endl;
    std::cout << "frame_id: " + std::to_string(frame_id) << std::endl;
    std::cout << "Rendering environment" << std::endl;
    unity_bridge_ptr->getRender(frame_id);
    std::cout << "Handling environment output" << std::endl;
    unity_bridge_ptr->handleOutputDelayed(frame_id);   // incoming frame_id has delay on sent frame_id

    // have some delay based on frame_id such that the renderer can be ready for position updates
    if (((int)frame_id) >= frame_id_delay) {
      std::cout << "Iterating over cameras" << std::endl;
      cv::Mat imgs[totalNumberOfCameras];

      for (int i = 0; i < numberOfAgents; i++) {
        // publish the camera image and save to folder for 1 agent
        for (int j = 0; j < numberOfCameras; j++) {
          cameras[j+numberOfCameras*i]->getRGBImage(imgs[j+numberOfCameras*i]);
          // Save image
          std::cout << "Saving picture " + std::to_string(j+numberOfCameras*i+numberOfCameras*((int)frame_id-frame_id_delay)) << std::endl;
          std::cout << (int)imgs[j+numberOfCameras*i].at<uint8_t>(0,0,0) << std::endl;
          cv::imwrite("img" + std::to_string(j+numberOfCameras*i+numberOfCameras*((int)frame_id-frame_id_delay)) + ".jpg", imgs[j+numberOfCameras*i]);
          // converting to grayscale in order to use properly in omni-directional projection
          cv::cvtColor(imgs[j+numberOfCameras*i],imgs[j+numberOfCameras*i],cv::COLOR_BGR2GRAY);
        }
      }
    
      std::cout << "Constructing omni-directional images" << std::endl;
      float final_w = 1024;
      float final_h = 1024;
      int data_size = final_w*final_h;
      std::cout << "Quad Position: {" + std::to_string(quad_states[0].x[QS::POSX]) + "}, {" + std::to_string(quad_states[0].x[QS::POSY]) + "}" << std::endl;

      // constructing the final image column-by-column
      std::vector<float> final_RGB_data(data_size);
      const int *img_index_ptr = img_index.ptr<int>();
      double *p_x_RGB_ptr = (double*)p_x_RGB.data;
      double *p_y_RGB_ptr = (double*)p_y_RGB.data;
      for (int i = 0; i < numberOfAgents; i++) {
        for (int j = 0; j < data_size; j++) {
          cv::Mat curr_img = imgs[*(img_index_ptr+j)+numberOfCameras*i];
          uint8_t *pixel_ptr = (uint8_t*)curr_img.data;
          int x_px_coord = (int)*(p_x_RGB_ptr+j);
          int y_px_coord = (int)*(p_y_RGB_ptr+j);
          final_RGB_data[j] = (int)*(pixel_ptr+x_px_coord+y_px_coord*(int)final_w);
        }
        cv::Mat final_RGB = cv::Mat(final_w,final_h,cv::DataType<float>::type,final_RGB_data.data());;
        //cv::Mat final_RGB_flipped = cv::Mat(final_w,final_h,cv::DataType<float>::type);
        //cv::flip(final_RGB,final_RGB_flipped,1);    // mirror along vertical axis
        // Save omni-directional image, the image rows are from right to left for some reason (map row from left to right)
        //std::cout << "Saving omni-directional picture " + std::to_string(9-i%10+((int)i/10)*10) << std::endl;
        //cv::imwrite(pkg_path + "/src/camera/images/omni-dir-img-" + std::to_string(9-i%10+((int)i/10)*10) + ".jpg",final_RGB);
        // making sure that the delay between requested and received frame_id is handled such that the correct movement is captured in output pictures
        if (msg_delay == 2) {
          std::cout << "Saving omni-directional picture " + std::to_string((int)frame_id-frame_id_delay-msg_delay) << std::endl;
          cv::imwrite(pkg_path + "/src/camera/images/omni-dir-img-" + std::to_string((int)frame_id-frame_id_delay-msg_delay) + ".jpg",final_RGB);
          //cv::imwrite("/media/mocca/data/omni/omni-dir-img-" + std::to_string((int)frame_id-frame_id_delay-msg_delay) + ".jpg",final_RGB);
        } else {
          msg_delay++;
        }      
      }

      // update quad position
      // spiral needs to be rotated to have correct orientation to North
      quad_states[0].x[QS::POSX] = -ypos_lst[(int)frame_id-frame_id_delay] + offsetX;
      quad_states[0].x[QS::POSY] = xpos_lst[(int)frame_id-frame_id_delay] + offsetY;
      quads[0]->setState(quad_states[0]);
    }
    frame_id++;
  }

  std::cout << "All training pictures are saved!" << std::endl;
  
  std::cout << "====================================================" << std::endl;
  
  std::cout << "Sending message to bearingnet to start training." << std::endl;
  py_bridge_ptr->getRender(frame_id);
  
  std::cout << "While bearingnet is training allow user to move quad with keyboard: \'wasd\' for translate, \'qe\' for rotate" << std::endl;
  bool outbound_to_inbound = false;   // when an incoming message from Unity indicates that the outbound trip has been completed

  while (unity_ready && !outbound_to_inbound) {
    std::cout << "====================================================" << std::endl;
    // send message to Unity that outbound journey is active
    unity_bridge_ptr->getOutbound();
    // get message and check whether the outbound journey is still active
    outbound_to_inbound = unity_bridge_ptr->handleOutboundOutput();
  }

  std::cout << "Outbound journey switched to inbound journey" << std::endl;
  bool inbound_to_eval = false;   // when message from Unity indicates that inbound journey is complete and evaluation begins

  while (unity_ready && outbound_to_inbound && !inbound_to_eval) {
    std::cout << "====================================================" << std::endl;
    // send message to Unity that inbound journey is active
    unity_bridge_ptr->getInbound();
    // get message with quad position and orientation and check whether the outbound journey is still active
    inbound_to_eval = unity_bridge_ptr->handleInboundOutput();
  }

  std::cout << "Inbound journey switched to evaluation phase" << std::endl;
  bool make_new_eval_available = true;    // when python is ready processing the image
  bool evaluation_complete = false;   // keeping track of whether the new home vector has been made available in flightmare
  //bool new_eval_incoming = false;   // checks whether the correct message has been received from Unity
  bool sim_finish = false;   // when message from Unity indicates that evaluation is complete
  msg_delay = 0;    // reset

  // make sure that the last images are cleared out of the queue
  std::cout << "Iterating over cameras" << std::endl;
  cv::Mat imgs[totalNumberOfCameras];
  for (int i = 0; i < numberOfAgents; i++) {
    // publish the camera image and save to folder for 1 agent
    for (int j = 0; j < numberOfCameras; j++) {
      cameras[j+numberOfCameras*i]->getRGBImage(imgs[j+numberOfCameras*i]);
      // Save image
      std::cout << "Saving picture " + std::to_string(j+numberOfCameras*i+numberOfCameras*((int)frame_id-frame_id_delay)) << std::endl;
      std::cout << (int)imgs[j+numberOfCameras*i].at<uint8_t>(0,0,0) << std::endl;
      cv::imwrite("img" + std::to_string(j+numberOfCameras*i+numberOfCameras*((int)frame_id-frame_id_delay)) + ".jpg", imgs[j+numberOfCameras*i]);
      // converting to grayscale in order to use properly in omni-directional projection
      cv::cvtColor(imgs[j+numberOfCameras*i],imgs[j+numberOfCameras*i],cv::COLOR_BGR2GRAY);
    }
  }

  std::cout << "Constructing omni-directional images" << std::endl;
  float final_w = 1024;
  float final_h = 1024;
  int data_size = final_w*final_h;
  std::cout << "Quad Position: {" + std::to_string(quad_states[0].x[QS::POSX]) + "}, {" + std::to_string(quad_states[0].x[QS::POSY]) + "}" << std::endl;
  std::cout << "Quad Rotation: {" + std::to_string(quad_states[0].x[QS::ATTW]) + "}, {" + std::to_string(quad_states[0].x[QS::ATTX]) + "}, {" + std::to_string(quad_states[0].x[QS::ATTY]) + "}, {" + std::to_string(quad_states[0].x[QS::ATTZ]) + "}" << std::endl;

  // constructing the final image column-by-column
  std::vector<float> final_RGB_data(data_size);
  const int *img_index_ptr = img_index.ptr<int>();
  double *p_x_RGB_ptr = (double*)p_x_RGB.data;
  double *p_y_RGB_ptr = (double*)p_y_RGB.data;
  for (int i = 0; i < numberOfAgents; i++) {
    for (int j = 0; j < data_size; j++) {
      cv::Mat curr_img = imgs[*(img_index_ptr+j)+numberOfCameras*i];
      uint8_t *pixel_ptr = (uint8_t*)curr_img.data;
      int x_px_coord = (int)*(p_x_RGB_ptr+j);
      int y_px_coord = (int)*(p_y_RGB_ptr+j);
      final_RGB_data[j] = (int)*(pixel_ptr+x_px_coord+y_px_coord*(int)final_w);
    }
    cv::Mat final_RGB = cv::Mat(final_w,final_h,cv::DataType<float>::type,final_RGB_data.data());;
    //cv::Mat final_RGB_flipped = cv::Mat(final_w,final_h,cv::DataType<float>::type);
    //cv::flip(final_RGB,final_RGB_flipped,1);    // mirror along vertical axis
    // Save omni-directional image, the image rows are from right to left for some reason (map row from left to right)
    //std::cout << "Saving omni-directional picture " + std::to_string(9-i%10+((int)i/10)*10) << std::endl;
    //cv::imwrite(pkg_path + "/src/camera/images/omni-dir-img-" + std::to_string(9-i%10+((int)i/10)*10) + ".jpg",final_RGB);      
    if (msg_delay == 0) {
      std::cout << "Saving omni-directional evaluation picture" << std::endl;
      cv::imwrite(pkg_path + "/src/camera/eval_images/omni-dir-eval-img-overflow.jpg",final_RGB);      
      // send zmq message to python indicating that new image is available
      std::cout << "Sending message to bearingnet to evaluate available image." << std::endl;
    } else {
      msg_delay++;
    }
  }
  
  while (unity_ready && inbound_to_eval && !sim_finish) {
    std::cout << "====================================================" << std::endl;
    std::cout << "frame_id: " + std::to_string(frame_id) << std::endl;
    // send message to unity that scene needs to be sampled for omni-direction image construction
    unity_bridge_ptr->getEvaluation();
    // check whether the evaluation phase is still active 
    sim_finish = unity_bridge_ptr->handleEvaluationOutput();
    // handle the images that are coming in and push them on the queue
    //new_eval_incoming = unity_bridge_ptr->handleOutput();

    //std::cout << unity_bridge_ptr->getQuadPosFromUnity() << std::endl;
    //std::cout << unity_bridge_ptr->getQuadRotFromUnity() << std::endl;

    // sync the flightmare quad state to the Unity quad state
    Vector<3> quad_pos_unity = unity_bridge_ptr->getQuadPosFromUnity();
    Vector<4> quad_rot_unity = unity_bridge_ptr->getQuadRotFromUnity();
    std::cout << quad_pos_unity << std::endl;
    quad_states[0].x[QS::POSX] = quad_pos_unity[0];
    quad_states[0].x[QS::POSY] = quad_pos_unity[2];   // righthand to lefthand or vice versa
    quad_states[0].x[QS::POSZ] = quad_pos_unity[1];
    quad_states[0].x[QS::ATTW] = quad_rot_unity[3];
    quad_states[0].x[QS::ATTX] = -quad_rot_unity[0];
    quad_states[0].x[QS::ATTY] = -quad_rot_unity[2];
    quad_states[0].x[QS::ATTZ] = -quad_rot_unity[1];
    quads[0]->setState(quad_states[0]);

    // only save the image and inform python whenever python is ready with processing previous image
    if (make_new_eval_available) {
      unity_bridge_ptr->getRender(frame_id); 
      bool synched = unity_bridge_ptr->handleOutputDelayed(frame_id);

      if (synched) {
        std::cout << "Iterating over cameras" << std::endl;
        cv::Mat imgs[totalNumberOfCameras];

        for (int i = 0; i < numberOfAgents; i++) {
          // publish the camera image and save to folder for 1 agent
          for (int j = 0; j < numberOfCameras; j++) {
            cameras[j+numberOfCameras*i]->getRGBImage(imgs[j+numberOfCameras*i]);
            // Save image
            std::cout << "Saving picture " + std::to_string(j+numberOfCameras*i+numberOfCameras*((int)frame_id-frame_id_delay)) << std::endl;
            std::cout << (int)imgs[j+numberOfCameras*i].at<uint8_t>(0,0,0) << std::endl;
            cv::imwrite("img" + std::to_string(j+numberOfCameras*i+numberOfCameras*((int)frame_id-frame_id_delay)) + ".jpg", imgs[j+numberOfCameras*i]);
            // converting to grayscale in order to use properly in omni-directional projection
            cv::cvtColor(imgs[j+numberOfCameras*i],imgs[j+numberOfCameras*i],cv::COLOR_BGR2GRAY);
          }
        }
      
        std::cout << "Constructing omni-directional images" << std::endl;
        float final_w = 1024;
        float final_h = 1024;
        int data_size = final_w*final_h;
        std::cout << "Quad Position: {" + std::to_string(quad_states[0].x[QS::POSX]) + "}, {" + std::to_string(quad_states[0].x[QS::POSY]) + "}" << std::endl;
        std::cout << "Quad Rotation: {" + std::to_string(quad_states[0].x[QS::ATTW]) + "}, {" + std::to_string(quad_states[0].x[QS::ATTX]) + "}, {" + std::to_string(quad_states[0].x[QS::ATTY]) + "}, {" + std::to_string(quad_states[0].x[QS::ATTZ]) + "}" << std::endl;

        // constructing the final image column-by-column
        std::vector<float> final_RGB_data(data_size);
        const int *img_index_ptr = img_index.ptr<int>();
        double *p_x_RGB_ptr = (double*)p_x_RGB.data;
        double *p_y_RGB_ptr = (double*)p_y_RGB.data;
        for (int i = 0; i < numberOfAgents; i++) {
          for (int j = 0; j < data_size; j++) {
            cv::Mat curr_img = imgs[*(img_index_ptr+j)+numberOfCameras*i];
            uint8_t *pixel_ptr = (uint8_t*)curr_img.data;
            int x_px_coord = (int)*(p_x_RGB_ptr+j);
            int y_px_coord = (int)*(p_y_RGB_ptr+j);
            final_RGB_data[j] = (int)*(pixel_ptr+x_px_coord+y_px_coord*(int)final_w);
          }
          cv::Mat final_RGB = cv::Mat(final_w,final_h,cv::DataType<float>::type,final_RGB_data.data());;
          //cv::Mat final_RGB_flipped = cv::Mat(final_w,final_h,cv::DataType<float>::type);
          //cv::flip(final_RGB,final_RGB_flipped,1);    // mirror along vertical axis
          // Save omni-directional image, the image rows are from right to left for some reason (map row from left to right)
          //std::cout << "Saving omni-directional picture " + std::to_string(9-i%10+((int)i/10)*10) << std::endl;
          //cv::imwrite(pkg_path + "/src/camera/images/omni-dir-img-" + std::to_string(9-i%10+((int)i/10)*10) + ".jpg",final_RGB);      
          if (msg_delay == 0) {
            std::cout << "Saving omni-directional evaluation picture" << std::endl;
            cv::imwrite(pkg_path + "/src/camera/eval_images/omni-dir-eval-img.jpg",final_RGB);      
            // current image is ready for python to evaluate, wait for response from python
            make_new_eval_available = false;
            // send zmq message to python indicating that new image is available
            std::cout << "Sending message to bearingnet to evaluate available image." << std::endl;
            py_bridge_ptr->getRender2(frame_id);
            // get the home vector coords from Python
            evaluation_complete = py_bridge_ptr->handleEvaluationOutput();
          } else {
            msg_delay++;
          }
        }
        frame_id++;
      } 
    // if statement to prevent waiting for messages that are not going to be send since not requested yet
    } else {
      // send the home vector coords to Unity
      if (evaluation_complete) {
        std::string home_vector = py_bridge_ptr->getHomeVectorCoords();
        // TEST
        //float x_home_vector = 0.5f;
        //float y_home_vector = -0.5f;
        //std::string real_home_vector = std::to_string(x_home_vector) + ", " + std::to_string(y_home_vector);
        // TEST
        unity_bridge_ptr->getEvaluationHomingStep(home_vector);
        // give the home vector coords message some time to arrive before the next evaluation message is send
        //std::cout << "Waiting on response from Unity" << std::endl;
        //usleep(0.5e6);
        // check whether the step has been evaluated and moved
        make_new_eval_available = unity_bridge_ptr->handleEvaluationStepOutput();
        //std::cout << "make_new_eval_available: " + std::to_string(make_new_eval_available) << std::endl;
        // give the home vector coords message some time to arrive before the next evaluation message is send
        //std::cout << "Waiting on pos and rot storage from Unity" << std::endl;
        //usleep(0.5e6);
        //evaluation_complete = false;
        //make_new_eval_available = true;
        std::cout << "make_new_eval_available: " + std::to_string(make_new_eval_available) << std::endl;
      }
    }
  }
  
  std::cout << "All stages of online simulation complete. Program exit.";

  ros::shutdown();
  return 0;
}

Matrix<3,3> rotMatrixXrounded(float angle) {
  return (Matrix<3, 3>() <<   1,            0,            0,
                              0,            round(cos(angle)),   -round(sin(angle)), 
                              0,            round(sin(angle)),   round(cos(angle)))
  .finished();
}

Matrix<3,3> rotMatrixYrounded(float angle) {
  return (Matrix<3, 3>() <<   round(cos(angle)),   0,            round(sin(angle)),
                              0,            1,            0, 
                              -round(sin(angle)),  0,            round(cos(angle)))
  .finished();
}

Matrix<3,3> rotMatrixZrounded(float angle) {
  return (Matrix<3, 3>() <<   round(cos(angle)),   -round(sin(angle)),  0,
                              round(sin(angle)),   round(cos(angle)),   0, 
                              0,            0,            1)
  .finished();
}

