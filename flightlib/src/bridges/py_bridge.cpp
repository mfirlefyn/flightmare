#include "flightlib/bridges/py_bridge.hpp"

namespace flightlib {

// constructor
PyBridge::PyBridge()
  : client_address_("tcp://*"),
    pub_port_("10255"),
    sub_port_("10256"),
    num_frames_(0),
    last_downloaded_utime_(0),
    last_download_debug_utime_(0),
    u_packet_latency_(0),
    unity_ready_(false) {
  // initialize connections upon creating unity bridge
  initializeConnections();
}

bool PyBridge::initializeConnections() {
  logger_.info("Initializing ZMQ connection!");

  // create and bind an upload socket
  pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
  pub_.bind(client_address_ + ":" + pub_port_);

  // create and bind a download_socket
  sub_.set(zmqpp::socket_option::receive_high_water_mark, 6);
  sub_.bind(client_address_ + ":" + sub_port_);

  // subscribe all messages from ZMQ
  sub_.subscribe("");

  logger_.info("Initializing ZMQ connections done!");
  return true;
}

bool PyBridge::connectUnity(const SceneID scene_id) {
  Scalar time_out_count = 0;
  Scalar sleep_useconds = 0.2 * 1e5;
  //logger_.info("Setting the scene ID.");
  setScene(scene_id);
  // try to connect unity
  logger_.info("Trying to Connect Unity.");
  std::cout << "[";
  while (!unity_ready_) {
    // if time out
    if (time_out_count / 1e6 > unity_connection_time_out_) {
      std::cout << "]" << std::endl;
      logger_.warn(
        "Unity Connection time out! Make sure that Unity Standalone "
        "or Unity Editor is running the Flightmare.");
      return false;
    }
    // initialize Scene settings
    //logger_.info("Initialize scene settings.");
    bool settings_send = sendInitialSettings();
    bool handled_settings = handleSettings();
    //std::cout << (settings_send && handled_settings) << std::endl;
    if (settings_send && handled_settings) {
      unity_ready_ = true;
    }
    // check if setting is done
    //logger_.info("Check if settings have been received and set.");
    //unity_ready_ = handleSettings();
    // sleep
    usleep(sleep_useconds);
    // increase time out counter
    time_out_count += sleep_useconds;
    // print something
    std::cout << ".";
    std::cout.flush();
  }
  logger_.info("Flightmare Unity is connected.");
  return unity_ready_;
}

bool PyBridge::disconnectUnity() {
  unity_ready_ = false;
  // create new message object
  pub_.close();
  sub_.close();
  return true;
}

bool PyBridge::sendInitialSettings(void) {
  // create new message object
  //logger_.info("Create new message object.");
  zmqpp::message msg;
  // add topic header
  //logger_.info("Add topic header to message.");
  msg << "Pose";
  // create JSON object for initial settings
  //logger_.info("Create JSON object for initial settings.");
  json json_mesg = settings_;
  //std::cout << sizeof(json_mesg.dump()) << std::endl;
  //std::cout << sizeof(json_mesg) << std::endl;
  //std::cout << json_mesg << std::endl;
  msg << json_mesg.dump();
  //std::cout << msg.get(1) << std::endl;
  // send message without blocking
  //logger_.info("Send message with blocking.");
  //std::cout << pub_.send(msg, true) << std::endl;
  // code proceeds without having checked correct sending
  if (pub_.send(msg, true)) {
    return true;
  } else {
    return false;
  }
};

bool PyBridge::handleSettings(void) {
  // create new message object
  zmqpp::message msg;

  bool done = false;
  // Unpack message metadata
  //std::cout << sub_.receive(msg, true) << std::endl;
  if (sub_.receive(msg, true)) {
    std::string metadata_string = msg.get(0);
    std::cout << metadata_string << std::endl;
    // Parse metadata
    if (json::parse(metadata_string).size() > 1) {
      logger_.info("Waiting for connection.");
      return false;  // hack
    }
    logger_.info("JSON data has been received.");
    done = json::parse(metadata_string).at("ready").get<bool>();
    //logger_.info("Scene status ", done);
  }
  return done;
};

bool PyBridge::getRender(const FrameID frame_id) {
  pub_msg_.frame_id = frame_id;
  QuadState quad_state;
  for (size_t idx = 0; idx < pub_msg_.vehicles.size(); idx++) {
    unity_quadrotors_[idx]->getState(&quad_state);
    pub_msg_.vehicles[idx].position = positionRos2Unity(quad_state.p);
    pub_msg_.vehicles[idx].rotation = quaternionRos2Unity(quad_state.q());
  }

  for (size_t idx = 0; idx < pub_msg_.objects.size(); idx++) {
    std::shared_ptr<StaticObject> gate = static_objects_[idx];
    pub_msg_.objects[idx].position = positionRos2Unity(gate->getPosition());
    pub_msg_.objects[idx].rotation = quaternionRos2Unity(gate->getQuaternion());
  }

  // create new message object
  zmqpp::message msg;
  // add topic header
  msg << "Train";
  // create JSON object for pose update and append
  //json json_msg = pub_msg_;
  //msg << json_msg.dump();
  msg << "start";
  // send message without blocking
  pub_.send(msg, true);
  return true;
}

bool PyBridge::getRender2(const FrameID frame_id) {
  pub_msg_.frame_id = frame_id;
  QuadState quad_state;
  for (size_t idx = 0; idx < pub_msg_.vehicles.size(); idx++) {
    unity_quadrotors_[idx]->getState(&quad_state);
    pub_msg_.vehicles[idx].position = positionRos2Unity(quad_state.p);
    pub_msg_.vehicles[idx].rotation = quaternionRos2Unity(quad_state.q());
  }

  for (size_t idx = 0; idx < pub_msg_.objects.size(); idx++) {
    std::shared_ptr<StaticObject> gate = static_objects_[idx];
    pub_msg_.objects[idx].position = positionRos2Unity(gate->getPosition());
    pub_msg_.objects[idx].rotation = quaternionRos2Unity(gate->getQuaternion());
  }

  // create new message object
  zmqpp::message msg;
  // add topic header
  msg << "Evaluate";
  // create JSON object for pose update and append
  //json json_msg = pub_msg_;
  //msg << json_msg.dump();
  msg << "start";
  // send message without blocking
  pub_.send(msg, true);
  return true;
}

bool PyBridge::setScene(const SceneID& scene_id) {
  if (scene_id >= UnityScene::SceneNum) {
    logger_.warn("Scene ID is not defined, cannot set scene.");
    return false;
  }
  // logger_.info("Scene ID is set to %d.", scene_id);
  settings_.scene_id = scene_id;
  return true;
}

bool PyBridge::addQuadrotor(std::shared_ptr<Quadrotor> quad) {
  Vehicle_t vehicle_t;
  // get quadrotor state
  QuadState quad_state;
  if (!quad->getState(&quad_state)) {
    logger_.error("Cannot get Quadrotor state.");
    return false;
  }

  vehicle_t.ID = "quadrotor" + std::to_string(settings_.vehicles.size());
  vehicle_t.position = positionRos2Unity(quad_state.p);
  vehicle_t.rotation = quaternionRos2Unity(quad_state.q());
  vehicle_t.size = scalarRos2Unity(quad->getSize());

  // get camera
  std::vector<std::shared_ptr<RGBCamera>> rgb_cameras = quad->getCameras();
  for (size_t cam_idx = 0; cam_idx < rgb_cameras.size(); cam_idx++) {
    std::shared_ptr<RGBCamera> cam = rgb_cameras[cam_idx];
    Camera_t camera_t;
    camera_t.ID = vehicle_t.ID + "_" + std::to_string(cam_idx);
    camera_t.T_BC = transformationRos2Unity(rgb_cameras[cam_idx]->getRelPose());
    camera_t.channels = rgb_cameras[cam_idx]->getChannels();
    camera_t.width = rgb_cameras[cam_idx]->getWidth();
    camera_t.height = rgb_cameras[cam_idx]->getHeight();
    camera_t.fov = rgb_cameras[cam_idx]->getFOV();
    camera_t.depth_scale = rgb_cameras[cam_idx]->getDepthScale();
    camera_t.enabled_layers = rgb_cameras[cam_idx]->getEnabledLayers();
    camera_t.is_depth = false;
    camera_t.output_index = cam_idx;
    vehicle_t.cameras.push_back(camera_t);

    // add rgb_cameras
    rgb_cameras_.push_back(rgb_cameras[cam_idx]);
  }
  unity_quadrotors_.push_back(quad);

  //
  settings_.vehicles.push_back(vehicle_t);
  pub_msg_.vehicles.push_back(vehicle_t);
  return true;
}

bool PyBridge::addStaticObject(std::shared_ptr<StaticObject> static_object) {
  Object_t object_t;
  object_t.ID = static_object->getID();
  object_t.prefab_ID = static_object->getPrefabID();
  object_t.position = positionRos2Unity(static_object->getPosition());
  object_t.rotation = quaternionRos2Unity(static_object->getQuaternion());
  object_t.size = scalarRos2Unity(static_object->getSize());

  static_objects_.push_back(static_object);
  settings_.objects.push_back(object_t);
  pub_msg_.objects.push_back(object_t);
  //
  return true;
}

bool PyBridge::handleOutputDelayed(const FrameID frame_id) {
  // create new message object
  std::cout << "Create message" << std::endl;
  zmqpp::message msg;
  sub_.receive(msg);
  std::cout << "unpack message metadata" << std::endl;
  // unpack message metadata
  std::string json_sub_msg = msg.get(0);
  std::cout << "parse metadata" << std::endl;
  // parse metadata
  SubMessage_t sub_msg = json::parse(json_sub_msg);
  std::cout << msg.get(0) << std::endl;

  size_t image_i = 1;
  // ensureBufferIsAllocated(sub_msg);
  // making sure that the requested frame_id and the received one are synced
  if (frame_id == sub_msg.frame_id) {
    for (size_t idx = 0; idx < settings_.vehicles.size(); idx++) {
      // update vehicle collision flag
      unity_quadrotors_[idx]->setCollision(sub_msg.sub_vehicles[idx].collision);

      std::cout << "Feed image data to RGB Camera" << std::endl;

      // feed image data to RGB camera
      for (const auto& cam : settings_.vehicles[idx].cameras) {
        for (size_t layer_idx = 0; layer_idx <= cam.enabled_layers.size();
             layer_idx++) {
          if (!layer_idx == 0 && !cam.enabled_layers[layer_idx - 1]) continue;

          if (layer_idx == 1) {
            // depth
            uint32_t image_len = cam.width * cam.height * 4;

            std::cout << "Get raw image bytes from message" << std::endl;

            // Get raw image bytes from ZMQ message.
            // WARNING: This is a zero-copy operation that also casts the input to
            // an array of unit8_t. when the message is deleted, this pointer is
            // also dereferenced.
            const uint8_t* image_data;
            msg.get(image_data, image_i);
            image_i = image_i + 1;
            // Pack image into cv::Mat
            cv::Mat new_image = cv::Mat(cam.height, cam.width, CV_32FC1);
            memcpy(new_image.data, image_data, image_len);
            // Flip image since OpenCV origin is upper left, but Unity's is lower
            // left.
            new_image = new_image * (100.f);
            cv::flip(new_image, new_image, 0);

            std::cout << "Feed image " << cam.output_index << " to queue" << std::endl;

            unity_quadrotors_[idx]
              ->getCameras()[cam.output_index]
              ->feedImageQueue(layer_idx, new_image);


          } else {
            uint32_t image_len = cam.width * cam.height * cam.channels;
            // Get raw image bytes from ZMQ message.
            // WARNING: This is a zero-copy operation that also casts the input to
            // an array of unit8_t. when the message is deleted, this pointer is
            // also dereferenced.
            const uint8_t* image_data;
            msg.get(image_data, image_i);
            image_i = image_i + 1;
            // Pack image into cv::Mat
            cv::Mat new_image =
              cv::Mat(cam.height, cam.width, CV_MAKETYPE(CV_8U, cam.channels));
            memcpy(new_image.data, image_data, image_len);
            // Flip image since OpenCV origin is upper left, but Unity's is lower
            // left.
            cv::flip(new_image, new_image, 0);

            std::cout << (int)new_image.at<uint8_t>(0,0,0) << std::endl; 

            // Tell OpenCv that the input is RGB.
            if (cam.channels == 3) {
              cv::cvtColor(new_image, new_image, CV_RGB2BGR);
            }
            unity_quadrotors_[idx]
              ->getCameras()[cam.output_index]
              ->feedImageQueue(layer_idx, new_image);
          }
        }
      }
    }
  } else {
    std::cout << "pub/sub messages not synced" << std::endl;
  }
  return true;
}

bool PyBridge::handleOutput() {
  // create new message object
  std::cout << "Create message" << std::endl;
  zmqpp::message msg;
  sub_.receive(msg);
  std::cout << "unpack message metadata" << std::endl;
  // unpack message metadata
  std::string json_sub_msg = msg.get(0);
  std::cout << "parse metadata" << std::endl;
  // parse metadata
  SubMessage_t sub_msg = json::parse(json_sub_msg);
  std::cout << msg.get(0) << std::endl;

  size_t image_i = 1;
  // ensureBufferIsAllocated(sub_msg);
  for (size_t idx = 0; idx < settings_.vehicles.size(); idx++) {
    // update vehicle collision flag
    unity_quadrotors_[idx]->setCollision(sub_msg.sub_vehicles[idx].collision);

    std::cout << "Feed image data to RGB Camera" << std::endl;

    // feed image data to RGB camera
    for (const auto& cam : settings_.vehicles[idx].cameras) {
      for (size_t layer_idx = 0; layer_idx <= cam.enabled_layers.size();
           layer_idx++) {
        if (!layer_idx == 0 && !cam.enabled_layers[layer_idx - 1]) continue;

        if (layer_idx == 1) {
          // depth
          uint32_t image_len = cam.width * cam.height * 4;

          std::cout << "Get raw image bytes from message" << std::endl;

          // Get raw image bytes from ZMQ message.
          // WARNING: This is a zero-copy operation that also casts the input to
          // an array of unit8_t. when the message is deleted, this pointer is
          // also dereferenced.
          const uint8_t* image_data;
          msg.get(image_data, image_i);
          image_i = image_i + 1;
          // Pack image into cv::Mat
          cv::Mat new_image = cv::Mat(cam.height, cam.width, CV_32FC1);
          memcpy(new_image.data, image_data, image_len);
          // Flip image since OpenCV origin is upper left, but Unity's is lower
          // left.
          new_image = new_image * (100.f);
          cv::flip(new_image, new_image, 0);

          std::cout << "Feed image " << cam.output_index << " to queue" << std::endl;

          unity_quadrotors_[idx]
            ->getCameras()[cam.output_index]
            ->feedImageQueue(layer_idx, new_image);


        } else {
          uint32_t image_len = cam.width * cam.height * cam.channels;
          // Get raw image bytes from ZMQ message.
          // WARNING: This is a zero-copy operation that also casts the input to
          // an array of unit8_t. when the message is deleted, this pointer is
          // also dereferenced.
          const uint8_t* image_data;
          msg.get(image_data, image_i);
          image_i = image_i + 1;
          // Pack image into cv::Mat
          cv::Mat new_image =
            cv::Mat(cam.height, cam.width, CV_MAKETYPE(CV_8U, cam.channels));
          memcpy(new_image.data, image_data, image_len);
          // Flip image since OpenCV origin is upper left, but Unity's is lower
          // left.
          cv::flip(new_image, new_image, 0);

          std::cout << (int)new_image.at<uint8_t>(0,0,0) << std::endl; 

          // Tell OpenCv that the input is RGB.
          if (cam.channels == 3) {
            cv::cvtColor(new_image, new_image, CV_RGB2BGR);
          }
          unity_quadrotors_[idx]
            ->getCameras()[cam.output_index]
            ->feedImageQueue(layer_idx, new_image);
        }
      }
    }
  }
  return true;
}

bool PyBridge::handleEvaluationOutput() {
  // create new message object
  std::cout << "Receiving Home Vector" << std::endl;
  zmqpp::message msg;
  std::string sub_msg_topic;
  std::string sub_msg;
  std::string sub_msg2;
  // with timeout
  //sub_.set(zmqpp::socket_option::receive_timeout, 5000);
  sub_.receive(msg);
  //sub_.setsockopt(ZMQ_RCVTIMEO, 1000); // set timeout to value of timeout_ms 
  //try {
    // unpack message metadata
  sub_msg_topic = msg.get(0);
  //} catch(std::exception& e) {
    //return false;
  //}
  
  if (sub_msg_topic == "Home") {
    std::cout << msg.get(0) << std::endl;
    sub_msg = msg.get(1);
    sub_msg2 = msg.get(2);
    std::cout << sub_msg << std::endl;
    std::cout << msg.get(2) << std::endl;

    //Vector<2> home_vector;
    // define the delimeter
    //std::string delimiter = ", ";
    //size_t delim_idx = 0;
    // populate the quad_desired_pos and rot variables from delimited string
    //int idx = 0;
    //while ((delim_idx = sub_msg.find(delimiter)) != std::string::npos) {
      //std::cout << sub_msg << std::endl;
      //home_vector[idx] = std::stof(sub_msg.substr(0,delim_idx));
      //sub_msg.erase(0, delim_idx + delimiter.length());
      //idx++;
    //}
    //home_vector[idx] = std::stof(sub_msg2);

    //std::cout << atan2(home_vector[1],home_vector[0])*180/PI << std::endl;

    // keep in account the multiple tries for sending from python
    //if (std::stof(sub_msg2) >= 110.0f && !accept_done) {
    home_vector_coords = sub_msg;
    return true;
    //accept_done = true;
    //}
  } 
  
  //if (sub_msg2 == "263") {
    // reset for next message
    //accept_done = false;
  //}

  return false;
}

std::string PyBridge::getHomeVectorCoords() {
  return home_vector_coords;
}

bool PyBridge::getPointCloud(PointCloudMessage_t& pointcloud_msg,
                                Scalar time_out) {
  // create new message object
  zmqpp::message msg;
  // add topic header
  msg << "PointCloud";
  // create JSON object for initial settings
  json json_msg = pointcloud_msg;
  msg << json_msg.dump();
  // send message without blocking
  pub_.send(msg, true);

  std::cout << "Generate PointCloud: Timeout=" << (int)time_out << " seconds."
            << std::endl;

  Scalar run_time = 0.0;
  while (!std::experimental::filesystem::exists(
    pointcloud_msg.path + pointcloud_msg.file_name + ".ply")) {
    if (run_time >= time_out) {
      logger_.warn("Timeout... PointCloud was not saved within expected time.");
      return false;
    }
    std::cout << "Waiting for Pointcloud: Current Runtime=" << (int)run_time
              << " seconds." << std::endl;
    usleep((time_out / 10.0) * 1e6);
    run_time += time_out / 10.0;
  }
  return true;
}

}  // namespace flightlib
