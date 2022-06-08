#include <string>
#include <fstream>
#include <vector>
#include <iostream>

/*ros headers*/
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>             //quality of service
#include <sensor_msgs/msg/image.hpp>  //sensordatatype
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
//TF2
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
/*converting ros headers*/
#include <cv_bridge/cv_bridge.h>     //opencv iamage type parser
#include <image_transport/image_transport.hpp>  //read compressed image streams
#include <sensor_msgs/image_encodings.hpp>        //useful constants and functions related to image encodings
/*opencv headers*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "marker_interfaces/msg/frame_markers.hpp"


#include <algorithm>
#include <cstdlib>
#include <locale>
#include <memory>
#include <string>

#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/get_env.hpp"
#include "camera_calibration_parsers/parse.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define DEBUG 0
#define TFFRAME "cam_frame"
#define MARKERSGROOTE 0.1

#define CAMERAMATRIX_N 3
#define DISTCOEF_N 5
#define MARKER_INTERVAL 21

using std::placeholders::_1;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

// cv::Mat &cameraMatrix = nullptr;
// cv::Mat &distCoef= nullptr;



class Brige_Hybrid : public rclcpp::Node
{
  public:
    Brige_Hybrid(): Node("brige_hybrid")
    {
      calibrationSubscription = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10, std::bind(&Brige_Hybrid::loadCameraCalibrationStream, this, _1));
      subscription = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&Brige_Hybrid::topic_processor, this, _1));
      publisher = this->create_publisher<sensor_msgs::msg::Image>("processed_Image", 10);
      dataPublisher = this->create_publisher<marker_interfaces::msg::FrameMarkers>("frame_Poses_data",10);
      tfPublisher = std::make_shared<tf2_ros::TransformBroadcaster>(this);  //naar std
      //const string nameFile = "/home/kasper/aruco_ws/calibrate/Calibration_OpenCV.yaml";
      //bool ok = loadCameraCalibration(nameFile);
    }


  private:
    void loadCameraCalibrationStream(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      //cout << "=========" << endl;
      //cout << "distcoef:" << endl;
      std::vector<double, std::allocator<double>> distCoef_ = msg->d;
      distCoef = cv::Mat::zeros(1,DISTCOEF_N, CV_64F);
      for (int r = 0; r<1; r++)
      {
        for (int c = 0; c<DISTCOEF_N; c++)
          {
            distCoef.at<double>(r, c) = distCoef_.at(r*3+c);
          }
      }
      //cout << distCoef << endl;
      std::array<double, 9UL> camCoef_ = msg->k;
      //cout << "tussencoef" << endl;
      //cout << camCoef_ << endl;
      cameraMatrix = cv::Mat::zeros(CAMERAMATRIX_N,CAMERAMATRIX_N, CV_64F);
      for (int r = 0; r<CAMERAMATRIX_N; r++)
        {
          for (int c = 0; c<CAMERAMATRIX_N; c++)
            {
              cameraMatrix.at<double>(r, c) = camCoef_.at(r*3+c);
            }
        }
      //cameraMatrix
      // cout << "camera matrix" << endl;
      // cout << cameraMatrix << endl;
      // cout << "=========" << endl;
    }

    bool loadCameraCalibration(string name)
  {
    ifstream inStream(name);
    if (inStream)
    {
        uint16_t rows, columns;

        inStream >> rows;
        inStream >> columns;

      std::cout << "rows "  << rows << std::endl;
      std::cout << "columns "  << columns << std::endl;
      cameraMatrix = cv::Mat(rows, columns, CV_64F);
        
        for (int r = 0; r<rows; r++)
        {
            for (int c = 0; c<columns; c++)
            {
                double read = 0.0f;
                inStream >> read;
                cameraMatrix.at<double>(r, c) = read;
                cout << cameraMatrix.at<double>(r, c) << endl;

            }
        }

        inStream >> rows;
        inStream >> columns;
        std::cout << "rows "  << rows << std::endl;
        std::cout << "columns "  << columns << std::endl;
        distCoef = cv::Mat::zeros(rows, columns, CV_64F);

        for (int r = 0; r<rows; r++)
        {
            for (int c = 0; c<columns; c++)
            {
                double read = 0.0f;
                inStream >> read;
                distCoef.at<double>(r, c) = read;
                cout << distCoef.at<double>(r, c) << endl;
                
            }
        }

        inStream.close();
        return true;
    }
    return false;
}
    
    bool wantedMarker(int id)
    {
      if (id <= MARKER_INTERVAL)
      {
        return true;
      }
      return false;
    }

    tf2::Transform arucoMarker2Tf(cv::Vec3f rvecs, cv::Vec3f tvecs) const
    {
      cv::Mat marker_rotation(3, 3, CV_32FC1);
      cv::Rodrigues(rvecs, marker_rotation);
      if (DEBUG)
      {
        RCLCPP_INFO(this->get_logger(), "x:%f  y:%f  z:%f",tvecs[0],tvecs[1],tvecs[2]);
        RCLCPP_INFO(this->get_logger(), "i:%f  j:%f  k:%f",rvecs[0],rvecs[1],rvecs[2]);
        RCLCPP_INFO(this->get_logger(), "rodrigazzzz %f  %f  %f", marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2));
        RCLCPP_INFO(this->get_logger(), "rodrigazzzz %f  %f  %f", marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2));
        RCLCPP_INFO(this->get_logger(), "rodrigazzzz %f  %f  %f", marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2));
      }

      marker_rotation = marker_rotation;

      cv::Mat marker_translation = (cv::Mat) tvecs;

      // Origin solution
      tf2::Matrix3x3 marker_tf_rot(
                              marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2),
                              marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2),
                              marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2)
                              );
      tf2::Vector3 marker_tf_tran(
                              marker_translation.at<float>(0,0),
                              marker_translation.at<float>(1,0),
                              marker_translation.at<float>(2,0)
                              );
      return tf2::Transform(marker_tf_rot, marker_tf_tran);
    }

    void topic_processor(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto timer = this->now(); 
      //RCLCPP_INFO(this->get_logger(), "first in first out");
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_INFO(this->get_logger(),"cv_bridge exception: %s", e.what());
        return;
      }


      vector<int> markerIds;
	    vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;

	    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

	    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
	    cv::aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);

      auto poseMessage = marker_interfaces::msg::FrameMarkers();      //make new message
      geometry_msgs::msg::PoseArray P;
        P.header.stamp = this->now();
        P.header.set__frame_id("no_poses_from_frame_found");

      if (markerIds.size() > 0) {
        poseMessage.markers_found = true;
        P.header.set__frame_id("poses_from_frame_found");
        
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, MARKERSGROOTE, cameraMatrix, distCoef, rvecs, tvecs);
        // for each marker
        for(int i=0; i< (int)markerIds.size(); i++)
        {
          if (!wantedMarker(markerIds[i]))
          {
            break;
          }
          poseMessage.ids.push_back(markerIds[i]);
          cv::drawFrameAxes(cv_ptr->image, cameraMatrix, distCoef, rvecs[i], tvecs[i], 0.05);
          /**
          String tmp_string = "pose of marker" + to_string(markerIds[i]) + ":"; 
          for (int rv = 0; rv<(int)sizeof(rvecs[i]); rv++)
          {
            tmp_string += to_string(rvecs[i][rv]);
          }
          tmp_string += "tangential";
          for (int tv = 0; tv<(int)sizeof(tvecs[i]); tv++)
          {
            tmp_string += to_string(tvecs[i][tv]);
          }
         poseMessage.data = tmp_string;
         dataPublisher->publish(poseMessage);
         **/
          /*==================POSE_IN_MESSAGE========================*/

          geometry_msgs::msg::Pose pose;
          tf2::Vector3 pos;
          pos.setX(tvecs[i][0]);
          pos.setY(tvecs[i][1]);
          pos.setZ(tvecs[i][2]);

          tf2::Quaternion q;
          //q.normalize();
          //q.setRPY(0,0,0);
          q.setRPY(rvecs[i][0],rvecs[i][1],rvecs[i][2]);
          //RCLCPP_INFO(this->get_logger(), "markerId:%i i:%f  j:%f  k:%f",markerIds[i],rvecs[i][0],rvecs[i][1],rvecs[i][2]);

          tf2::Transform t;
          t.setOrigin(pos);
          t.setRotation(q);
          t = arucoMarker2Tf(rvecs[i], tvecs[i]);
          
          tf2::toMsg(t, pose);

          P.poses.push_back(pose);
          /**
            RCLCPP_INFO(this->get_logger(), "markerId:%i x:%f  y:%f  z:%f",markerIds[i],tvecs[i][0],tvecs[i][1],tvecs[i][2]);
            RCLCPP_INFO(this->get_logger(), "markerId:%i i:%f  j:%f  k:%f",markerIds[i],rvecs[i][0],rvecs[i][1],rvecs[i][2]);
          **/
          
          //===============TF2==================
          geometry_msgs::msg::TransformStamped tmessage;
          tmessage.header.stamp = P.header.stamp;
          tmessage.header.frame_id = TFFRAME;
          tmessage.child_frame_id = "temp_id_" + std::to_string(poseMessage.ids[i]);
          tmessage.transform = tf2::toMsg(t);

          tfPublisher->sendTransform(tmessage);
        }
        poseMessage.set__markers_pose(P);
      }

      

      // Output modified video stream
      auto message = sensor_msgs::msg::Image();
      cv_ptr->toImageMsg(message);
      publisher->publish(message);
      
      dataPublisher->publish(poseMessage);
      if (DEBUG)
      {
      for (int i = 0; i< (int)markerIds.size(); i++)
      {
        if(i==0)
          RCLCPP_INFO(this->get_logger(), "i found id's:");
        RCLCPP_INFO(this->get_logger(), "   %d",markerIds[i]);
      }
      }
      auto tester =  this->now() - timer;
      RCLCPP_INFO(this->get_logger(), "process time: %f seconds",tester.seconds());
    }
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr calibrationSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::Publisher<marker_interfaces::msg::FrameMarkers>::SharedPtr dataPublisher;
    shared_ptr<tf2_ros::TransformBroadcaster> tfPublisher;
    std_msgs::msg::String::SharedPtr calibrationParameters;
    cv::Mat cameraMatrix, distCoef;
};

int main(int argc, char * argv[])
{
    int i=0;

  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Brige_Hybrid>());
  printf("vreemd");
  rclcpp::shutdown();
  return 0;
}
