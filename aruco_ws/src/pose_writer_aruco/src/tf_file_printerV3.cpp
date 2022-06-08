#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <list>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
/*ros headers*/
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>             //quality of service
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
/*TF2*/
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>              //eulerstransformations
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <bits/stdc++.h>

#include "pose_writer_aruco/srv/file_details.hpp"

//#define FPS_PERIOD 33333333ns
#define FPS_PERIOD 11111111ns

#define ROOT "ROOT"
#define OUTPUTFOLDER_PATH "/home/vintecc/Documents/kasper/"

using namespace std;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

std::list<string> WANTED =
  {
    "tracker3.0_LHR_32DA5600",
    "camera_5",
  };
  //"tracker3.0_LHR_32DA5600"
  /*    "kandidaat_cam_0",
    "kandidaat_cam_1",
    "kandidaat_cam_2",
    "kandidaat_cam_3",
    "kandidaat_cam_4",
    "kandidaat_cam_16",
    "kandidaat_cam_17",
    "kandidaat_cam_18",
    "kandidaat_cam_19"
    
        "lighthouse_LHB_D5E9E27C"
    */
//


class tfFilePrinter : public rclcpp::Node
{
  public:
    tfFilePrinter(): Node("tf_file_printer")
    {
      service = this->create_service<pose_writer_aruco::srv::FileDetails>("file_writer", std::bind(&tfFilePrinter::file_writer_callback, this, _1, _2));
      tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }

  private:
    bool wanted(string name)
    {
      for (string wanted_name : WANTED)
      {
        if (name.compare(wanted_name))
        {
          continue;
        }
        return true;
      }
      return false;
    }

    void writeTransform(ofstream *outfile, string id, string parent=ROOT)
    {
      /*===TF2 LISTENER================================*/
      string toFrameReferentie = parent;
      string FrameNew = id;
      geometry_msgs::msg::TransformStamped transformStamped;
      try 
      {
        transformStamped = tfBuffer->lookupTransform(toFrameReferentie, FrameNew,  tf2::TimePointZero); //the latest transform => 0 (as time)
      } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "ERROR: Could not transform %s to %s: %s", toFrameReferentie.c_str(), FrameNew.c_str(), ex.what());
      }
      tf2::Quaternion q;
      q.setX(transformStamped.transform.rotation.x);
      q.setY(transformStamped.transform.rotation.y);
      q.setZ(transformStamped.transform.rotation.z);
      q.setW(transformStamped.transform.rotation.w);
      tf2Scalar rollO, pitchO , yawO, rollN, pitchN, yawN;

      tf2::Transform TF;
      TF.setRotation(q);
      TF.getBasis().getRPY(rollN, pitchN, yawN);

      (*outfile)<< fixed << setprecision(5) 
                << transformStamped.transform.translation.x << ";"
                << transformStamped.transform.translation.y << ";"
                << transformStamped.transform.translation.z << ";" 
                << rollN << ";"
                << pitchN << ";"
                << yawN << ";";
    }

    void file_writer_callback(const std::shared_ptr<pose_writer_aruco::srv::FileDetails::Request> request,
                              std::shared_ptr<pose_writer_aruco::srv::FileDetails::Response> response)
    {
      string technique = request->techniek;
      double Px = request->position_measured.position.x;
      double Py = request->position_measured.position.y;
      double Pz = request->position_measured.position.z;
      bool rotation_ = request->with_orientation;
      double Ox,Oy,Oz;
      if(rotation_)
      {
        Ox = request->position_measured.orientation.x;
        Oy = request->position_measured.orientation.y;
        Oz = request->position_measured.orientation.z;
      }
      const string filename = request->pre_info +
                              "_" + to_string(Px) +
                              "_" + to_string(Py) +
                              "_" + to_string(Pz) + 
                              ".csv";
      const string folderpath = OUTPUTFOLDER_PATH + technique + "/" + request->testgroep + "/" ;
      const string filepath = OUTPUTFOLDER_PATH + technique + "/" + request->testgroep + "/" + filename;
      string status;

      try
      {
        // Creating a directory
        auto ret = boost::filesystem::create_directories(folderpath);
        if (ret) cout << "created folder follows: "<< folderpath << endl;
        else cout << "no directories created" << endl;
        // Creating a file
        ofstream outfile;
        outfile.open(filepath);
        
        // get the frameids (all the transform-names)
        vector<string> ids;
        tfBuffer->_getFrameStrings(ids);

        // the transform of the frames in ROOT_Reference
        for (int cluster=request->durationnumber; cluster>0; cluster--)
        {
        for (int devices = 0; devices<(int)ids.size(); devices++)
        {
          if (wanted(ids[devices]))
          {
            writeTransform(&outfile,ids[devices]);
          }
          string a = (string)ids[devices];
          RCLCPP_INFO(this->get_logger(), "ignored: %s", a);
          continue;
        }
        outfile << endl;
        // std::chrono::nanoseconds fps_period;
        // fps_period = 33333333;
        rclcpp::sleep_for(FPS_PERIOD);
        }

        //print nog de filename
        //outfile << filename << endl;
        // close the opened file.
        outfile.close();
        status = "congrats data saved at:" + filepath;
      }
      
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
        status = "failed: " + filepath + " IS NOT CREATED";
      }

      response->file_name = status;
    }
    rclcpp::Service<pose_writer_aruco::srv::FileDetails>::SharedPtr service;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<tfFilePrinter>());

  rclcpp::shutdown();
  return 0;
}