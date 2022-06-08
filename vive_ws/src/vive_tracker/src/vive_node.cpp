#include <string>
#include <chrono>
#include <functional>
#include <string.h>
#include <fstream>
#include <vector>
#include <math.h>

/*ros headers*/
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>            //quality of service
#include <sensor_msgs/msg/image.hpp> //sensordatatype
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
// TF2
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
/*converting ros headers*/
#include <cv_bridge/cv_bridge.h>               //opencv iamage type parser
#include <image_transport/image_transport.hpp> //read compressed image streams
#include <sensor_msgs/image_encodings.hpp>     //useful constants and functions related to image encodings

#include "vive_tracker/srv/calibrate.hpp"

#include "vr_interface.h"

using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

#define DEBUG false
#define ROOT "ROOT"
#define VR_ROOT "vr_base"
#define BASE_ROOT "lighthouse_LHB_D5E9E27C"

class VIVEnode : public rclcpp::Node
{
public:
    VIVEnode() : Node("vivePoseTracker"), vr_() , startup_(false)
    {
        dataPublisher = this->create_publisher<std_msgs::msg::String>("vive_data", 10);
        tfPublisher = std::make_shared<tf2_ros::TransformBroadcaster>(this); // naar std
        timer_ = this->create_wall_timer( 11ms, std::bind(&VIVEnode::vive_tracker_callback, this));
        calibrate_vector = this->create_service<vive_tracker::srv::Calibrate>("calibrate_vive", std::bind(&VIVEnode::calibrate_callback, this, _1, _2));
        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // vr_.setDebugMsgCallback(handleDebugMessages);
        // vr_.setInfoMsgCallback(handleInfoMessages);
        // vr_.setErrorMsgCallback(handleErrorMessages);
        
    }
    // void handleDebugMessages(const std::string &msg) { RCLCPP_DEBUG(this->get_logger()," [VIVE] %s", msg.c_str()); }
    // void handleInfoMessages(const std::string &msg) { RCLCPP_INFO(this->get_logger()," [VIVE] %s", msg.c_str()); }
    // void handleErrorMessages(const std::string &msg) { RCLCPP_ERROR(this->get_logger()," [VIVE] %s", msg.c_str()); }
    

private:
    tf2::Transform get_transform(std::string ref, std::string object)
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      tf2::Transform transformStampedTF;
      try
      {
        transformStamped = tfBuffer->lookupTransform(ref, object, tf2::TimePointZero);

      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      tf2::fromMsg(transformStamped.transform, transformStampedTF);
      return transformStampedTF;
    }
    void transform_sender(std::string ref, std::string object, tf2::Transform tf, const rclcpp::Time time)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = time;
    
      t.header.frame_id = ref;
      t.child_frame_id = object;
      t.transform = tf2::toMsg(tf);
      //RCLCPP_INFO(this->get_logger(), "debug: %s %s \t%f %f %f \t%f %f %f %f", t.header.frame_id, t.child_frame_id, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x, t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
      try
      {
        tfPublisher->sendTransform(t);
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
    }
    void static_transform_sender(std::string ref, std::string object, tf2::Transform tf, const rclcpp::Time time)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = time;
    
      t.header.frame_id = ref;
      t.child_frame_id = object;
      t.transform = tf2::toMsg(tf);
      //RCLCPP_INFO(this->get_logger(), "debug: %s %s \t%f %f %f \t%f %f %f %f", t.header.frame_id, t.child_frame_id, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x, t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
      try
      {
        tf_publisher_->sendTransform(t);
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
    }


    void calibrate_callback(const std::shared_ptr<vive_tracker::srv::Calibrate::Request> request,
                              std::shared_ptr<vive_tracker::srv::Calibrate::Response> response)
    {
        rclcpp::Time now = this->now();
        geometry_msgs::msg::Vector3 vect = request->calibrate_vector;
        tf2::Vector3 vect3;
        vect3.setX(vect.x);
        vect3.setY(vect.y);
        vect3.setZ(vect.z);

        geometry_msgs::msg::Vector3 vectO = request->calibrate_orientation;
        tf2::Quaternion q;
        q.setEuler(vectO.x*M_PI/180.0, vectO.y*M_PI/180.0, vectO.z*M_PI/180.0);

        tf2::Transform expectedTF;
        expectedTF.setRotation(q);
        expectedTF.setOrigin(vect3);
        transform_sender(ROOT,"calibrate",expectedTF,now);


        std::string trackerName = request->trackername;
        tf2::Transform previous_vrTF = get_transform(VR_ROOT,trackerName);
        transform_sender(VR_ROOT,"test_calibrate",previous_vrTF,now);

        tf2::Transform new_vrTF = expectedTF * previous_vrTF.inverse();
        
        ++punt;

        std::string meten =  "punt_"+ std::to_string(punt);

        if (request->calibrate)
        {
            static_transform_sender(ROOT,VR_ROOT,new_vrTF,now);
        }
        else
        {
            static_transform_sender(VR_ROOT,meten,previous_vrTF,now);
        }
        response->response = "verry nice";
    }


    std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
    {
        uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
        if( unRequiredBufferLen == 0 )
            return "";

        char *pchBuffer = new char[ unRequiredBufferLen ];
        unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
        std::string sResult = pchBuffer;
        delete [] pchBuffer;
        return sResult;
    }

    void vive_tracker_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "first in first out");
        if (!startup_)
        {
            bool temp_ = vr_.Init();
            RCLCPP_INFO(this->get_logger(), "status:%s", (temp_)? "great succes" : "******* .....fml");
            if (!temp_)
                return;
            startup_ = true;
        }


        vr_.Update();
        auto tijd = this->now();
        string plakString = "";
        int devcounter_ = 0;
        bool device_is_base_;
        /*
        if (base_root_found_)
        {
            //geometry_msgs::msg::TransformStamped transformStampedRef;
            //transformStampedRef = tfBuffer->lookupTransform(BASE_ROOT, VR_ROOT, tijd);
            tf2::Transform transformStampedRefTF;
            tf2::fromMsg(transformStampedRef.transform, transformStampedRefTF);
        }
        */
        for (int i = 0; i < (int)vr::k_unMaxTrackedDeviceCount; i++)
        {
            double tf_matrix[3][4];
            int dev_type = vr_.GetDeviceMatrix(i, tf_matrix);
            //RCLCPP_INFO(this->get_logger(), "apparaattype:%i,", dev_type);
            // No device
            if (dev_type == 0) continue;

            device_is_base_ = false;
            devcounter_++;
            //geometry_msgs::msg::Pose pose;
            tf2::Vector3 pos;
            pos.setX(tf_matrix[0][3]);
            pos.setY(tf_matrix[1][3]);
            pos.setZ(tf_matrix[2][3]);

            tf2::Quaternion q,qRot;
            // q.normalize();
            // q.setRPY(0,0,0);
            tf2::Matrix3x3 rot_matrix(  tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                                        tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                                        tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);

            rot_matrix.getRotation(q);
            //q.setRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);
            //RCLCPP_INFO(this->get_logger(), "---> i:%f  j:%f  k:%f",tf_matrix[0][3],tf_matrix[1][3],tf_matrix[2][3]);
            //qRot.setRPY(0,0,M_PI_2);
            //q = q * qRot;
            tf2::Transform t;
            t.setOrigin(pos);
            t.setRotation(q);


            //t = arucoMarker2Tf(rvecs[i], tvecs[i]);

            //get device serial number
            std::string cur_sn = GetTrackedDeviceString( vr_.pHMD_, i, vr::Prop_SerialNumber_String);
            plakString += cur_sn;
            std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');
            //tf2::toMsg(t, pose);
            /**
              RCLCPP_INFO(this->get_logger(), "markerId:%i x:%f  y:%f  z:%f",markerIds[i],tvecs[i][0],tvecs[i][1],tvecs[i][2]);
              RCLCPP_INFO(this->get_logger(), "markerId:%i i:%f  j:%f  k:%f",markerIds[i],rvecs[i][0],rvecs[i][1],rvecs[i][2]);
            **/

            //===============TF2==================
            geometry_msgs::msg::TransformStamped tmessage;
            tmessage.header.set__stamp(tijd);
            tmessage.header.frame_id = VR_ROOT;


            if (dev_type == 1)
            {
                continue;
                tmessage.child_frame_id = "hmd" + to_string(i);
            }
            if (dev_type == 2)
            {
                continue;
                tmessage.child_frame_id = "controller" + cur_sn;
            }
            if (dev_type == 3)
            {
                tmessage.child_frame_id = "tracker3.0_" + cur_sn;
            }
            if (dev_type == 4)
            {
                tmessage.child_frame_id = "lighthouse_" + cur_sn;
                if (cur_sn.compare("LHB_D5E9E27C")==0)
                {
                    device_is_base_ = true;
                    transformStampedRefTF.setOrigin(pos);
                    transformStampedRefTF.setRotation(q);
                }

                if (pos.isZero())
                {
                    base_root_found_ = true;
                    if (cur_sn.compare("LHB_C3138C5D")==0)
                        RCLCPP_INFO(this->get_logger(), "ROOT_LIGHTHOUSE: LHB_C3138C5D");
                    else 
                    {   
                        if (cur_sn.compare("LHB_D5E9E27C")==0)
                            RCLCPP_INFO(this->get_logger(), "ROOT_LIGHTHOUSE: LHB_D5E9E27C");
                        else
                            RCLCPP_INFO(this->get_logger(), "PROBLEMMMMMMM===>=<=<=<=<=>=");
                    }
                }
            }
            if (device_is_base_ && false)
            {
                
                tf2::Vector3 posTran;
                pos.setX(0);
                pos.setY(0);
                pos.setZ(0);
                tf2::Quaternion qRot;
                qRot.setRPY(0,0,0);
                t.setOrigin(posTran);
                t.setRotation(qRot);
                RCLCPP_INFO(this->get_logger(), "base is app:%d", devcounter_);
            }
            if (base_root_found_)
            {
                RCLCPP_INFO(this->get_logger(), "transform done app:%d", devcounter_);
            //t = transformStampedRefTF.inverse()*t;
            }
            
            tmessage.transform = tf2::toMsg(t);

            tfPublisher->sendTransform(tmessage);
        }
        RCLCPP_INFO(this->get_logger(), "-----aantal apparaattypen:%i", devcounter_);
        std_msgs::msg::String textMessage;
        textMessage.set__data(plakString);
        dataPublisher->publish(textMessage);
    }

    bool startup_;
    VRInterface vr_;
    int punt = 0;

    bool base_root_found_ = false;
    tf2::Transform transformStampedRefTF;
    tf2::Transform calibrateTF;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dataPublisher;
    shared_ptr<tf2_ros::TransformBroadcaster> tfPublisher;
    rclcpp::Service<vive_tracker::srv::Calibrate>::SharedPtr calibrate_vector;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<VIVEnode>());
    printf("vreemd");
    rclcpp::shutdown();
    return 0;
}
