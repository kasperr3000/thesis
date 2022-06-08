/*cpp*/
#include <string>
#include <vector>
#include <array>

/*ros headers*/
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>             //quality of service
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

/*TF2*/
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>              //eulerstransformations
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>



/*custom message*/
#include "marker_interfaces/msg/frame_markers.hpp"

#define DEBUG false
#define DEBUG2 false
#define OK false

#define UPDATE true
#define UPDATE_FACTOR 0.01

#define ABSOLUUT_ID "world"
#define TFFRAME "cam_frame"   //header parent of tf incoming broadcast
#define ROOT "id_0"
#define MEASURED_MARKERS_N 21

using std::placeholders::_1;

class MarkerMapper : public rclcpp::Node
{
  public:
    MarkerMapper(): Node("marker_mapper")
    {
        received_poses_ = this->create_subscription<marker_interfaces::msg::FrameMarkers>("frame_Poses_data", 10, std::bind(&MarkerMapper::mapper_callback, this, _1));
        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_publisher_cam_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        for (int i = 0; i < MEASURED_MARKERS_N ; i++)
        {
          static_marker_mapper(MEASURED_MAP[i].id, MEASURED_MAP[i].position, MEASURED_MAP[i].rotation);
        }
        
    }

  private:
   void static_marker_mapper(int id, tf2::Vector3 position, tf2::Vector3 orientation)
    {
      rclcpp::Time now = this->get_clock()->now();
      geometry_msgs::msg::TransformStamped t;

      std::string name = "map_id" + std::to_string(id);

      t.header.stamp = now;
      t.header.frame_id = "ROOT";
      t.child_frame_id = name;

      // tf2::Vector3 pos;
      // pos.setX(position.getX());
      // pos.setY(position.getY());
      // pos.setZ(position.getZ());

      // tf2::Quaternion q,q_rot,q_new;
      //     //q.normalize();
      //     //q.setRPY(0,0,0);
      // q.setRPY(orientation.getX()*M_PI/180.0,orientation.getY()*M_PI/180.0,orientation.getZ()*M_PI/180.0);
      //     //RCLCPP_INFO(this->get_logger(), "markerId:%i i:%f  j:%f  k:%f",markerIds[i],rvecs[i][0],rvecs[i][1],rvecs[i][2]);

      // tf2::Transform tussen;
      // tussen.setOrigin(pos);
      // tussen.setRotation(q);

      // t.transform = tf2::toMsg(tussen);


      t.transform.translation.x = position.getX();
      t.transform.translation.y = position.getY();
      t.transform.translation.z = position.getZ();

      tf2::Quaternion q;
      q.setRPY(orientation.getX()*M_PI/180.0,orientation.getY()*M_PI/180.0,orientation.getZ()*M_PI/180.0);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_publisher_->sendTransform(t);
    }

    void mapper_callback(const marker_interfaces::msg::FrameMarkers::SharedPtr markers)
    {
      Frame ref;
      ref.id_i= slamMap.markers_n;

      int toMap_n = 0;                      //aantal te mappen markers
      int toMap[(int)markers->ids.size()];  //id lijst van te mappen markers
      int toUpdate_n = 0;
      int toUpdate[(int)markers->ids.size()];

      if (markers->markers_found) //DOEL : kijken of er een bekende marker in beeld is en de hoogste(tree-gewijs) in three als referentie markeren met ref
      {
        RCLCPP_INFO(this->get_logger(), "in frame ids:");
        //0 VOOR ELKE MARKER IN FRAME:
        for (int i = 0; i<(int)markers->ids.size(); i++)
        {
          int temp_id = (int)markers->ids[i];
          RCLCPP_INFO(this->get_logger(), "%d",temp_id);
          //1 MARKER is AL GEMAPT? => dan aanduiden en als referentie gebruiken (in toekomst  verbetering)
          if (slamMap.markers_found[temp_id])
          {
            toUpdate[toUpdate_n] = temp_id;
            toUpdate_n++;
            for (int temp_i = 0; temp_i<ref.id_i; temp_i++)
            {
              if (slamMap.id_list[temp_i]==temp_id)
              {
                toUpdate[--toUpdate_n] = ref.id;
                if(ref.found)
                  toUpdate_n++;
                ref.id = temp_id;
                ref.id_i = temp_i;
                ref.found = true;
                break;
              }
            }
          }
          //1 NIEUWE MARKER=> AM in SLAM ;)
          else
          {
            //2 ROOTMARKER?
            if (slamMap.markers_n && ref.found || markers->ids[i]==0)
            {
              RCLCPP_INFO(this->get_logger(), "new marker mapped with id: %d", temp_id);
              toMap[toMap_n++] = temp_id;
              slamMap.id_list[slamMap.markers_n++] = temp_id;
              slamMap.markers_found[temp_id] = true;
            }
            //2 GEEN ROOTMARKER
            else
            {
              //als de root nog niet gemapt is=> eerst root aanmaken
              RCLCPP_INFO(this->get_logger(), "new marker %d==>SKIPPED: no referention or rootmarker detected", temp_id);
            }

          }

        }
        if (toUpdate_n)
          RCLCPP_INFO(this->get_logger(), "toUpdate[%d]==%d", toUpdate_n, toUpdate[toUpdate_n-1]);
        if (DEBUG2)
        {
        RCLCPP_INFO(this->get_logger(), "----------------------->map:");
        RCLCPP_INFO(this->get_logger(), "-->gevonden markers: %d", slamMap.markers_n);
        for (int a_ = 0; a_< slamMap.markers_n; a_++)
          RCLCPP_INFO(this->get_logger(), "-->id_list: %d", slamMap.id_list[a_]);
        
        RCLCPP_INFO(this->get_logger(), "-------->referentie inframe:");
        RCLCPP_INFO(this->get_logger(), "-->found: %s", (ref.found)? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "--->id: %d", ref.id);
        RCLCPP_INFO(this->get_logger(), "->id_i: %d", ref.id_i);
        }
      }
      
      if(OK)
      {
      if(ref.found)  //pas mappen bij een gevonden referentie: achteraf kan met apparte stukken gewerkt worden (nu enkel met de tree die de root (id 0) heeft) of seciaal geval: de root moet gemapt worden
      {
        int zknumber = 99;
        for (int zoek = 0; zoek<(int)markers->ids.size(); zoek++) if (markers->ids[zoek]==ref.id) {zknumber = zoek;break;} 
        geometry_msgs::msg::TransformStamped t;
          t.header.stamp = markers->markers_pose.header.stamp;
          t.header.frame_id = "id_" + std::to_string(ref.id);
          t.child_frame_id = "cam" + std::to_string(slamMap.markers_n);
          t.transform.translation.x = markers->markers_pose.poses[zknumber].position.x;
          t.transform.translation.y = markers->markers_pose.poses[zknumber].position.y;
          t.transform.translation.z = markers->markers_pose.poses[zknumber].position.z;
          t.transform.rotation.x = markers->markers_pose.poses[zknumber].orientation.x;
          t.transform.rotation.y = markers->markers_pose.poses[zknumber].orientation.y;
          t.transform.rotation.z = markers->markers_pose.poses[zknumber].orientation.z;
          t.transform.rotation.w = markers->markers_pose.poses[zknumber].orientation.w;
          tf_publisher_->sendTransform(t);
        
        RCLCPP_INFO(this->get_logger(), "reffound");
        for(int i = 0; i<toMap_n; i++)
        {
          for (int zoek = 0; zoek<(int)markers->ids.size(); zoek++) if (markers->ids[zoek]==toMap[i]) {zknumber = zoek;break;}
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = markers->markers_pose.header.stamp;
          t.header.frame_id = "cam" + std::to_string(slamMap.markers_n);
          t.child_frame_id = "id_" + std::to_string(toMap[i]);
          t.transform.translation.x = markers->markers_pose.poses[zknumber].position.x;
          t.transform.translation.y = markers->markers_pose.poses[zknumber].position.y;
          t.transform.translation.z = markers->markers_pose.poses[zknumber].position.z;
          t.transform.rotation.x = markers->markers_pose.poses[zknumber].orientation.x;
          t.transform.rotation.y = markers->markers_pose.poses[zknumber].orientation.y;
          t.transform.rotation.z = markers->markers_pose.poses[zknumber].orientation.z;
          t.transform.rotation.w = markers->markers_pose.poses[zknumber].orientation.w;
          tf_publisher_->sendTransform(t);
        }

      }
      else if ((toMap[0]==0 && toMap_n)) //speciaal geval: root 
      {
        geometry_msgs::msg::TransformStamped t;
          t.header.stamp = markers->markers_pose.header.stamp;
          t.header.frame_id = "world";
          t.child_frame_id = ROOT;
          t.transform.translation.x = 0;
          t.transform.translation.y = 0;
          t.transform.translation.z = 0;
          t.transform.rotation.x = 0;
          t.transform.rotation.y = 0;
          t.transform.rotation.z = 0;
          t.transform.rotation.w = 0;
          tf_publisher_->sendTransform(t);
      }
      }

      /*===TF2 LISTENER================================*/

      //mappen
      if(toMap_n)
      for(int temp_id_i = 0; temp_id_i<toMap_n; temp_id_i++){
        std::string toFrameReferentie = std::string("temp_id_") + std::to_string(ref.id);
        std::string toFrameReferentieAbsoluut = std::string("id_") + std::to_string(ref.id);
        std::string FrameNew = "temp_id_" + std::to_string(toMap[temp_id_i]);
        geometry_msgs::msg::TransformStamped transformStamped;
        geometry_msgs::msg::TransformStamped transformStampedRef;
        try {
          transformStamped = tfBuffer->lookupTransform(toFrameReferentie, FrameNew, markers->markers_pose.header.stamp);
          RCLCPP_INFO(this->get_logger(), "mapped_transform======> refID:%d; New_marker:%d =====> x: %f, y: %f, z: %f",  ref.id, toMap[temp_id_i], transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
          geometry_msgs::msg::TransformStamped t;
            t.header.stamp = markers->markers_pose.header.stamp;
          if(ref.found)
          {
            t.header.frame_id = ROOT;
            t.child_frame_id = "id_" + std::to_string(toMap[temp_id_i]);
            //absolute pad naar root berekenen van ref
            transformStampedRef = tfBuffer->lookupTransform(ROOT, toFrameReferentieAbsoluut, markers->markers_pose.header.stamp);
            RCLCPP_INFO(this->get_logger(), "ref_transform======> refID:%s; New_marker:id_%d =====> x: %f, y: %f, z: %f",  ROOT, ref.id, transformStampedRef.transform.translation.x, transformStampedRef.transform.translation.y, transformStampedRef.transform.translation.z);
            //transformedstmpd met transformstampendref verm tf vorm 
            tf2::Transform transformStampedTF, transformStampedRefTF;
            tf2::fromMsg(transformStamped.transform, transformStampedTF);
            tf2::fromMsg(transformStampedRef.transform, transformStampedRefTF);

            transformStampedTF = transformStampedRefTF*transformStampedTF;
            transformStamped.transform = tf2::toMsg(transformStampedTF);
            //tf2::doTransform(transformStampedRef.transform, t.transform, transformStamped);
          }
          else  //is de root*
          {
            t.header.frame_id = ABSOLUUT_ID;
            t.child_frame_id = "id_" + std::to_string(toMap[temp_id_i]);
            ref.found = true;
            
          }
            t.transform.translation.x = transformStamped.transform.translation.x;
            t.transform.translation.y = transformStamped.transform.translation.y;
            t.transform.translation.z = transformStamped.transform.translation.z;
            t.transform.rotation.x = transformStamped.transform.rotation.x;
            t.transform.rotation.y = transformStamped.transform.rotation.y;
            t.transform.rotation.z = transformStamped.transform.rotation.z;
            t.transform.rotation.w = transformStamped.transform.rotation.w;
          
          tf_publisher_->sendTransform(t);
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "ERROR: Could not transform %s to %s(to_map=%d;ref->%d): %s", toFrameReferentie.c_str(), FrameNew.c_str(), toMap_n,ref.id, ex.what());
        }
        //RCLCPP_INFO(this->get_logger(), "with time:%d", markers->markers_pose.header.stamp);
        //RCLCPP_INFO(this->get_logger(), "---------:%d", this->now());
      }
      
      //map updaten
      if(UPDATE)
      {
      if(toUpdate_n)
      {
        for(int temp_id_i = 0; temp_id_i<toUpdate_n; temp_id_i++){
        std::string toFrameReferentie = "temp_id_" + std::to_string(ref.id);
        std::string toFrameReferentieAbsoluut = "id_" + std::to_string(ref.id);
        std::string FrameNew = "temp_id_" + std::to_string(toUpdate[temp_id_i]);
        geometry_msgs::msg::TransformStamped transformStamped;
        geometry_msgs::msg::TransformStamped transformStampedRef;
        geometry_msgs::msg::TransformStamped transformStampedOriginal;
        try {
          transformStamped = tfBuffer->lookupTransform(toFrameReferentie, FrameNew, markers->markers_pose.header.stamp);
          //RCLCPP_INFO(this->get_logger(), "updateted_transform======> refID:%d; marker:%d =====> x: %f, y: %f, z: %f",  ref.id, toUpdate[temp_id_i], transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
          geometry_msgs::msg::TransformStamped t;
            t.header.stamp = markers->markers_pose.header.stamp;
          if(ref.found)
          {
            t.header.frame_id = ROOT;
            t.child_frame_id = "id_" + std::to_string(toUpdate[temp_id_i]);
            //absolute pad naar root berekenen van ref
            transformStampedRef = tfBuffer->lookupTransform(ROOT, toFrameReferentieAbsoluut, markers->markers_pose.header.stamp);
            transformStampedOriginal = tfBuffer->lookupTransform(ROOT, t.child_frame_id, markers->markers_pose.header.stamp);
            //RCLCPP_INFO(this->get_logger(), "ref_transform======> refID:%s; New_marker:%s =====> x: %f, y: %f, z: %f",  ROOT, toFrameReferentie, transformStampedRef.transform.translation.x, transformStampedRef.transform.translation.y, transformStampedRef.transform.translation.z);
            //transformedstmpd met transformstampendref verm tf vorm 
            tf2::Transform transformStampedTF, transformStampedRefTF,transformStampedOriginalTF;
            tf2::fromMsg(transformStamped.transform, transformStampedTF);
            tf2::fromMsg(transformStampedRef.transform, transformStampedRefTF);
            tf2::fromMsg(transformStampedOriginal.transform, transformStampedOriginalTF);

            //transformStampedTF = (1-UPDATE_FACTOR) * transformStampedOriginalTF + (transformStampedRefTF*transformStampedTF) * (UPDATE_FACTOR);
            transformStampedTF = transformStampedRefTF * transformStampedTF;
            tf2::Matrix3x3 scalerUpdater( UPDATE_FACTOR,  0,              0,
                                          0,              UPDATE_FACTOR,  0,
                                          0,              0,              UPDATE_FACTOR
                                          );
            tf2::Matrix3x3 scalerOriginal(  1-UPDATE_FACTOR,  0,                0,
                                            0,                1-UPDATE_FACTOR,  0,
                                            0,                0,                1-UPDATE_FACTOR
                                          );

            transformStampedTF.setOrigin(scalerUpdater * transformStampedTF.getOrigin() + scalerOriginal * transformStampedOriginalTF.getOrigin());
            tf2Scalar rollO, pitchO , yawO, rollN, pitchN, yawN;
            transformStampedTF.getBasis().getRPY(rollN, pitchN, yawN);
            transformStampedOriginalTF.getBasis().getRPY(rollO, pitchO, yawO);
            rollN = rollN * UPDATE_FACTOR + rollO * (1-UPDATE_FACTOR);
            pitchN = pitchN * UPDATE_FACTOR + pitchO * (1-UPDATE_FACTOR);
            yawN = yawN * UPDATE_FACTOR + yawO * (1-UPDATE_FACTOR);
            tf2::Matrix3x3 temp_update_matrix;
            temp_update_matrix.setRPY(rollN,pitchN,yawN);
            transformStampedTF.setBasis(temp_update_matrix);
            //transformStampedTF.setBasis(transformStampedTF.getBasis().scaled({UPDATE_FACTOR,UPDATE_FACTOR,UPDATE_FACTOR}) * transformStampedOriginalTF.getBasis().scaled({1-UPDATE_FACTOR,1-UPDATE_FACTOR,1-UPDATE_FACTOR}));
            transformStamped.transform = tf2::toMsg(transformStampedTF);
            RCLCPP_INFO(this->get_logger(), "updateted_transform======> marker:%d =====> x: %f, y: %f, z: %f",  toUpdate[temp_id_i], transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            //tf2::doTransform(transformStampedRef.transform, t.transform, transformStamped);
          }
          else  //is de root*
          {
            t.header.frame_id = ABSOLUUT_ID;
            t.child_frame_id = "id_" + std::to_string(toUpdate[temp_id_i]);
            ref.found = true;
            RCLCPP_INFO(this->get_logger(),"ROOT UPDATE????????????????");
          }
            t.transform.translation.x = transformStamped.transform.translation.x;
            t.transform.translation.y = transformStamped.transform.translation.y;
            t.transform.translation.z = transformStamped.transform.translation.z;
            t.transform.rotation.x = transformStamped.transform.rotation.x;
            t.transform.rotation.y = transformStamped.transform.rotation.y;
            t.transform.rotation.z = transformStamped.transform.rotation.z;
            t.transform.rotation.w = transformStamped.transform.rotation.w;
          
          tf_publisher_->sendTransform(t);
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "ERROR: Could not transform %s to %s: %s", toFrameReferentie.c_str(), FrameNew.c_str(), ex.what());
        }
        //RCLCPP_INFO(this->get_logger(), "with time:%d", markers->markers_pose.header.stamp);
        //RCLCPP_INFO(this->get_logger(), "---------:%d", this->now());
        }
      }
      }

      //camera orienteren
      // if(ref.found)
      // {
      //   std::string toFrameReferentie = "temp_id_" + std::to_string(ref.id);
      //   std::string fromFrame = "cam_frame";
      //   geometry_msgs::msg::TransformStamped transformStamped;
      //   try {
      //     //  old   transformStamped = tfBuffer->lookupTransform(toFrameNew, fromFrameReferentie,tf2::TimePointZero);
      //     transformStamped = tfBuffer->lookupTransform(toFrameReferentie, fromFrame, markers->markers_pose.header.stamp);
      //     RCLCPP_INFO(this->get_logger(), "cam_transform======> refID:%d; cam =====> x: %f, y: %f, z: %f", ref.id, transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
      //     geometry_msgs::msg::TransformStamped t;
      //       t.header.stamp = markers->markers_pose.header.stamp;
      //       t.header.frame_id = "id_" + std::to_string(ref.id);
      //       t.child_frame_id = "cam";
      //       t.transform.translation.x = transformStamped.transform.translation.x;
      //       t.transform.translation.y = transformStamped.transform.translation.y;
      //       t.transform.translation.z = transformStamped.transform.translation.z;
      //       t.transform.rotation.x = transformStamped.transform.rotation.x;
      //       t.transform.rotation.y = transformStamped.transform.rotation.y;
      //       t.transform.rotation.z = transformStamped.transform.rotation.z;
      //       t.transform.rotation.w = transformStamped.transform.rotation.w;
      //     tf_publisher_cam_->sendTransform(t);
      //   } catch (tf2::TransformException & ex) {
      //     RCLCPP_INFO(this->get_logger(), "ERROR: Could not transform %s to %s: %s", toFrameReferentie.c_str(), fromFrame.c_str(), ex.what());
      //   }
      // }

      if(ref.found)
      {
        std::string toFrameReferentie = "temp_id_" + std::to_string(ref.id);
        std::string fromFrame = "cam_frame";
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
          //  old   transformStamped = tfBuffer->lookupTransform(toFrameNew, fromFrameReferentie,tf2::TimePointZero);
          transformStamped = tfBuffer->lookupTransform(toFrameReferentie, fromFrame, markers->markers_pose.header.stamp);
          RCLCPP_INFO(this->get_logger(), "cam_transform======> refID:%d; cam =====> x: %f, y: %f, z: %f", ref.id, transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
          geometry_msgs::msg::TransformStamped t;
            t.header.stamp = markers->markers_pose.header.stamp;
            t.header.frame_id = "map_id" + std::to_string(ref.id);
            t.child_frame_id = "cam";
            t.transform.translation.x = transformStamped.transform.translation.x;
            t.transform.translation.y = transformStamped.transform.translation.y;
            t.transform.translation.z = transformStamped.transform.translation.z;
            t.transform.rotation.x = transformStamped.transform.rotation.x;
            t.transform.rotation.y = transformStamped.transform.rotation.y;
            t.transform.rotation.z = transformStamped.transform.rotation.z;
            t.transform.rotation.w = transformStamped.transform.rotation.w;
          RCLCPP_INFO(this->get_logger(), "debug: %s %s \t%f %f %f \t%f %f %f %f", t.header.frame_id, t.child_frame_id, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x, t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
          tf_publisher_cam_->sendTransform(t);
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "ERROR: Could not transform %s to %s: %s", toFrameReferentie.c_str(), fromFrame.c_str(), ex.what());
        }
      }

      if (DEBUG2 || toMap_n)
      {
        RCLCPP_INFO(this->get_logger(), "============to map: %d================",toMap_n);
        for (int a = 0; a<toMap_n; a++)
          RCLCPP_INFO(this->get_logger(), "================id: %d================", toMap[a]);
        RCLCPP_INFO(this->get_logger(), "=====================================");
      }
      if(DEBUG){
      if (markers->markers_found)
        for (int i = 0; i<(int)(markers->ids).size(); i++)
          {
            RCLCPP_INFO(this->get_logger(), "id's markers: %d", markers->ids[i]);
            RCLCPP_INFO(this->get_logger(), "mapped_markers: %d", slamMap.markers_n);
          }
      }
    }



    //nog in headerfile steken
    typedef struct map{
      int markers_n {0};          //aantal markers gevonden tot 250
      int64_t id_list[250];      //lijst sequentieel(in tijd gevonden) opgebouwd door gevonden arucos en id nummering start met nummer 0
      bool markers_found[250];  //bitmap of de marker met id gevonden is == markers_found[id]
      int pose_weight[250]; 
    } Map;
    Map slamMap;

    typedef struct ref{
      bool found {false};
      int id {0};
      int id_i {0};
    } Frame;

   typedef struct measuredMap{
     int id;
     tf2::Vector3 position;
     tf2::Vector3 rotation;
   } MeasuredMap;

    MeasuredMap MEASURED_MAP[MEASURED_MARKERS_N] {
      {0,   {0.157, 0.0, 0.15}, {90.0, 0.0, 180.0}},

      {1,   {0.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},
      {2,   {1.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},
      {3,   {2.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},
      {4,   {3.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},
      {5,   {4.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},

      {6,   {5.0, 0.5, 0.5},     {90.0, 0.0, 270.0}},
      {7,   {5.0, 1.5, 0.5},     {90.0, 0.0, 270.0}},
      {8,   {5.0, 2.5, 0.5},     {90.0, 0.0, 270.0}},
      {9,   {5.0, 3.5, 0.5},     {90.0, 0.0, 270.0}},
      {10,  {5.0, 4.5, 0.5},     {90.0, 0.0, 270.0}},

      {11,  {4.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},
      {12,  {3.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},
      {13,  {2.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},
      {14,  {1.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},
      {15,  {0.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},

      {16,  {0.0, 4.5, 0.5},     {-90.0, 180.0, -90.0}},
      {17,  {0.0, 3.5, 0.5},     {-90.0, 180.0, -90.0}},
      {18,  {0.018, 2.5, 0.5},   {-90.0, 180.0, -90.0}},
      {19,  {0.0, 1.5, 0.5},     {-90.0, 180.0, -90.0}},
      {20,  {0.0, 0.5, 0.5},     {-90.0, 180.0, -90.0}},
      };

    rclcpp::Subscription<marker_interfaces::msg::FrameMarkers>::SharedPtr received_poses_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_cam_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerMapper>());
  rclcpp::shutdown();
  return 0;
}