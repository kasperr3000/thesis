/*cpp*/
#include <string>
#include <vector>
#include <array>
#include <cmath>

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
#include <tf2_eigen/tf2_eigen.h>
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
#define VISUALIZE true
#define VISUALIZE_IMPROVED false
#define FILTER true

#define UPDATE true
#define UPDATE_FACTOR 0.01

#define ABSOLUUT_ID "world"
#define TFFRAME "cam_frame"   //header parent of tf incoming broadcast
#define ROOT "id_0"
#define MEASURED_MARKERS_N 21
#define MAP_STRING "map_id"
#define MARKERS_TEMP_STRING "temp_id_"
#define CAMERA_TEMP_STRING "cam_frame"
#define IMPROVED_CAMERA "camera_improved_"
#define MAP_ROOT "ROOT"
#define INTREPRET_FPS 60.0  //input:30fps -> sample:60 nyquist: sample atleast the double of the frequency
#define INTREPRET_MARGE 0.05 //5cm
#define INTREPRET_ANGLE 0.9 //cosinus marge to 1

using std::placeholders::_1;
using namespace std::chrono_literals;

class MarkerMapper : public rclcpp::Node
{
  public:
    MarkerMapper(): Node("marker_mapper")
    {
        received_poses_ = this->create_subscription<marker_interfaces::msg::FrameMarkers>("frame_Poses_data", 10, std::bind(&MarkerMapper::map_callback, this, _1));
        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_publisher_cam_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        for (int i = 0; i < 3 ; i++)
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

    double distance_to_cam(int temp_id, rclcpp::Time timePoint)
    {
      double distance = pow(9999,2.0);
      std::string toFrameReferentie = "temp_id_" + std::to_string(temp_id);
      std::string fromFrame = "cam_frame";
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        //  old   transformStamped = tfBuffer->lookupTransform(toFrameNew, fromFrameReferentie,tf2::TimePointZero);
        transformStamped = tfBuffer->lookupTransform(toFrameReferentie, fromFrame, timePoint);
        distance = pow((transformStamped.transform.translation.x),2.0) + pow((transformStamped.transform.translation.y),2.0);
      } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "ERROR: Could not transform %s to %s: %s", toFrameReferentie.c_str(), fromFrame.c_str(), ex.what());
      }
      RCLCPP_INFO(this->get_logger(), "decision distance of marker %d = %f", temp_id, distance);
      return distance;
    }

    tf2::Transform interpret_transform(int object_id, const tf2::Transform sample, rclcpp::Time time)
    {
      tf2::Transform determinedTF = sample;
      if (newFrame)
      {
        tf2::Transform transformFrames[(int)INTREPRET_FPS];
        tf2Scalar rollO, pitchO , yawO;
        sample.getBasis().getEulerYPR(yawO, pitchO, rollO);
        std::string object_string = MARKERS_TEMP_STRING + std::to_string(object_id);
        int balance = 0;
        tf2Scalar backup = 0;

        for (int frame_i = 0; frame_i<(int)INTREPRET_FPS; frame_i++)
        {
          try
          {
            rclcpp::Time tussen = time;
            const uint32_t nanoseconden = (1.0/INTREPRET_FPS *1000*1000*1000);
            time = time - rclcpp::Duration(0, nanoseconden);
            double tussen_extr = tussen.seconds() - time.seconds();
            //std::cout << "tijd verschil:" << tussen_extr <<"seconden"<< std::endl;
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tfBuffer->lookupTransform(CAMERA_TEMP_STRING, object_string, time);
            tf2::fromMsg(transformStamped.transform, transformFrames[frame_i]);
            // std::cout << "tf is:"
            //           << " x:"<<transformFrames[frame_i].getOrigin().getX() 
            //           << " y:"<<transformFrames[frame_i].getOrigin().getY() 
            //           << " z:"<<transformFrames[frame_i].getOrigin().getZ() 
            //           << std::endl;
          }
          catch(const std::exception& e)
          {
            std::cerr << e.what() << '\n';
            break;
          }
          
          if (INTREPRET_MARGE > transformFrames[frame_i].getOrigin().distance(sample.getOrigin()))  //check if the transfrom is in the same position~marge as the sample
          {
            tf2Scalar rollN, pitchN, yawN;
            transformFrames[frame_i].getBasis().getEulerYPR(yawN, pitchN, rollN);
            // std::cout << "buffer:"
            //           << "\troll:"<< rollN 
            //           << "\tpitch:"<< pitchN 
            //           << "\tyaw:"<< yawN
            //           << std::endl;
            // std::cout << "sample:"
            //           << "\troll:"<< rollO
            //           << "\tpitch:"<< pitchO 
            //           << "\tyaw:"<< yawO
            //           << std::endl;
            // std::cout << "hoekcos: " << cos(rollO-rollN) << "\tnew" << rollN << "\told" << rollO << std::endl;

            if (abs(cos(yawO-yawN)) > INTREPRET_ANGLE)//anders de hoek * 2
            {
              //RCLCPP_INFO(this->get_logger(), "n: %d  o: %d ",yawN , yawO);
              //std::cout << "in marge: same start-angle"<< std::endl;
              if (rollN * rollO < 0) //negative new is wrong
              {
                --balance;
                //std::cout << "favor: change"<< std::endl;
                if (backup == 0)
                {
                  backup = rollN;
                }
              }
              else
              {
                ++balance;
                //std::cout << "favor: keep"<< std::endl;
              }
            }
          }
          else
          {
            break;
          }
        }

        //RCLCPP_INFO(this->get_logger(), "testing tf of marker %i with balance: %i", object_id, balance);
        if (balance<0) //adjustment needed when balance is lower
        {
          tf2::Matrix3x3 mtrx;
          if (backup==0)
          {
            backup = rollO;
          }
          mtrx.setEulerYPR(yawO,pitchO,backup);
          determinedTF.setBasis(mtrx);
          RCLCPP_INFO(this->get_logger(), "ADJUSTED tf of marker %i", object_id);
          if (newFrame && trustlevel[object_id]>0)
          {
            --trustlevel[object_id];
          } 
        }
        else
        {
          if (newFrame)
          {
            if (balance>0)
            {
              if(trustlevel[object_id]<=balance+(int)INTREPRET_FPS/2)
              {
                ++trustlevel[object_id];
              }
              else
              {
                --trustlevel[object_id];
              }
            }
            else//balance is 0
            {
              trustlevel[object_id]=(int)INTREPRET_FPS/2;
            }
          }
        }

        //otherwise deviding by 0 later
        if (trustlevel[object_id]==0)
        {
          trustlevel[object_id] = 1;
        }

        stability_buffer[object_id] = determinedTF;//stock in buffer for performance
      }
      else
      {
        if (std::isnan(stability_buffer[object_id].getOrigin().getX()) || std::isnan(stability_buffer[object_id].getOrigin().getY()) || std::isnan(stability_buffer[object_id].getOrigin().getZ()))
        {
          RCLCPP_INFO(this->get_logger(), "HAD NAN PROBLEM with the buffer of:%i", object_id);
        }
        else
        {
          determinedTF = stability_buffer[object_id];
        }
      }
      return determinedTF;
    }

    tf2::Transform get_transform(std::string ref_pre, int ref_id, std::string object_pre, int object_id, const rclcpp::Time time)
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      tf2::Transform transformStampedTF;
      bool intrepret = false;
      std::string ref_string = ref_pre + std::to_string(ref_id);
      std::string object_string = object_pre + std::to_string(object_id);

      if (!object_pre.compare(CAMERA_TEMP_STRING))
      {
        object_string = CAMERA_TEMP_STRING;
      }
      if (!ref_pre.compare(CAMERA_TEMP_STRING))
      {
        ref_string = CAMERA_TEMP_STRING;
        if (FILTER)
        {
          intrepret = true;
        }
      }
      if (!ref_pre.compare(MAP_ROOT))
      {
        ref_string = MAP_ROOT;
      }
      
      try
      {
        transformStamped = tfBuffer->lookupTransform(ref_string, object_string, time);
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      tf2::fromMsg(transformStamped.transform, transformStampedTF);
      if (intrepret) 
      {
        transformStampedTF = interpret_transform(object_id, transformStampedTF, time);
      }
      return transformStampedTF;
    }

    Eigen::Vector4f quaternionAverage(std::vector<Eigen::Vector4f> quaternions)
{
	if (quaternions.size() == 0)
	{
		std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
		return Eigen::Vector4f::Zero();
	}

	// first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
	Eigen::Matrix4f A = Eigen::Matrix4f::Zero();

	for (int q=0; q<quaternions.size(); ++q)
		A += quaternions[q] * quaternions[q].transpose();

	// normalise with the number of quaternions
	A /= quaternions.size();

	// Compute the SVD of this 4x4 matrix
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::VectorXf singularValues = svd.singularValues();
	Eigen::MatrixXf U = svd.matrixU();

	// find the eigen vector corresponding to the largest eigen value
	int largestEigenValueIndex;
	float largestEigenValue;
	bool first = true;

	for (int i=0; i<singularValues.rows(); ++i)
	{
		if (first)
		{
			largestEigenValue = singularValues(i);
			largestEigenValueIndex = i;
			first = false;
		}
		else if (singularValues(i) > largestEigenValue)
		{
			largestEigenValue = singularValues(i);
			largestEigenValueIndex = i;
		}
	}

	Eigen::Vector4f average;
	average(0) = U(0, largestEigenValueIndex);
	average(1) = U(1, largestEigenValueIndex);
	average(2) = U(2, largestEigenValueIndex);
	average(3) = U(3, largestEigenValueIndex);

	return average;
}


    tf2::Transform updateCamera(tf2::Transform new_camera, int newtrust)
    {
      tf2::Transform transformStampedTF = new_camera;
      tf2::Transform transformStampedOriginalTF = cameraBuffer;

      const double update_factor = (double)newtrust/(cameraTrust+newtrust) * 0.2;
      cameraTrust = (1-update_factor) * cameraTrust + update_factor * newtrust;

      tf2::Matrix3x3 scalerUpdater( update_factor,  0,              0,
                                    0,              update_factor,  0,
                                    0,              0,              update_factor
                                    );
      tf2::Matrix3x3 scalerOriginal(  1-update_factor,  0,                0,
                                      0,                1-update_factor,  0,
                                      0,                0,                1-update_factor
                                    );
      transformStampedTF.setOrigin(scalerUpdater * transformStampedTF.getOrigin() + scalerOriginal * transformStampedOriginalTF.getOrigin());
/*
      tf2Scalar rollO, pitchO , yawO, rollN, pitchN, yawN;
      transformStampedTF.getBasis().getRPY(rollN, pitchN, yawN);
      transformStampedOriginalTF.getBasis().getRPY(rollO, pitchO, yawO);
      rollN = rollN * update_factor + rollO * (1-update_factor);
      pitchN = pitchN * update_factor + pitchO * (1-update_factor);
      yawN = yawN * update_factor + yawO * (1-update_factor);
      tf2::Matrix3x3 temp_update_matrix;
      temp_update_matrix.setRPY(rollN,pitchN,yawN);
      transformStampedTF.setBasis(temp_update_matrix);
*/
      Eigen::Vector4f rotNew, rotOriginal, rotMean;
      auto rot = transformStampedTF.getRotation();
      rotNew << rot.getX(), rot.getY(), rot.getZ(), rot.getW(); 
      rot = transformStampedOriginalTF.getRotation();
      rotOriginal << rot.getX(), rot.getY(), rot.getZ(), rot.getW(); 
      std::vector<Eigen::Vector4f> rotations;
      rotations.push_back(rotNew);
      rotations.push_back(rotOriginal);
      rotMean = quaternionAverage(rotations);
      tf2::Quaternion q;
      q.setX(rotMean.x());
      q.setY(rotMean.y());
      q.setZ(rotMean.z());
      q.setW(rotMean.w());
      transformStampedTF.setRotation(q);
/*
      geometry_msgs::msg::Transform t1,t2;
      t1 = tf2::toMsg(new_camera);
      t2 = tf2::toMsg(cameraBuffer);

      Eigen::Isometry3d new_eigen = tf2::transformToEigen(t1);
      Eigen::Isometry3d old_eigen = tf2::transformToEigen(t1);

      Eigen::Matrix3d eigen;
      eigen = transformStampedOriginalTF.getBasis();



      new_eigen.scale(update_factor);
      old_eigen.scale(1-update_factor);

      new_eigen = new_eigen * old_eigen;

      tf2::fromMsg(tf2::eigenToTransform(new_eigen),transformStampedTF);
*/

      return transformStampedTF;
    }

    void transform_sender(std::string ref_pre, int ref_id, std::string object_pre, int object_id, tf2::Transform tf, const rclcpp::Time time)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = time;
      if (ref_pre.compare(MAP_ROOT))
      {
        t.header.frame_id = ref_pre + std::to_string(ref_id);
      }
      else
      {
        t.header.frame_id = ref_pre;
      }
      
      t.child_frame_id = object_pre + std::to_string(object_id);
      t.transform = tf2::toMsg(tf);
      //RCLCPP_INFO(this->get_logger(), "debug: %s %s \t%f %f %f \t%f %f %f %f", t.header.frame_id, t.child_frame_id, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x, t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
      try
      {
        tf_publisher_cam_->sendTransform(t);
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      
    }

    void map_callback(const marker_interfaces::msg::FrameMarkers::SharedPtr markers)
    {
      if (!markers->markers_found)
      {
        return;
      }
      auto timer = this->now(); 
      rclcpp::Time time = markers->markers_pose.header.stamp;
      const int number_markers = markers->ids.size();
      tf2::Vector3 kostVectors[number_markers];
      tf2Scalar kost[number_markers];
      tf2::Transform best_camera_kandidaat_tf;
      tf2::Transform camera_improved_tf;
      tf2Scalar lowest_kost = 999;
      int best_marker_id;
      tf2::Vector3 improver;

      //bufferfil
      newFrame = true;
      for (int marker_test_i = 0; marker_test_i<number_markers; marker_test_i++)
      {
        const int marker_test_id = markers->ids[marker_test_i];
        get_transform(CAMERA_TEMP_STRING,0,MARKERS_TEMP_STRING,marker_test_id,time);
      } 
      newFrame = false;


      /*calculate for every marker as candidate the cost(this is the length of the vectorsum of the difrences between the estimated poses to the camera and the mapped poses to the camera with the camera pose estimated to the candidate)*/
      for (int marker_kandidaat_i =0; marker_kandidaat_i<number_markers; marker_kandidaat_i++)
      {
        const int marker_kandidaat_id = markers->ids[marker_kandidaat_i];
        tf2::Transform camera_kandidaat_tf = get_transform(CAMERA_TEMP_STRING,0,MARKERS_TEMP_STRING,marker_kandidaat_id,time);  //get the camera pose to the candidate marker
        if (VISUALIZE) {transform_sender(MAP_STRING,marker_kandidaat_id,"kandidaat_cam_",marker_kandidaat_id, camera_kandidaat_tf.inverse(),time);}

        if (marker_kandidaat_i==0)
        {
          best_camera_kandidaat_tf = camera_kandidaat_tf;
        }

        kostVectors[marker_kandidaat_i].setZero();
        kost[marker_kandidaat_i] = 0;
        for (int marker_test_i = 0; marker_test_i<number_markers; marker_test_i++)
        {
          const int marker_test_id = markers->ids[marker_test_i];
          tf2::Transform marker_to_kandidaat_tf = get_transform(MAP_STRING,marker_kandidaat_id,MAP_STRING,marker_test_id,time);
          tf2::Transform camera_kandidaat_to_marker_tf = get_transform(CAMERA_TEMP_STRING,0,MARKERS_TEMP_STRING,marker_test_id,time);
            if (VISUALIZE) {transform_sender("kandidaat_cam_",marker_kandidaat_id,"test_marker_"  + std::to_string(marker_test_id) + "_of_ " ,marker_kandidaat_id, camera_kandidaat_to_marker_tf,time);}

          tf2::Transform camera_kandidaat_to_map_marker_tf =  camera_kandidaat_tf * marker_to_kandidaat_tf;
            if (VISUALIZE) {transform_sender("kandidaat_cam_",marker_kandidaat_id,"test_estimated_marker_"  + std::to_string(marker_test_id) + "_of_ " ,marker_kandidaat_id, camera_kandidaat_to_map_marker_tf,time);}

          kostVectors[marker_kandidaat_i] += camera_kandidaat_to_marker_tf.getOrigin() - camera_kandidaat_to_map_marker_tf.getOrigin();
        }


        kost[marker_kandidaat_i] = kostVectors[marker_kandidaat_i].length(); //previous with distance
        //RCLCPP_INFO(this->get_logger(), "marker %d kost: %f",marker_kandidaat_id,kost[marker_kandidaat_i]);
        if (lowest_kost>kost[marker_kandidaat_i])
        {
          lowest_kost = kost[marker_kandidaat_i];
          best_camera_kandidaat_tf = camera_kandidaat_tf;
          best_marker_id = marker_kandidaat_id;
          improver = kostVectors[marker_kandidaat_i];
        }
      }
      RCLCPP_INFO(this->get_logger(), "best marker %d kost: %f",best_marker_id,lowest_kost);
        if (VISUALIZE_IMPROVED) {transform_sender(MAP_STRING,best_marker_id,"camera_",best_marker_id, best_camera_kandidaat_tf.inverse(),time);}



      //improve camera pose
      tf2::Transform to_root_tf = get_transform(MAP_ROOT,0,MAP_STRING,best_marker_id,time);
      tf2::Transform camera_marker_to_root_improved_tf = to_root_tf;
      camera_improved_tf = best_camera_kandidaat_tf;
      RCLCPP_INFO(this->get_logger(), "markers=%i",number_markers);
      if (number_markers>1)
      {
        improver *= 1.0/number_markers;
        //RCLCPP_INFO(this->get_logger(), "tf: %f %f %f",camera_improved_tf.getOrigin().getX(),camera_improved_tf.getOrigin().getY(),camera_improved_tf.getOrigin().getZ());
        camera_marker_to_root_improved_tf.setOrigin(to_root_tf.getOrigin() + improver);
        //camera_improved_tf.setOrigin(camera_improved_tf.getOrigin() + improver); //improved camera pose
        //  if (VISUALIZE_IMPROVED) {transform_sender(MAP_STRING,best_marker_id,IMPROVED_CAMERA,best_marker_id, camera_improved_tf.inverse(),time);}
        
      }


      to_root_tf = camera_marker_to_root_improved_tf * camera_improved_tf.inverse();
        RCLCPP_INFO(this->get_logger(), "camera trust: %d\t new pose trust: %d",cameraTrust, trustlevel[best_marker_id]);

      if (VISUALIZE_IMPROVED) {transform_sender(MAP_ROOT,0,"test_before_previous_camera",best_marker_id, to_root_tf,time);}

      tf2::Transform buffer_camera_;
      if (cameraTrust == 0)
      {
        buffer_camera_ = to_root_tf;
        cameraTrust = trustlevel[best_marker_id];
      }
      else
      {
        buffer_camera_ = updateCamera(to_root_tf, trustlevel[best_marker_id]);
      }

      auto origin = buffer_camera_.getOrigin();
      if (std::isnan(origin.getX()) || std::isnan(origin.getY()) || std::isnan(origin.getZ()))
      {
        RCLCPP_INFO(this->get_logger(), "skipped");
      }
      else
      {
        transform_sender(MAP_ROOT,0,IMPROVED_CAMERA,0, buffer_camera_,time);
        cameraBuffer = buffer_camera_;
      }

      //calculate an improved camera_pose
      if(VISUALIZE_IMPROVED)
      {
        tf2::Vector3 improvedKost;
      improvedKost.setZero();
        for (int marker_test_i = 0; marker_test_i<number_markers; marker_test_i++)
        {
          const int marker_test_id = markers->ids[marker_test_i];
          tf2::Transform marker_to_best_kandidaat_tf = get_transform(MAP_STRING,best_marker_id,MAP_STRING,marker_test_id,time);  //get the transform from referentie in map to testmarker in map + improvement
          tf2::Transform camera_best_to_map_marker_tf = camera_improved_tf * marker_to_best_kandidaat_tf;                        //camera_candidate to the marker_candidate to the test_marker
          if (VISUALIZE_IMPROVED) {transform_sender(IMPROVED_CAMERA,best_marker_id,"test_improved_estimated_marker_",marker_test_id, camera_best_to_map_marker_tf,time);}

          //+improving
          tf2::Transform camera_best_to_marker_tf = get_transform(CAMERA_TEMP_STRING,0,MARKERS_TEMP_STRING,marker_test_id,time);  //get the transform from the cam_frame to the testmarker in the cam_frame
          if (VISUALIZE_IMPROVED) {transform_sender(IMPROVED_CAMERA,best_marker_id,"ground_improved_estimated_marker_"  ,marker_test_id, camera_best_to_marker_tf,time);}

          improvedKost += camera_best_to_marker_tf.getOrigin() - camera_best_to_map_marker_tf.getOrigin();
        }
      //RCLCPP_INFO(this->get_logger(), "=====================================best marker improved %d kost: %f",best_marker_id,improvedKost.length());
      }      
      auto tester =  this->now() - timer;
      RCLCPP_INFO(this->get_logger(), "map time: %f seconds",tester.seconds());
    }


    void mapper_callback(const marker_interfaces::msg::FrameMarkers::SharedPtr markers)
    {
      map_callback(markers);
      Frame ref;
      ref.id_i= slamMap.markers_n;
      double ref_distance = pow(9999,2.0);
      int cam_ID = 0;

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
            double temp_distance = distance_to_cam(temp_id, markers->markers_pose.header.stamp);

            if (ref_distance > temp_distance)
            {
              cam_ID = temp_id;
              ref_distance = temp_distance;
            }
            
            // for (int temp_i = 0; temp_i<ref.id_i; temp_i++)
            // {
            //   if (ref_distance > temp_distance)
            //   {
            //     toUpdate[--toUpdate_n] = ref.id;
            //     if(ref.found)
            //       toUpdate_n++;
            //     ref.id = temp_id;
            //     ref.found = true;
            //     ref_distance = temp_distance;
            //     //replace the ref.id with this id, if this marker is closer to the cam-CENTER
            //   }
            // }
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
            //2 is there a ROOTMARKER?
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
        std::string toFrameReferentie = "temp_id_" + std::to_string(cam_ID);
        std::string fromFrame = "cam_frame";
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
          //  old   transformStamped = tfBuffer->lookupTransform(toFrameNew, fromFrameReferentie,tf2::TimePointZero);
          transformStamped = tfBuffer->lookupTransform(toFrameReferentie, fromFrame, markers->markers_pose.header.stamp);
          RCLCPP_INFO(this->get_logger(), "cam_transform======> refID:%d; cam =====> x: %f, y: %f, z: %f", ref.id, transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
          geometry_msgs::msg::TransformStamped t;
            t.header.stamp = markers->markers_pose.header.stamp;
            t.header.frame_id = "map_id" + std::to_string(cam_ID);
            t.child_frame_id = "cam";
            t.transform.translation.x = transformStamped.transform.translation.x;
            t.transform.translation.y = transformStamped.transform.translation.y;
            t.transform.translation.z = transformStamped.transform.translation.z;
            t.transform.rotation.x = transformStamped.transform.rotation.x;
            t.transform.rotation.y = transformStamped.transform.rotation.y;
            t.transform.rotation.z = transformStamped.transform.rotation.z;
            t.transform.rotation.w = transformStamped.transform.rotation.w;
          //RCLCPP_INFO(this->get_logger(), "debug: %s %s \t%f %f %f \t%f %f %f %f", t.header.frame_id, t.child_frame_id, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x, t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
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

    MeasuredMap MEASURED_MAP[3] {
      {5,   {0.0, 0.0, -0.02},   {90.0, 0.0, 180.0}},
      {15,  {0.7, 1.10, 0.31},   {90.0, 0.0, 180.0}},
      {14,  {0.7, 3.76, 0.31},   {90.0, 0.0, 180.0}}
      };

    // MeasuredMap MEASURED_MAP[MEASURED_MARKERS_N] {
    //   {0,   {0.157, 0.0, 0.15}, {90.0, 0.0, 180.0}},

    //   {1,   {0.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},
    //   {2,   {1.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},
    //   {3,   {2.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},
    //   {4,   {3.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},
    //   {5,   {4.427, 0.0, 0.5},   {90.0, 0.0, 180.0}},

    //   {6,   {5.0, 0.5, 0.5},     {90.0, 0.0, 270.0}},
    //   {7,   {5.0, 1.5, 0.5},     {90.0, 0.0, 270.0}},
    //   {8,   {5.0, 2.5, 0.5},     {90.0, 0.0, 270.0}},
    //   {9,   {5.0, 3.5, 0.5},     {90.0, 0.0, 270.0}},
    //   {10,  {5.0, 4.5, 0.5},     {90.0, 0.0, 270.0}},

    //   {11,  {4.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},
    //   {12,  {3.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},
    //   {13,  {2.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},
    //   {14,  {1.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},
    //   {15,  {0.5, 5.0, 0.5},     {-90.0, 180.0, -180.0}},

    //   {20,  {0.0, 4.5, 0.5},     {-90.0, 180.0, -90.0}},
    //   {19,  {0.0, 3.5, 0.5},     {-90.0, 180.0, -90.0}},
    //   {18,  {0.018, 2.5, 0.5},   {-90.0, 180.0, -90.0}},
    //   {17,  {0.0, 1.5, 0.5},     {-90.0, 180.0, -90.0}},
    //   {16,  {0.0, 0.5, 0.5},     {-90.0, 180.0, -90.0}},
    //   };

    int trustlevel[250];
    int cameraTrust;
    tf2::Transform cameraBuffer;
    tf2::Transform stability_buffer[250];
    bool newFrame;

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