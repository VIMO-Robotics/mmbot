#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <stdlib.h>
#include <vector>
#include <isam/isam.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <cmath>
#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sis_arm_msgs/Odom_uno.h>
#include <action_server/landmark_info.h>


using namespace std;
using namespace isam;
using namespace Eigen;

typedef message_filters::Subscriber<sis_arm_msgs::Odom_uno> odom_sub_type;
typedef message_filters::Subscriber<apriltags2_ros::AprilTagDetectionArray> apriltags_sub_type;
// typedef message_filters::TimeSynchronizer<sis_arm_msgs::Odom_uno, apriltags2_ros::AprilTagDetectionArray> sync_type;
typedef message_filters::sync_policies::ApproximateTime<sis_arm_msgs::Odom_uno, apriltags2_ros::AprilTagDetectionArray> sync_type;

typedef struct{
    unsigned short int tag_id;
    unsigned short int node_id;
}measurement_info_member;

namespace isam_odom_apriltags
{

class iSAMOdomApriltagsServer
{
private:
    
  ros::NodeHandle nh;
  ros::Publisher marker_odom_pub, marker_isam_odom_pub, marker_landmark_pub, marker_measurement_pub;
  ros::Publisher localization_correct_pub, landmark_pub;
  ros::Timer update_timer;

  odom_sub_type *sub_odom;
  apriltags_sub_type *sub_apriltags;
  // sync_type *sync;
  message_filters::Synchronizer<sync_type> *sync;

  visualization_msgs::Marker odom_marker, pose_marker, landmark_marker, measurement_marker;
  geometry_msgs::Point32 correct_msg;
  action_server::landmark_info landmark_info_msg;
  // tf::TransformListener tf_listener_;
  

  unsigned short int tag_id, index, size; 
  unsigned int odom_count, a_count, slam_node_count;

  short int odom_flag, a_flag, robot_stop_flag, update_flag;
  float last_odom_x, last_odom_y, last_odom_theta;
  measurement_info_member measurement_info;

  Slam slam;
  Noise odom_noise;
  Noise apriltags_noise;

  vector<Pose2d_Node*> pose_nodes;
  vector<Point2d_Node*> point_nodes;
  vector<int> landmarks_id;//the id of Apriltags
  vector<measurement_info_member> measurement_info_list;//the id of Measurement

public:
  iSAMOdomApriltagsServer(const std::string name) : 
    nh("~")
  {
    odom_flag = 1; a_flag = robot_stop_flag = update_flag = 0;
    last_odom_x = last_odom_y = last_odom_theta = 0;
    slam_node_count = odom_count = a_count = 0;



    odom_noise = Information(1 * eye(3));
    apriltags_noise = Information(0.1 * eye(2));
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    slam.add_node(new_pose_node);
    pose_nodes.push_back(new_pose_node);
    // create a prior measurement (a factor)
    Pose2d origin(0., 0., 0.);
    Pose2d_Factor* prior = new Pose2d_Factor(pose_nodes[0], origin, odom_noise);
    // add it to the graph
    slam.add_factor(prior);


    sub_odom = new odom_sub_type(nh, "/odom", 100); //100
    sub_apriltags = new apriltags_sub_type(nh, "/tag_detections", 100); //100
    // sync = new sync_type(*sub_odom, *sub_apriltags, 100);
    sync = new message_filters::Synchronizer<sync_type>(sync_type(10), *sub_odom, *sub_apriltags);
    sync->registerCallback(boost::bind(&iSAMOdomApriltagsServer::measureCb, this, _1, _2));
    marker_isam_odom_pub = nh.advertise<visualization_msgs::Marker>("/isam_graph", 10);
    marker_odom_pub = nh.advertise<visualization_msgs::Marker>("/odom_graph", 10);
    marker_landmark_pub = nh.advertise<visualization_msgs::Marker>("/landmark_graph", 10);
    marker_measurement_pub = nh.advertise<visualization_msgs::Marker>("/measurement_graph", 10);
    localization_correct_pub = nh.advertise<geometry_msgs::Point32>("/isam_odom", 10);
    landmark_pub = nh.advertise<action_server::landmark_info>("/landmark_info", 10);

    update_timer = nh.createTimer(ros::Duration(0.5), &iSAMOdomApriltagsServer::graph_update_timer, this);



  }

  void measureCb(const sis_arm_msgs::Odom_uno::ConstPtr& odom_msg, const apriltags2_ros::AprilTagDetectionArray::ConstPtr& apriltags_msg)
  {
    apriltags2_ros::AprilTagDetectionArray tag_msg = *apriltags_msg;

    // cout << "odom_time_stamp: " << odom_msg->header.stamp << " apriltags_time_stamp: " << apriltags_msg->header.stamp << endl;
    // cout << "tag_msg: " << apriltags_msg->detections << endl;
    size = tag_msg.detections.end()-tag_msg.detections.begin();
    // cout << "size: " << size << endl;
    if (size > 0 && robot_stop_flag == 0)
    {
      a_count++;

      if (a_count%4 == 0 || a_flag == 1)
      {
        odom_add_isam(odom_msg);  // update the newest robot state
        odom_flag = 0;

        for (int i=0; i < size ; i++)
        {
          tag_id = tag_msg.detections[i].id[0];
          index = find (landmarks_id.begin(), landmarks_id.end(), tag_id) - landmarks_id.begin();
          // cout << "tag_id: " << tag_id << endl;
          if (find (landmarks_id.begin(), landmarks_id.end(), tag_id) == landmarks_id.end())
          {
            cout << "==============  FIND NEW LANDMARK: " << tag_id << "  ==============" << endl;
            landmarks_id.push_back(tag_id);
            landmark_info_msg.ids.push_back(tag_id);
            Point2d_Node* new_point_node = new Point2d_Node();
            point_nodes.push_back(new_point_node);
            slam.add_node(new_point_node); 

            Point2d measure(tag_msg.detections[i].pose.pose.pose.position.z, - tag_msg.detections[i].pose.pose.pose.position.x);
            Pose2d_Point2d_Factor* measurement = new Pose2d_Point2d_Factor(pose_nodes[slam_node_count], new_point_node, measure, apriltags_noise);
            slam.add_factor(measurement);

            measurement_info.tag_id = tag_id;
            measurement_info.node_id = slam_node_count;
            measurement_info_list.push_back(measurement_info);
          }
          else
          {
            // cout << "old landmark: " << landmarks_id[index] << endl;
            Point2d measure(tag_msg.detections[i].pose.pose.pose.position.z, - tag_msg.detections[i].pose.pose.pose.position.x);
            Pose2d_Point2d_Factor* measurement = new Pose2d_Point2d_Factor(pose_nodes[slam_node_count], point_nodes[index], measure, apriltags_noise);
            slam.add_factor(measurement);

            measurement_info.tag_id = tag_id;
            measurement_info.node_id = slam_node_count;
            measurement_info_list.push_back(measurement_info);
          }

          // cout << "index: " << index << endl;

        }
        a_flag = 0;
      }
    }
    // cout << "odom_count: " << odom_count << "  odom_flag: " << odom_flag << endl;
    if (odom_count%2 == 0 && odom_flag == 1)
    {
      odom_add_isam(odom_msg);
    }

    odom_count++;
    odom_flag = 1;
    // cout << "total_nodes: " << slam_node_count << endl << endl;
  }

  void odom_add_isam(const sis_arm_msgs::Odom_uno::ConstPtr& odom_msg)
  {
    // cout << "odom_msg: " << odom_msg->odom.x << " " << odom_msg->odom.y << " " << odom_msg->odom.theta << endl;
    if (last_odom_x == odom_msg->odom.x && last_odom_y == odom_msg->odom.y && last_odom_theta == odom_msg->odom.theta)
    {
      cout << "============== Robot STOP ==============" << endl;
      if (robot_stop_flag == 0)
        graph_update();

      a_flag = 1; //after restart to move, detect first msg of apriltags
      robot_stop_flag = 1;
    }
    else
    {
      robot_stop_flag = 0;

      Pose2d_Node* new_pose_node = new Pose2d_Node();
      slam.add_node(new_pose_node);
      pose_nodes.push_back(new_pose_node);
      slam_node_count++;
      // connect to previous with odometry measurement
      Pose2d odometry( (odom_msg->odom.x - last_odom_x) * cos(last_odom_theta) + (odom_msg->odom.y - last_odom_y) * sin(last_odom_theta), 
                       (odom_msg->odom.y - last_odom_y) * cos(last_odom_theta) - (odom_msg->odom.x - last_odom_x) * sin(last_odom_theta), 
                      odom_msg->odom.theta - last_odom_theta); // x,y,theta
      Pose2d_Pose2d_Factor* constraint = new Pose2d_Pose2d_Factor(pose_nodes[slam_node_count-1], pose_nodes[slam_node_count], odometry, odom_noise);
      slam.add_factor(constraint);

    }

    localization_correct( (odom_msg->odom.x - last_odom_x) * cos(last_odom_theta - pose_nodes.back()->value().t()) + (odom_msg->odom.y - last_odom_y) * sin(last_odom_theta - pose_nodes.back()->value().t()), 
                       (odom_msg->odom.y - last_odom_y) * cos(last_odom_theta - pose_nodes.back()->value().t()) - (odom_msg->odom.x - last_odom_x) * sin(last_odom_theta - pose_nodes.back()->value().t()), 
                      odom_msg->odom.theta - last_odom_theta); // x,y,theta

    last_odom_x = odom_msg->odom.x;
    last_odom_y = odom_msg->odom.y;
    last_odom_theta = odom_msg->odom.theta;

    geometry_msgs::Point odom_p;
    odom_p.x = last_odom_x;
    odom_p.y = last_odom_y;
    odom_p.z = 0;
    odom_marker.points.push_back(odom_p);
  }

  void graph_update_timer(const ros::TimerEvent& event)
  {
    if (robot_stop_flag == 0)
    {
      graph_update();
      update_flag = 1;
      localization_correct(0,0,0);
    }


  }
  void graph_update()
  {
    ros::Time optimize_start_time = ros::Time::now();
    slam.update();
    ros::Time optimize_end_time = ros::Time::now();
    slam.save("isam_output.graph");

    cout << "================= TOTAL Information =================" << endl;
    cout << "optimizing computing_time: " << optimize_end_time-optimize_start_time << endl;
    cout << "Nodes: " << slam_node_count << " Landmarks: " << landmarks_id.size() 
         << " Measurement: " << measurement_info_list.size() << endl << endl;

    graph_marker_generate();

  }

private:
  void localization_correct(float x, float y,float theta)
  {
    if (update_flag)
    {
      correct_msg.x = pose_nodes.back()->value().x();
      correct_msg.y = pose_nodes.back()->value().y();
      correct_msg.z = pose_nodes.back()->value().t();
      update_flag = 0;
    }
    else
    {
      correct_msg.x += x;
      correct_msg.y += y;
      correct_msg.z += theta;
    }
    localization_correct_pub.publish(correct_msg);
  }
  void graph_marker_generate()
  {
    odom_marker.header.frame_id = pose_marker.header.frame_id = landmark_marker.header.frame_id = measurement_marker.header.frame_id = "/world";
    odom_marker.header.stamp = pose_marker.header.stamp = landmark_marker.header.stamp = measurement_marker.header.stamp = ros::Time::now();
    odom_marker.action = pose_marker.action = landmark_marker.action = measurement_marker.action = visualization_msgs::Marker::ADD;
    odom_marker.id = 0;  pose_marker.id = 1;  landmark_marker.id = 2; measurement_marker.id = 3;
    odom_marker.type = pose_marker.type = 4;
    measurement_marker.type = 5;
    landmark_marker.type = 6;

    odom_marker.pose.orientation.w = pose_marker.pose.orientation.w = landmark_marker.pose.orientation.w =  measurement_marker.pose.orientation.w = 1.0;
    odom_marker.scale.x =  pose_marker.scale.x = odom_marker.scale.y =  pose_marker.scale.y = odom_marker.scale.z =  pose_marker.scale.z = 0.01;
    measurement_marker.scale.x = measurement_marker.scale.y = measurement_marker.scale.z = 0.001;
    landmark_marker.scale.x = landmark_marker.scale.y = 0.05; landmark_marker.scale.z = 0.02;

    odom_marker.color.a = pose_marker.color.a = landmark_marker.color.a = measurement_marker.color.a = 1;
    odom_marker.color.r = pose_marker.color.g = landmark_marker.color.b = landmark_marker.color.g = 1;
    measurement_marker.color.r = measurement_marker.color.g = measurement_marker.color.b = 1 ;

    geometry_msgs::Point p;
    for (int i = 0; i < pose_nodes.end()-pose_nodes.begin(); i += 1)
    {
      p.x = pose_nodes[i]->value().x();
      p.y = pose_nodes[i]->value().y();
      p.z = 0;
      pose_marker.points.push_back(p);
    }

    geometry_msgs::Point l_p;

    for (int i = 0; i < point_nodes.end()-point_nodes.begin(); i++)
    {
      l_p.x = point_nodes[i]->value().x();
      l_p.y = point_nodes[i]->value().y();
      l_p.z = 0;
      landmark_marker.points.push_back(l_p);
      landmark_info_msg.points.push_back(l_p);
    }
    geometry_msgs::Point m_p;
    for (int i = 0; i < measurement_info_list.end()-measurement_info_list.begin(); i++)
    {

      m_p.x = point_nodes[find (landmarks_id.begin(), landmarks_id.end(), measurement_info_list[i].tag_id) - landmarks_id.begin()]->value().x();
      m_p.y = point_nodes[find (landmarks_id.begin(), landmarks_id.end(), measurement_info_list[i].tag_id) - landmarks_id.begin()]->value().y();
      m_p.z = 0;
      measurement_marker.points.push_back(m_p);

      m_p.x = pose_nodes[measurement_info_list[i].node_id]->value().x();
      m_p.y = pose_nodes[measurement_info_list[i].node_id]->value().y();
      measurement_marker.points.push_back(m_p);

    }
    
    marker_isam_odom_pub.publish(pose_marker);
    marker_odom_pub.publish(odom_marker);
    marker_landmark_pub.publish(landmark_marker);
    marker_measurement_pub.publish(measurement_marker);
    landmark_pub.publish(landmark_info_msg);

    pose_marker.points.clear();
    landmark_marker.points.clear();
    landmark_info_msg.points.clear();
    measurement_marker.points.clear();
  } 

  
};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "isam_odom_apriltags_action_server");

  isam_odom_apriltags::iSAMOdomApriltagsServer server("isam_odom_apriltags");
  ros::spin();

  return 0;
}