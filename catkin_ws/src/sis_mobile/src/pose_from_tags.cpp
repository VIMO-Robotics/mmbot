#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>

tf::Transform map2tag_gt[8];
tf::Transform car2cam;
ros::Publisher pub_car_pose; 
void cb(const apriltags2_ros::AprilTagDetectionArray &msg)
{
  int len = msg.detections.size();
  if(len == 0) return;
  geometry_msgs::PoseWithCovarianceStamped car_pose;
  car_pose.header = msg.header;
  car_pose.header.frame_id = "map";
  double pose_x = 0, pose_y = 0, pose_z = 0,
         pose_qx = 0, pose_qy = 0, pose_qz = 0, pose_qw = 0;
  for(int i=0; i < len; ++i){
    int tag_id = msg.detections[i].id[0];
    tf::Transform cam2tag, map2car;
    cam2tag.setOrigin(tf::Vector3(msg.detections[i].pose.pose.pose.position.x,
                                  msg.detections[i].pose.pose.pose.position.y,
                                  msg.detections[i].pose.pose.pose.position.z));
    cam2tag.setRotation(tf::Quaternion(msg.detections[i].pose.pose.pose.orientation.x,
                                       msg.detections[i].pose.pose.pose.orientation.y,
                                       msg.detections[i].pose.pose.pose.orientation.z,
                                       msg.detections[i].pose.pose.pose.orientation.w));
    map2car = map2tag_gt[tag_id] * cam2tag.inverse() * car2cam.inverse();
    double x = map2car.getOrigin().getX(),
           y = map2car.getOrigin().getY(),
           z = map2car.getOrigin().getZ(),
           qx = map2car.getRotation().getX(),
           qy = map2car.getRotation().getY(),
           qz = map2car.getRotation().getZ(),
           qw = map2car.getRotation().getW();
    pose_x += x, pose_y += y, pose_z += z,
    pose_qx += qx, pose_qy += qy, pose_qz += qz, pose_qw += qw;
  }
  pose_x /= len, pose_y /= len, pose_z /= len,
  pose_qx /= len, pose_qy /= len, pose_qz /= len, pose_qw /= len;
  //std::cout << pose_x << " " << pose_y << " " << pose_z << " " <<
  //             pose_qx << " " << pose_qy << " " << pose_qz << " " << pose_qw << " " << std::endl;
  tf::Quaternion quat_norm = tf::Quaternion(pose_qx, pose_qy, pose_qz, pose_qw).normalize();
  car_pose.pose.pose.position.x = pose_x;
  car_pose.pose.pose.position.y = pose_y;
  car_pose.pose.pose.position.z = pose_z;
  car_pose.pose.pose.orientation.x = quat_norm.getX();
  car_pose.pose.pose.orientation.y = quat_norm.getY();
  car_pose.pose.pose.orientation.z = quat_norm.getZ();
  car_pose.pose.pose.orientation.w = quat_norm.getW();
  car_pose.pose.covariance[0] = 0.05; // X, 5cm
  car_pose.pose.covariance[7] = 0.05; // Y, 5cm
  car_pose.pose.covariance[14] = 100; // Z, not used
  car_pose.pose.covariance[21] = 100; // RX, not used
  car_pose.pose.covariance[28] = 100; // RY, not used
  car_pose.pose.covariance[35] = 0.15; // RZ, about 8.6 deg
  pub_car_pose.publish(car_pose);  
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_from_tags_node");
  ros::NodeHandle nh;
  pub_car_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/car_pose_from_tag", 10);
  ros::Subscriber sub_detection = nh.subscribe("/tag_detections", 1, cb);
  // Static transform
  map2tag_gt[0].setOrigin(tf::Vector3(2.87, 0.3, 0.316)); map2tag_gt[0].setRotation(tf::Quaternion(0.5, -0.5, -0.5, 0.5));
  map2tag_gt[1].setOrigin(tf::Vector3(2.87, 0.9, 0.325)); map2tag_gt[1].setRotation(tf::Quaternion(0.5, -0.5, -0.5, 0.5));
  map2tag_gt[2].setOrigin(tf::Vector3(2.87, 1.5, 0.324)); map2tag_gt[2].setRotation(tf::Quaternion(0.5, -0.5, -0.5, 0.5));
  map2tag_gt[3].setOrigin(tf::Vector3(2.0, 1.67, 0.326)); map2tag_gt[3].setRotation(tf::Quaternion(0.707, 0, 0, 0.707));
  map2tag_gt[4].setOrigin(tf::Vector3(1.0, 1.67, 0.322)); map2tag_gt[4].setRotation(tf::Quaternion(0.707, 0, 0, 0.707));
  map2tag_gt[5].setOrigin(tf::Vector3(0.13, 0.91, 0.308)); map2tag_gt[5].setRotation(tf::Quaternion(0.5, 0.5, 0.5, 0.5));
  map2tag_gt[6].setOrigin(tf::Vector3(1.0, 0.13, 0.312)); map2tag_gt[6].setRotation(tf::Quaternion(0, 0.707, 0.707, 0));
  map2tag_gt[7].setOrigin(tf::Vector3(2.0, 0.13, 0.3095)); map2tag_gt[7].setRotation(tf::Quaternion(0, 0.707, 0.707, 0));
  car2cam.setOrigin(tf::Vector3(0.03, 0.013, 0.363)); car2cam.setRotation(tf::Quaternion(-0.5, 0.5, -0.5, 0.5));
  ros::spin();
  return 0;
}
