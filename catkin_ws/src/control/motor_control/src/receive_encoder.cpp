#include <vector>
#include <signal.h>
#include <serial/serial.h>
#include <pid/pid.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "motor_control/motor_pwm.h"
bool DEBUG = 1;

class vimo_velocity_controller {
 private:
  bool firstData, // If first data received
       firstControl, // If first control pwm is published
       pub_tf; // If publish tf
  int baud; // Serial baudrate
  const int TO = 50; // Serial timeout, not sure what this really are
  long int encoder_pos_l, encoder_pos_r; // posterior
  double freq, // Publish frequency
         width, // Robot width
         radius, // Wheel radius
         cpr, // Encoder count per round
         x, // Robot position x
         y, // Robot position y
         theta, // Robot orientation yaw
         v_l, // Left wheel velocity
         v_r; // Right wheel velocity
  std::string port; // Serial port string
  serial::Serial mySerial; // Serial object
  PID::pid pid_l, pid_r; // PID controller
  ros::NodeHandle _nh, _pnh; // Node handler
  ros::Publisher pub_wheel_odom, pub_pwm; // Publisher
  ros::Subscriber sub_cmd_vel; // Subscriber
  ros::Time last_read, // Last read time
            last_update; // Last update pwm time
  ros::Timer timer; // Timer to read serial data
  nav_msgs::Odometry odom; // Published topic
  // Timer callback, read serial data from arduino and publish wheel odometry
  void cbTimer(const ros::TimerEvent&);
  // Subscriber callback, handle input command velocity and output pwm for motors
  void cbCmdVel(const geometry_msgs::Twist&);
  /* 
   * Set controller pid
   * int side: set which controller, 0 for left and 1 for right 
   * std::vector<double> vec: pid vector
   */
  void setPID(int side, std::vector<double> vec);
 public:
  vimo_velocity_controller(ros::NodeHandle nh, ros::NodeHandle pnh):
   _nh(nh),
   _pnh(pnh),
   firstData(true),
   firstControl(true),
   x(0),
   y(0),
   theta(0),
   odom(nav_msgs::Odometry())
   {
     // Get parameters
     if(!_pnh.getParam("pub_tf", pub_tf)) pub_tf = false;
     if(!_pnh.getParam("port", port)) port = "/dev/ttyACM0";
     if(!_pnh.getParam("baud", baud)) baud = 9600;
     if(!_pnh.getParam("freq", freq)) freq = 100.0;
     if(!_pnh.getParam("width", width)) width = 1.0; // FIXME
     if(!_pnh.getParam("radius", radius)) radius = 0.063; // FIXME
     if(!_pnh.getParam("cpr", cpr)) cpr = 550.0; // FIXME
     // Print parameter information
     ROS_INFO("[%s] pub_tf: %s", ros::this_node::getName().c_str(), (pub_tf==true?"true":"false"));
     ROS_INFO("[%s] port: %s", ros::this_node::getName().c_str(), port.c_str());
     ROS_INFO("[%s] baud: %d", ros::this_node::getName().c_str(), baud);
     ROS_INFO("[%s] freq: %f", ros::this_node::getName().c_str(), freq);
     ROS_INFO("[%s] width: %f", ros::this_node::getName().c_str(), width);
     ROS_INFO("[%s] radius: %f", ros::this_node::getName().c_str(), radius);
     ROS_INFO("[%s] cpr: %f", ros::this_node::getName().c_str(), cpr);
     // Fill static information
     odom.header.frame_id = "odom";
     odom.child_frame_id = "base_link";
     // PID controller related
     pid_l = PID::pid(255, -255); // max min
     pid_r = PID::pid(255, -255); 
     std::vector<double> vec{50.0, 180.0, 0.05}; // FIXME
     setPID(0, vec); setPID(1, vec); 
     // Open serial
     mySerial.setPort(port);
     mySerial.setBaudrate(baud);
     serial::Timeout to = serial::Timeout::simpleTimeout(TO);
     mySerial.setTimeout(to);
     mySerial.open();
     if(!mySerial.isOpen()){ // Fail to open serial
       ROS_ERROR("Cannot open target serial, terminate");
       ros::shutdown();
     } else{ROS_INFO("Serial connect");}
     // Flush data for one second
     ROS_INFO("Flush data...");
     ros::Time now = ros::Time::now();
     while((ros::Time::now() - now).toSec() < 1.5){
       std::string temp;
       mySerial.readline(temp);
     } ROS_INFO("Start to receive data...");
     // Timer, publisher and subscriber
     timer  = _pnh.createTimer(ros::Duration(1/freq), &vimo_velocity_controller::cbTimer, this);
     pub_wheel_odom = _pnh.advertise<nav_msgs::Odometry>("odom", 1);
     pub_pwm = _pnh.advertise<motor_control::motor_pwm>("pwm", 1);
     sub_cmd_vel = _pnh.subscribe("/Vimobot/joy_mapper_node/car_cmd", 1, &vimo_velocity_controller::cbCmdVel, this);
   } // End constructor
  /*
  // Shutdown signal handler
  void mySigintHandler(int);
  */
  ~vimo_velocity_controller(){
    mySerial.close();
    ros::shutdown();
  }
  
};

// Refer to here: https://stackoverflow.com/questions/20617214/how-to-register-a-signal-handler-as-a-class-method
//static vimo_velocity_controller* signal_object;
//extern "C" void signal_handler(int signum) {signal_object->mySigintHandler(signum);}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vimo_velocity_controller");
  ros::NodeHandle nh, pnh("~");
  vimo_velocity_controller foo(nh, pnh);
  // Make signal_object refer to foo
  //signal_object = &foo;
  // Regist signal handler
  //signal(SIGINT, signal_handler);
  while(ros::ok()) ros::spinOnce();
  return 0;
}

void vimo_velocity_controller::cbTimer(const ros::TimerEvent &event){
  if(!mySerial.available()) return;
  std::string res;
  mySerial.readline(res);
  double encoder_pre_l, encoder_pre_r; // present
  std::size_t found = res.find(" ");
  if(found!=std::string::npos){
    encoder_pre_l = atoi(res.substr(0, found).c_str());
    encoder_pre_r = atoi(res.substr(found+1, res.length()).c_str()); 
  } else return;
  // if(DEBUG) ROS_INFO("encoder L : %lf, R : %lf", encoder_pre_l, encoder_pre_r);
  if(firstData) firstData = false;
  else{
    double delta_l = encoder_pre_l - encoder_pos_l,
           delta_r = encoder_pre_r - encoder_pos_r;
    double dt = (ros::Time::now() - last_read).toSec(); // Time difference, in second
    double omega_l = 2*M_PI / cpr * delta_l / dt, // rad/s
           omega_r = 2*M_PI / cpr * delta_r / dt;
    v_l = radius * omega_l; // m/s
    v_r = radius * omega_r; 
    // std::cout << "omega_l: " << omega_l << " omega_r: " << omega_r << " v_l: " << v_l << " v_r: " << v_r << std::endl;
    x += radius / 2 * cos(theta) * (omega_l + omega_r) * dt, // m
    y += radius / 2 * sin(theta) * (omega_l + omega_r) * dt, // m
    theta += radius / width * (-omega_l + omega_r) * dt;
    tf::Quaternion quat; quat.setRPY(0, 0, theta);
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.x = quat.getX();
    odom.pose.pose.orientation.y = quat.getY();
    odom.pose.pose.orientation.z = quat.getZ();
    odom.pose.pose.orientation.w = quat.getW();
    odom.twist.twist.linear.x = radius / 2 * (omega_l + omega_r);
    odom.twist.twist.angular.z = radius / width * (-omega_l + omega_r);
    odom.pose.covariance[0] =  odom.twist.covariance[0] =  0.05; // X
    odom.pose.covariance[7] =  odom.twist.covariance[7] =  0.05; // Y
    odom.pose.covariance[14] = odom.twist.covariance[14] = 0.05; // Z
    odom.pose.covariance[21] = odom.twist.covariance[21] = 0.05; // RX
    odom.pose.covariance[28] = odom.twist.covariance[28] = 0.05; // RY
    odom.pose.covariance[35] = odom.twist.covariance[35] = 0.5;  // RZ
    pub_wheel_odom.publish(odom);
    if(pub_tf){
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(x, y, 0.0));
      transform.setRotation(quat);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    }
  }
  // Update variables
  encoder_pos_l = encoder_pre_l;
  encoder_pos_r = encoder_pre_r;
  last_read = ros::Time::now();
}

void vimo_velocity_controller::cbCmdVel(const geometry_msgs::Twist &msg){
  int pwm_l, pwm_r;
  double v_desired = msg.linear.x,
         omega_desired = msg.angular.z,
         v_l_desired = v_desired - width/2*omega_desired,
         v_r_desired = v_desired + width/2*omega_desired;
  std::cout << "vl_de: " << v_l_desired << " vr_de: " << v_r_desired << " vl: " << v_l << " vr: " << v_r << std::endl; 

  if(v_desired == 0.0 and omega_desired == 0.0) {
    pwm_l = pwm_r = 0; // Stop
    pid_l.resetPid();
    pid_r.resetPid();
  }
  else if(firstControl) {
    firstControl = false;
    pwm_l = (int)pid_l.calculate(v_l_desired, v_l, 0.0); // dt set to zero
    pwm_r = (int)pid_r.calculate(v_r_desired, v_r, 0.0);
  }
  else{
    double dt = (ros::Time::now() - last_update).toSec();
    pwm_l = (int)pid_l.calculate(v_l_desired, v_l, dt);
    pwm_r = (int)pid_r.calculate(v_r_desired, v_r, dt);
  }
  last_update = ros::Time::now();
  if(DEBUG) ROS_INFO("PWM_L: %d, PWM_R: %d", pwm_l, pwm_r);

  motor_control::motor_pwm pwm_msg = motor_control::motor_pwm();
  pwm_msg.pwm_r = pwm_r;
  pwm_msg.pwm_l = pwm_l;
  pub_pwm.publish(pwm_msg);
}

/*
void vimo_velocity_controller::mySigintHandler(int signum){
  ROS_INFO("Received shutdown signal, start to clean up");
  ros::shutdown();
}
*/

void vimo_velocity_controller::setPID(int side, std::vector<double> vec){
  if(side==0){ // left
    pid_l.setKp(vec[0]);
    pid_l.setKi(vec[1]);
    pid_l.setKd(vec[2]);
  }
  else{
    pid_r.setKp(vec[0]);
    pid_r.setKi(vec[1]);
    pid_r.setKd(vec[2]);
  }
}
