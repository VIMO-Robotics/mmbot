#include <ros.h>
#include <ros/time.h>
#include <sis_arm_msgs/Odom_uno.h>

#define SPD_INT_L2 8
#define SPD_INT_R2 9
#define CPR 2970.0
#define RADIUS 0.032
#define WIDTH 0.189
volatile long encoder_pos_L, encoder_pos_R;
volatile long encoder_pre_L, encoder_pre_R;
float x, y, theta, d_theta;
float dis_per_tick = 2* PI* RADIUS / CPR;
float dis_L, dis_R;  

ros::NodeHandle nh;
sis_arm_msgs::Odom_uno msg;
ros::Publisher pub_odom("/odom", &msg);

void setup() {
  // put your setup code here, to run once:
  pinMode(SPD_INT_L2, INPUT);
  pinMode(SPD_INT_R2, INPUT);
  encoder_pos_L = 0;
  encoder_pos_R = 0;
  encoder_pre_L = 0;
  encoder_pre_R = 0;
  x = 0;
  y = 0;
  theta = 0;          
  Serial.begin(57600);
  attachInterrupt(0, Encoder_L, RISING);  //2
  attachInterrupt(1, Encoder_R, RISING);  //3 
  //now = millis();
  nh.initNode();
  nh.advertise(pub_odom);
}

void loop() {

  dis_L = dis_per_tick * (encoder_pre_L - encoder_pos_L);
  dis_R = dis_per_tick * (encoder_pre_R - encoder_pos_R);
  theta += (dis_R - dis_L) / WIDTH;
  d_theta = (dis_R - dis_L) / (2 *WIDTH);
  x += cos(theta-d_theta) * (dis_R + dis_L) / 2;
  y += sin(theta-d_theta) * (dis_R + dis_L) / 2;
  
  msg.odom.x = x;
  msg.odom.y = y;
  msg.odom.theta = theta;
  
  msg.R_tick.data = encoder_pre_R;
  msg.L_tick.data = encoder_pre_L;  
  msg.header.stamp = nh.now();
  
  
  pub_odom.publish(&msg);
  encoder_pos_L = encoder_pre_L;
  encoder_pos_R = encoder_pre_R;

  nh.spinOnce();
  delay(100);   // Update at 10 Hz
}
void Encoder_L()
{
  if(digitalRead(SPD_INT_L2) == HIGH)
    {
      encoder_pre_L--;
    }
    
  else
  {
      encoder_pre_L++;
  }
}
void Encoder_R()
{
  if(digitalRead(SPD_INT_R2) == LOW)
    {
      encoder_pre_R++;
    }
  else
    {
      encoder_pre_R--;
    }
}
