/* Using encoder to calculate the linear velocity of the motors
 * and transport them through serial to PI for control and dead  
 * reckoning, for usage of differential-drive mobile robot.
 * Developed by Sean Lu at Nov., 2018. 
 */
 
/*
 * L1, R1, L2, R2 stand for 
 * left front, right front, left back and right back motor
 * L1 and R1 for differential drive
 * L1, R1, L2 and R2 for m drive
 */
// Motor direction output pin
#define MOTOR_DIR_L1 34
#define MOTOR_DIR_L2 35
#define MOTOR_DIR_R1 36
#define MOTOR_DIR_R2 37
// PWM output for motor control
#define MOTOR_PWM_L1 8
#define MOTOR_PWM_L2 9
#define MOTOR_PWM_R1 12
#define MOTOR_PWM_R2 13
// For external interrupt pin
#define MOTOR_ENCODER_L1 18
#define MOTOR_ENCODER_L2 19
#define MOTOR_ENCODER_R1 20
#define MOTOR_ENCODER_R2 21
#define CPR 2970.0   // Encoder Counts Per Revolution

volatile long encoder_pos_L1, encoder_pos_R1; // post
long time_;
double hz = 70; // 70Hz
char rcv_char;
String rcv_str = ""; // Serial input string
bool dir_L1, dir_R1; // Direction of motor (true -> forward | false -> backward)

void setup()
{
  Serial.begin(9600);
  pinMode(MOTOR_DIR_L1, OUTPUT);
  pinMode(MOTOR_DIR_R1, OUTPUT);
  encoder_pos_L1 = 0; encoder_pos_R1 = 0;
  dir_L1 = true; dir_R1 = true;
  analogWrite(MOTOR_PWM_L1, 0);
  analogWrite(MOTOR_PWM_R1, 0);
  // Regist interrupt callback functions
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_L1), encoder_L1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_R1), encoder_R1, RISING);
  time_ = millis();
}

void loop()
{
  if (Serial.available()>0) {
    rcv_char = Serial.read();
    if (rcv_char != '\n'){
      rcv_str += rcv_char;
    }
    else{
      int pwm_l = rcv_str.toInt();
      analogWrite(MOTOR_PWM_L1, pwm_l);
      Serial.print("PWM: ");
      Serial.print(pwm_l);
      Serial.println();
      rcv_str = "";
    }
  }
  if(millis() - time_ >= 1000/hz) {
    //analogWrite(3, 0);
    long dt = (millis() - time_); // Time difference, in ms
    time_ = millis(); // Update time
    Serial.print(encoder_pos_L1); Serial.print(" ");
    Serial.print(encoder_pos_R1); Serial.print(" ");
    Serial.println("");
  }
}

void encoder_L1()
{
  if(dir_L1){++encoder_pos_L1;}
  else{--encoder_pos_L1;}
}

void encoder_R1()
{
  if(dir_R1){++encoder_pos_R1;}
  else{--encoder_pos_R1;}
} 
