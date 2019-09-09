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

volatile long encoder_pos_L1, encoder_pos_R1; // post
long time_;
double hz = 70; // 70Hz
char rcv_char;
String rcv_str = ""; // Serial input string
bool dir_L1, dir_R1; // Direction of motor (true -> forward | false -> backward)
String temp_str;
int count = 0;
int msg[4] = {0, 0, 1, 1}; // Serial reading message: {pwm_L1, pwm_R1, dir_L1, dir_R1}

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
      if (rcv_str[0] != '#' || rcv_str[rcv_str.length()-1] != '%'){ // wrong message
        Serial.println("Wrong MSG!");
        rcv_str = "";
        return;
       }
      for (int i = 1; i < rcv_str.length(); ++i){
        while (rcv_str[i] != '%'){
          temp_str += rcv_str[i];
          i++;
        }
        if (count < 4){
          msg[count] = temp_str.toInt();
          temp_str = "";
          count++;
        }
      }
      analogWrite(MOTOR_PWM_L1, msg[0]);
      analogWrite(MOTOR_PWM_R1, msg[1]);
      if (msg[2]==1){dir_L1 = true; digitalWrite(MOTOR_DIR_L1, HIGH);}
      else {dir_L1 = false; digitalWrite(MOTOR_DIR_L1, LOW);}
      if (msg[3]==1){dir_R1 = true; digitalWrite(MOTOR_DIR_R1, HIGH);}
      else {dir_R1 = false; digitalWrite(MOTOR_DIR_R1, LOW);}
      rcv_str = "";
      count = 0;
      temp_str = "";
      Serial.print(msg[0]); Serial.print(" ");
      Serial.print(msg[1]); Serial.print(" ");
      Serial.print(msg[2]); Serial.print(" ");
      Serial.println(msg[3]);
    }
  }
  if(millis() - time_ >= 1000/hz) {
    //analogWrite(3, 0);
    long dt = (millis() - time_); // Time difference, in ms
    time_ = millis(); // Update time
//    Serial.print(encoder_pos_L1); Serial.print(" ");
//    Serial.print(encoder_pos_R1); Serial.print(" ");
//    Serial.println("");
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
