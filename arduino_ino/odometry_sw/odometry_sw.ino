/* Using encoder to calculate the angular velocity of the motors
 * and transport them through serial to PI for control and dead  
 * reckoning, for usage of differential-drive mobile robot.
 * Developed by Sean Lu at Nov., 2018. 
 */
/*
 * Notation:
 *  1-> FL: front left
 *  2-> FR: front right
 *  3-> RL: rear left
 *  4-> RR: rear right
 */
 
#define SPD_INT_FL 4
#define SPD_INT_FR 5
#define SPD_INT_RL 6
#define SPD_INT_RR 7

#define CPR 2970.0   // Encoder Counts Per Revolution

volatile long encoder_pre_FL, encoder_pre_FR, encoder_pre_RL, encoder_pre_RR; // present
volatile long encoder_pos_FL, encoder_pos_FR, encoder_pos_RL, encoder_pos_RR; // post
long time_;
double hz = 50; // 50Hz

void setup()
{
  pinMode(SPD_INT_FL, INPUT);
  pinMode(SPD_INT_FR, INPUT);
  pinMode(SPD_INT_RL, INPUT);
  pinMode(SPD_INT_RR, INPUT);
  encoder_pre_FL = 0; encoder_pre_FR = 0, encoder_pre_RL = 0, encoder_pre_RR = 0;
  encoder_pos_FL = 0; encoder_pos_FR = 0, encoder_pos_RL = 0, encoder_pos_RR = 0;
  Serial.begin(57600);
  // Regist interrupt callback functions
  // UNO pin 18, call Encoder_FL when the signal is rising
  attachInterrupt(5, Encoder_FL, RISING);
  // UNO pin 19, call Encoder_FR when the signal is rising
  attachInterrupt(4, Encoder_FR, RISING);
  // UNO pin 20, call Encoder_RL when the signal is rising
  attachInterrupt(3, Encoder_RL, RISING);
  // UNO pin 21, call Encoder_RR when the signal is rising
  attachInterrupt(2, Encoder_RR, RISING);
  time_ = millis();
}

void loop()
{
  if(millis() - time_ >= 1000/hz) {
    long dt = (millis() - time_); // Time difference, in ms
    time_ = millis(); // Update time
    // Calculate angular velocity, times 100 (100*rad/s)
    double w_fl = (encoder_pre_FL - encoder_pos_FL)*2*PI/CPR / dt * 1000 * 100;
    double w_fr = (encoder_pre_FR - encoder_pos_FR)*2*PI/CPR / dt * 1000 * 100;
    double w_rl = (encoder_pre_RL - encoder_pos_RL)*2*PI/CPR / dt * 1000 * 100;
    double w_rr = (encoder_pre_RR - encoder_pos_RR)*2*PI/CPR / dt * 1000 * 100;
    // Update encoder
    encoder_pos_FL = encoder_pre_FL;
    encoder_pos_FR = encoder_pre_FR;
    encoder_pos_RL = encoder_pre_RL;
    encoder_pos_RR = encoder_pre_RR;
    // Print data in serial
    // Data format:
    // w_fl w_fr w_rl w_rr
    Serial.print(w_fl); Serial.print(" ");
    Serial.print(w_fr); Serial.print(" ");
    Serial.print(w_rl); Serial.print(" ");
    Serial.println(w_rr);
  }
  
}

void Encoder_FL()
{
  if(digitalRead(SPD_INT_FL) == HIGH) ++encoder_pre_FL;
  else --encoder_pre_FL;
}

void Encoder_FR()
{
  if(digitalRead(SPD_INT_FR) == HIGH) --encoder_pre_FR;
  else ++encoder_pre_FR;
} 

void Encoder_RL()
{
  if(digitalRead(SPD_INT_RL) == HIGH) ++encoder_pre_RL;
  else --encoder_pre_RL;
}

void Encoder_RR()
{
  if(digitalRead(SPD_INT_RR) == HIGH) --encoder_pre_RR;
  else ++encoder_pre_RR;
}
