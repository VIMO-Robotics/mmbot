/* Using encoder to calculate the linear velocity of the motors
 * and transport them through serial to PI for control and dead  
 * reckoning, for usage of differential-drive mobile robot.
 * Developed by Sean Lu at Nov., 2018. 
 */
#define SPD_INT_L2 8
#define SPD_INT_R2 9
#define RADIUS 0.032 // Wheel radius, in meter
#define CPR 2970.0   // Encoder Counts Per Revolution
#define WIDTH 0.172 // Two wheel distance, in meter
// Kuo's one 0.179

volatile long encoder_pre_L, encoder_pre_R; // present
volatile long encoder_pos_L, encoder_pos_R; // post
long time_;
double hz = 70; // 70Hz
double theta = 0; // heading, in rad

void setup()
{
  pinMode(SPD_INT_L2, INPUT);
  pinMode(SPD_INT_R2, INPUT);
  encoder_pre_L = 0; encoder_pre_R = 0;
  encoder_pos_L = 0; encoder_pos_R = 0;
  Serial.begin(57600);
  // Regist interrupt callback functions
  // UNO pin 2, call Encoder_L when the signal is rising
  attachInterrupt(0, Encoder_L, RISING);
  // UNO pin 3, call Encoder_R when the signal is rising
  attachInterrupt(1, Encoder_R, RISING);
  time_ = millis();
}

void loop()
{
  if(Serial.available() > 0) {
    int _ = Serial.read();
    theta = 0;
  }
  if(millis() - time_ >= 1000/hz) {
    long dt = (millis() - time_); // Time difference, in ms
    time_ = millis(); // Update time
    // Calculate distance two wheel traversed and linear velocity
    double s_l = (encoder_pre_L - encoder_pos_L)*2*PI/CPR * RADIUS;
    double s_r = (encoder_pre_R - encoder_pos_R)*2*PI/CPR * RADIUS;
    double v_l = s_l / dt * 1000 * 100; // cm/s
    double v_r = s_r / dt * 1000 * 100;
    // Calculate heading
    theta += (s_r - s_l) / WIDTH;
    // In range [0, 2*PI)
    if(theta >= 2*PI) theta -= 2*PI;
    if(theta < 0) theta += 2*PI;
    // Update encoder
    encoder_pos_L = encoder_pre_L;
    encoder_pos_R = encoder_pre_R;
    // Print data in serial
    // Data format:
    // v_r v_l heading
    Serial.print(v_r); Serial.print(" ");
    Serial.print(v_l); Serial.print(" ");
    Serial.println(theta*100); // Times 100
  }
}

void Encoder_L()
{
  if(digitalRead(SPD_INT_L2) == HIGH) {--encoder_pre_L;}
  else {++encoder_pre_L;}
}

void Encoder_R()
{
  if(digitalRead(SPD_INT_R2) == HIGH) {--encoder_pre_R;}
  else {++encoder_pre_R;}
} 
