const int DIR_MOTOR_L = 7; //PWM MOTOR LEFT
const int DIR_MOTOR_R = 8; //PWM MOTOR RIGHT
const int SPEED_MOTOR_L = 9; //DIRECTION MOTOR LEFT
const int SPPED_MOTOR_R = 10; //DIRECTION MOTOR RIGHT

//ENCODER SET UP
const int ENCODER_R_A = 3;
const int ENCODER_R_B = 5;
//using const because the values aren't going to change and they are 
const int ENCODER_L_A = 4;
const int ENCODER_L_B = 2;

volatile in encoderCount = 0;
//need this 'volatile' because of data issues i think, im unsure need to double check
//????






void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(SPEED_MODE_L, OUTPUT);
  pinMode(SPEED_MODE_R, OUTPUT);
  pinMode(DIR_MOTOR_L, OUTPUT);
  pinMode(DIR_MOTOR_R, OUTPUT);


  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMODE(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMODE(ENCODER_L_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterupt(ENCODER_L_B), readEncoder, RISING);
  //attachInterrupt(digitalPinToInterupt(ENCODER_L_B), encoderCount, RISING);
}
void readEncoder(){
  if(digitalRead(ENCODER_L_A) == HIGH){
    encoderCount++;
  } else{
    encoderCount--;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(DIR_MOTOR_L, HIGH);//go forward // also the left must be high and right must be low
  analogWrite(SPEED_MOTOR_L, 150);

  digitalWrite(DIR_MOTOR_R, LOW);
  analogWrite(SPEED_MOTOR_R, 150);
  Serial.print(encoderCount);

}
