/** DO NOT CHANGE THE FOLLOWING DEFINITONS - From UKMARS MazeRunner GitHub - DO NOT WORRY ABOUT IT **/
/** =============================================================================================================== **/
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define __digitalPinToPortReg(P) (((P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(P) (((P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(P) (((P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) (((P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P)-8 : (P)-14))

// general macros/defines
#if !defined(BIT_READ)
#define BIT_READ(value, bit) ((value) & (1UL << (bit)))
#endif
#if !defined(BIT_SET)
#define BIT_SET(value, bit) ((value) |= (1UL << (bit)))
#endif
#if !defined(BIT_CLEAR)
#define BIT_CLEAR(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#if !defined(BIT_WRITE)
#define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))
#endif

#define fast_write_pin(P, V) BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (V));
#define fast_read_pin(P) ((int)(((BIT_READ(*__digitalPinToPINReg(P), __digitalPinToBit(P))) ? HIGH : LOW)))

#else
#define fast_write_pin(P, V) fast_write_pin(P, V)
#define fast_read_pin(P) digitalRead(P)
#endif
/** =============================================================================================================== **/
 
// CODE STARTS HERE
/** DEFINE OUR PINS AND WHICH COMPONENTS THEY ARE CONNECTED TO **/
/** _______________________________________________________________________________________________________________ **/
const int ENCODER_R_A = 3; // ENCODER RIGHT A (ticks first when motor forward)
const int ENCODER_R_B = 5; // ENCODER RIGHT B (ticks first when motor backward) 

const int ENCODER_L_A = 4; // ENCODER LEFT A (ticks first when motor forward)
const int ENCODER_L_B = 2; // ENCODER LEFT B (ticks first when motor backward)

const int SPEED_MOTOR_L = 9; // PWM MOTOR LEFT 
const int SPEED_MOTOR_R = 10; // PWM MOTOR RIGHT 

const int DIR_MOTOR_L = 7; // DIRECTION MOTOR LEFT 
const int DIR_MOTOR_R = 8; // DIRECTION MOTOR RIGHT 

//____________________________________________________________________________
const int EMITTERS = 12; // EMITTERS
const int RED_LED = 13; // RED LED AT H BRIDGE

const int INDICATOR_LED_R = 6; // INDICATOR LED RIGHT 
const int INDICATOR_LED_L = 11; // INDICATOR LED LEFT

// // Phototransistors
// const int RIGHT_SENSOR = A0;
// const int LEFT_SENSOR = A2;
// const int MIDDLE_SENSOR = A1;
#define SR A0
#define SM A1
#define SL A2
int R, M, L; //to store the values of sensors 

//____________________________________________________________________________

// 4 Way switch and push button
const int DIP_SWITCH = A6; 
/** _______________________________________________________________________________________________________________ **/

/* GLOBAL VARIABLES */

volatile int rightEncoderPos = 0; // Counts for right encoder ticks
volatile int leftEncoderPos = 0; // Counts for left encoder ticks

// Variables to help us with our PID
int prevTime = 0;
int prevError;
int errorIntegral;
bool switchOn = false;

//the left-right counter
//int LR;


void setup() {
  Serial.begin(9600);
  
  //_________________MOTORS AND ENCODERS________
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  pinMode(SPEED_MOTOR_L, OUTPUT);
  pinMode(SPEED_MOTOR_R, OUTPUT);
  pinMode(DIR_MOTOR_L, OUTPUT);
  pinMode(DIR_MOTOR_R, OUTPUT);
  //__________________________________________

  pinMode(DIP_SWITCH, INPUT_PULLUP); 

  //________________LEDS_____________________
  pinMode(EMITTERS, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(INDICATOR_LED_R, OUTPUT);
  pinMode(INDICATOR_LED_L, OUTPUT);
  //_________________________________________

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_B), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), readEncoderRight, CHANGE);

  delay(500);//just put for now
}



/** INTERRUPT SERVICE ROUTINES FOR HANDLING ENCODER COUNTING USING STATE TABLE METHOD **/
void readEncoderLeft() {
  static uint8_t prevState = 0; 
  static uint8_t currState = 0; 
  static unsigned long lastTime = 0; 
  
  currState = (fast_read_pin(ENCODER_L_B) << 1) | fast_read_pin(ENCODER_L_A);
  
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  
  // direction based on prev state
  uint8_t direction = (prevState << 2) | currState;
  switch(direction) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      leftEncoderPos++;
      break;
    case 0b0010:
    case 0b1100:
    case 0b0101:
    case 0b1011:
      leftEncoderPos--;
      break;

    default:
      break;
  }

  prevState = currState;
}

void readEncoderRight() {
  static uint8_t prevState = 0; 
  static uint8_t currState = 0; 
  static unsigned long lastTime = 0; 
  
  currState = (fast_read_pin(ENCODER_R_B) << 1) | fast_read_pin(ENCODER_R_A);
  
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  
  uint8_t direction = (prevState << 2) | currState;
  switch(direction) {
    case 0b0100:
    case 0b1010:
    case 0b0111:
    case 0b1001:
      rightEncoderPos++;
      break;
    case 0b1000:
    case 0b0110:
    case 0b1101:
    case 0b0011:
      rightEncoderPos--;
      break;

    default:
      break;
  }
  
  prevState = currState;
}




/** Function to set motor speed and direction for BOTH motors
    @params dir - can either be HIGH or LOW for clockwise / counter clockwise rotation
    @params speed - analogWrite() value between 0-255
**/
//==============================================================================================
void setMotors(int dir, int speed){
  /*now speed can be variable and this is especially great to slow down for turns
  and sprint on straights (that are longer than 1 cell)*/
  analogWrite(SPEED_MOTOR_L, speed);
  analogWrite(SPEED_MOTOR_R, speed);
  
  if(dir == 1){//forward
    fast_write_pin(DIR_MOTOR_L, HIGH);
    fast_write_pin(DIR_MOTOR_R, LOW);
  } else if (dir == -1){//reverse
    fast_write_pin(DIR_MOTOR_L, LOW);
    fast_write_pin(DIR_MOTOR_R, HIGH);
  // } else if (dir == 1 && LR = 1){//left-forward
  //   fast_write_pin(DIR_MOTOR_L, LOW);
  //   //fast_write_pin(DIR_MOTOR_R, HIGH);
  // }/*else if (dir == 1 && LR = 1){//left-reverse
  //   fast_write_pin(DIR_MOTOR_L, LOW);
  //   //fast_write_pin(DIR_MOTOR_R, HIGH);
  // }*/ else if (dir == 1 && LR = 2){//right-forward
  //   //fast_write_pin(DIR_MOTOR_L, LOW);
  //   fast_write_pin(DIR_MOTOR_R, HIGH);
  // }/*else if (dir == -1 && LR = 2){//right-reverse
    //fast_write_pin(DIR_MOTOR_L, LOW);
    // fast_write_pin(DIR_MOTOR_R, HIGH);
    } else{
    analogWrite(SPEED_MOTOR_L, 0);
    analogWrite(SPEED_MOTOR_R, 0);
  }
}
//==============================================================================================

void turnL(int speed){//left-forward
  fast_write_pin(DIR_MOTOR_L, 0);
  analogWrite(SPEED_MOTOR_R, speed);
  fast_write_pin(DIR_MOTOR_R, LOW);
}

void turnR(int speed){//right-forward
  fast_write_pin(DIR_MOTOR_L, HIGH);
  analogWrite(SPEED_MOTOR_L, speed);
  fast_write_pin(DIR_MOTOR_R, 0);
}


/**  Function to make the robot travel for a certain amount of encoder ticks, calls upon setMotors at end
    @params dir - setPoint: The target value for how far we want to go (in encoder ticks)
    @params speed - analogWrite() value between 0-255
    @params kp - proportional gain, this is the main one you should be changing
    @params ki - intergral gain, use this for steady state errors
    @params kd - derivative gain, use this for overshoot and oscillation handling 
**/
void motorPID(int setPoint, float kp, float ki, float kd){
  int currentTime = micros();
  int deltaT = ((float)(currentTime - prevTime)) / 1.0e6; // time difference between ticks in seconds
  prevTime = currentTime; // update prevTime each loop 
  
  int error = setPoint - rightEncoderPos;
  int errorDerivative = (error - prevError) / deltaT;
  errorIntegral = errorIntegral + error*deltaT;

  float u = kp*error + ki*errorIntegral + kd*errorDerivative; 

  float speed = fabs(u);
  if(speed > 255){
    speed = 255;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1; // Move backward
  } else {
    dir = 1; // Move forward
  }

  int LR = 0; //just setting it to 0 as base start

  // setMotors(dir, speed, LR);
  setMotors(dir, speed);
  prevError = 0;
}
//==============================================================================================




//==============================================================================================
// Function to convert from encoder ticks to centimeters!
int tickConvertToCm(int encoderTicks){
  // Convert encoder ticks to centimeters
  // Define your conversion factor based on your measurements
  float ticksPerCm = 3.0; // For example, if 10 encoder ticks correspond to 1 cm
  
  // Calculate distance traveled in centimeters
  float distanceCm = encoderTicks / ticksPerCm;
  
  return distanceCm;
}
//==============================================================================================


void loop() {
   // I have to set up the sensors
  R = digitalRead(SR);
  M = digitalRead(SM);
  L = digitalRead(SL);

  int dipSwitch = analogRead(DIP_SWITCH);
  if (dipSwitch > 1000) {
      delay(500);
      // Call function to detect wall and rotate
      detectWallAndRotate();
  } else {
    //forward
    if (L != 0 && M == 0 && R != 0){
      setMotors(1, 80);
    }//reverse
    if (L != 0 && M != 0 && R != 0){
      setMotors(-1, 80);
    }//ONLY left
    if (L == 0 && M != 0 && R != 0){
      turnL(80);
    }//ONLY right
    if (L != 0 && M != 0 && R == 0){
      turnR(80);
    }
      // Implement your PID control or other navigation logic here
  }

 

/* digitalRead() for 0 or 1 - WHITE is 0 and BLACK is 1
  although the phototransistors are analogue pins [A0, A1, A2] we want the easier 
  0/1 digital values for starting out
  !! can do testing later if bothered for analogue values!! */

  /**TRUTH CASES**///basic lot just to have it move forward and turn functionalities
  //forward
  // if (L != 0 && M == 0 && R != 0){
  //   setMotors(1, 80);
  // }//reverse
  // if (L != 0 && M != 0 && R != 0){
  //   setMotors(-1, 80);
  // }//ONLY left
  // if (L == 0 && M != 0 && R != 0){
  //   turnL(80);
  // }//ONLY right
  // if (L != 0 && M != 0 && R == 0){
  //   turnR(80);
  // }

  //printing sensor values in order R M L
  // Serial.print(R);
  // Serial.print("\t");
  // Serial.print(M);
  // Serial.print("\t");
  // Serial.println(L);

  /*not sure why Zak put this in the starter code*/
  digitalWrite(EMITTERS, HIGH);
  Serial.println(analogRead(L));
  Serial.print("\t");
  Serial.println(analogRead(M));
  Serial.print("\t");
  Serial.println(analogRead(R));
  //=============================================

  

  delay(10);

}
