#include<Arduino.h>

#define S_WHITE 0
#define S_BLACK 1

#define C_WHITE 0
#define C_GRAY 1
#define C_BLACK 2

#define QUEUE_LEN_MAX 50
#define QUEUE_LEN_DEFAULT 20

#define seconds(s) (s*200000/3)

const int IR_A0 = A0; 
const int IR_A1 = A1; 
const int IR_A2 = A2; 
const int IR_A3 = A3; 
const int motor1_in1 = 9;
const int motor1_in2 = 8;
const int motor2_in1 = 7;
const int motor2_in2 = 6;
const int motor1_pwm = 10;
const int motor2_pwm = 5;

const int threshold_low[4] = {500, 400, 100, 85}, threshold_high[4] = {850, 850, 300, 300};

const int maxSpeed = 255; 

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class Queue{
  public:
    int* q;
    int capacity, size;
    int head;

    Queue(int _capacity = QUEUE_LEN_DEFAULT){
      if(_capacity>QUEUE_LEN_MAX) _capacity=QUEUE_LEN_MAX;
      q = new int[_capacity];
      capacity = _capacity;
      size=0;
      head=0;
    }
    ~Queue(){
      delete[] q;
    }
    void enqueue(int item){
      if(size==capacity){
        q[head]=item;
        head = (head+1)%capacity;
      }else{
        q[(head+size)%capacity]=item;
        size++;
      } 
    }
    void dequeue(){
      if(size==0) return;
      head=(head+1)%capacity;
      size--;
    }
    int avg(){
      if(size==0) return 0;
      int sum = 0;
      for(int i=0; i<size; i++) sum+=q[(head+i)%capacity];
      return (int)(sum/((double)size) + 0.5); //rounded
    }
    int peek_last(){
      if(size==0) return -1;
      return q[(head+size-1)%capacity];
    }
};


class Motor{
  public:
    int in1, in2, pwm;
    bool dir;
    Motor(int in1, int in2, int pwm, bool dir=0): in1(in1), in2(in2), pwm(pwm), dir(dir) {} 
    void setPinMode(void){
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(pwm, OUTPUT);
    }
    void setSpeed(int speed){
      speed=constrain(speed, -255, 255);
      if(dir) speed*=-1;
      if(speed==0){
        digitalWrite(pwm, HIGH); // fast stop
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return;
      }
      if(speed>0){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
      }else{
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      }
      analogWrite(pwm, abs(speed));
    }
    void brake(float s=0.3){
      setSpeed(-maxSpeed);
      delay(seconds(s));
      setSpeed(0);
    }
};

Motor motorL(motor1_in1, motor1_in2, motor1_pwm, 1), motorR(motor2_in1, motor2_in2, motor2_pwm, 0);

int irPins[4]={IR_A0, IR_A1, IR_A2, IR_A3};

// might need initialization
Queue irValuesRaw[4], irValues[4];

// A correction value, based on the error from target. 
// It is used to change the relative motor speed with PWM.
int correction = 0; 
int correction_scaled =0;
int previous_correction = 0; 

int status;
int stop_count;


void betterPWMSetup(){
  /* 
  Optional change for better PWM control of the DC motors below.
  
  Change the PWM frequency of digital pins 5 and 6 (timer0) to Phase-correct 
  PWM of 31.250 kHz from the default of ~500Hz. Using code from "Adjusting PWM 
  Frequencies http://playground.arduino.cc/Main/TimerPWMCheatsheet".
  
  This requires a separate change in the wiring.c function in the Arduino 
  program file hardware\arduino\cores\arduino\wiring.c from:
  #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
  
  to:
  #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(1 * 510))
  
  Without the change to wiring.c time functions (millis, delay, as well as 
  libraries using them will not work corectly.
  */
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS00);
}


int convert_raw_to_color(int raw, int index){
  if(raw<threshold_low[index]) 
    return C_WHITE;
  else if(raw<threshold_high[index])
    return C_GRAY;
  else
    return C_BLACK;
}


void Scan(){
  int raw, color;
  for(int i=0;i<4;i++){
    raw =analogRead(irPins[i]);
    color = convert_raw_to_color(raw, i);
    irValuesRaw[i].enqueue(raw);
    irValues[i].enqueue(color);
    
    Serial.print(raw);
    Serial.print("(");
    if(color==C_WHITE) Serial.print("W");
    if(color==C_GRAY) Serial.print("G");
    if(color==C_BLACK) Serial.print("B");
    Serial.print(")\t");
  }
  Serial.println();
}

uint8_t isGrayBin(bool from_raw=false){
  uint8_t bin = (uint8_t)0;
  int color;
  for(int i=0;i<4;i++){
    color=from_raw?convert_raw_to_color(irValuesRaw[i].avg(), i):irValues[i].avg();
    bin|=(color == C_GRAY)<<(4-i-1);
  }
  return bin;
} 

void UpdateCorrection() {
  uint8_t isGray=isGrayBin();
  
  switch(isGray){
    case 0b1111:
      break;
    case 0b1110: case 0b0111:
      correction = 255;
      break;
    case 0b1100: case 0b0011:
      correction = 200;
      break;
    case 0b1000: case 0b0001:
      correction = 130;
      break;
    case 0b1101: case 0b1011:
      correction = 0;
      break;
    case 0b1001:
      correction = 0;
      break;
    default:
      break;
  }
  
  if((isGray&0b0001)+((isGray&0b0010)>>1) > ((isGray&0b0100)>>2)+((isGray&0b1000)>>3))
    correction *= -1;
  correction_scaled = correction;
  correction_scaled = (int)((double)correction_scaled * maxSpeed / 255 + 0.5f);
} 

void Brake(float s=0.3){
  motorR.setSpeed(-255);
  motorL.setSpeed(-255);
  delay(seconds(s));
  motorR.setSpeed(0);
  motorL.setSpeed(0);
}

void StripeStop(){
  int black_count=0, white_count=0;
  for(int i=0;i<4;i++){
    white_count+=!(irValues[i].avg() ^ C_WHITE);
    black_count+=!(irValues[i].avg() ^ C_BLACK);
  }
  if(white_count > black_count && status!=S_WHITE) stop_count++, status=S_WHITE;
  if(white_count < black_count && status!=S_BLACK) stop_count++, status=S_BLACK;

  if(stop_count>=20){
    stop_count=0;
    Brake(0.5);
    delay(seconds(3)); // 3 sec
    motorR.setSpeed(255); //!
    motorL.setSpeed(255); //!
    delay(seconds(0.3));
  }
}

void Steer(){
  if(0 && sgn(previous_correction)!=sgn(correction)){
    if(correction>0) Serial.println("turn right");
    if(correction<0) Serial.println("turn left");
    if(correction==0) Serial.println("go straight");
  }
  if(correction_scaled==0){
    motorR.setSpeed(maxSpeed);
    motorL.setSpeed(maxSpeed);
  }else if (correction_scaled > 0) {
    // turn right
    motorR.setSpeed(maxSpeed-correction_scaled);
    motorL.setSpeed(maxSpeed);
  }else{
    // turn left
    motorR.setSpeed(maxSpeed);
    motorL.setSpeed(maxSpeed-correction_scaled);
  }

  previous_correction = correction;
}


void setup() {
  betterPWMSetup();  
  for(int i=0; i<4; i++) pinMode(irPins[i], INPUT);
  motorL.setPinMode(), motorR.setPinMode();
  motorL.setSpeed(0), motorR.setSpeed(0);
  status = S_WHITE;
  stop_count=0;

  Serial.begin(230400);
}

void loop(){
  Scan();
  StripeStop();
  UpdateCorrection();
  Steer();
  delay(seconds(0.0001));
}



