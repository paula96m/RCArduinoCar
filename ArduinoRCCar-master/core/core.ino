#include <Servo.h>

#define phase 8
#define en 9


#define DIRECTION_FORWARD LOW
#define DIRECTION_BACKWARD HIGH
#define STEERING_POWER 100

//#define STEERING_MIN_POSITION 46  // turning maximum left
//#define STEERING_MAX_POSITION 130  // turning maximum right
#define STEERING_CENTER 80

#define CMD_DIRECTION_FORWARD 0
#define CMD_DIRECTION_BACKWARD 1

unsigned char command;
Servo myservo;
int pos = 0;    // variable to store the servo position
void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pinMode(en, OUTPUT);
  pinMode(phase, OUTPUT);
  digitalWrite(en, LOW);
  digitalWrite(phase, LOW);
  center_steering();
 // myservo.attach(13);  // attaches the servo on pin 13 to the servo object
 
}

void Forward(int speed) 
{
  //digitalWrite(sleep, HIGH);
  digitalWrite(phase, DIRECTION_FORWARD);
  analogWrite(en, speed);
}


void Stop()
{
  //digitalWrite(sleep, LOW);
  digitalWrite(phase, LOW);
  analogWrite(en, 0);
  
}



void Backward(int speed) {
  //digitalWrite(sleep, HIGH);
  digitalWrite(phase, DIRECTION_BACKWARD);
  analogWrite(en, speed);
}

void drive(unsigned char command)
{
  // command structure
  // BIT7 BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0
  //  S    S    S    S    S    D    ST   ST
  // S  = SPEED
  // D  = DIRECTION
  // ST = STEERING
  
  unsigned char steering;
  unsigned char car_direction;
  unsigned char car_speed;
  
  steering = command & 0b00000011;  // extract steering
  car_direction = (command & 0b00000100) >> 2;  // extract direction
  car_speed = (command & 0b11111000) >> 3;  // extract speed

  car_speed *= 8;  // bring speed in range [0, 255]


    Serial.println(steering, DEC);
    Serial.println(car_direction, DEC);
    Serial.println(car_speed, DEC);

  if(car_speed == 0)
  {
    Stop();
    return;
  }

  

  switch(car_direction)
  {
    case CMD_DIRECTION_FORWARD:
    {
      Forward(car_speed);
      break;
    }
    case CMD_DIRECTION_BACKWARD:
    {
      Backward(car_speed);
      break;
    }
    default:
    {
      Stop();
      break;
    }
  }
 
}




void loop() 
{
  if (Serial.available() > 0) 
  {
    command = Serial.read(); 
    drive(command);
  }
    
}

void center_steering(void)
{
  myservo.write(STEERING_CENTER);
}
