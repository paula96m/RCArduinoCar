#include <Servo.h>
#include <NewPing.h>
Servo steering;

#define IR_PIN_LEFT 2
#define IR_PIN_CENTER 3
#define IR_PIN_RIGHT 4
unsigned char IR_LEFT, IR_CENTER, IR_RIGHT, IR_TOTAL, LF_STARTED = 0;

// 0: BLACK, 1: WHITE
// L C R

#define FULL_BLACK_STOP       0b000 
#define STEER_RIGHT_SOFT      0b001
#define NOT_PLAUSIBLE_STOP    0b010
#define STEER_RIGHT_HARD      0b011
#define STEER_LEFT_SOFT       0b100
#define NO_STEERING           0b101
#define STEER_LEFT_HARD       0b110
#define FULL_WHITE_STOP       0b111

#define center 80
#define left_soft 60
#define left_hard 50
#define right_soft 100
#define right_hard 130
#define speed_lf 
unsigned char LF_LAST_STEER = center, full_white_or_black_stop_counter = 0;

#define trigPin 12
#define echoPin 11
float duration, distance;

int buzzerPin = 9; //Define buzzerPin
int need_beep = 0;
int beep_counter = 0;

#define SERIAL_BAUDRATE 9600  // bps

int Battery_1 = A0;   
int Battery_2 = A1;   
int Battery_3 = A2;   
int digitalVal_b1 = 0;  // variable to store the value
int digitalVal_b2 = 0;  // variable to store the value
int digitalVal_b3 = 0;  // variable to store the value
float Battery1_voltage = 0.00;
float Battery2_voltage = 0.00;
float Battery3_voltage = 0.00;
int procent1 = 0;
int procent2 = 0;
int procent3 = 0;

// serial commands
#define CMD_STOP 10
#define CMD_DRIVE 11

// steering
#define STEERING_PIN 13

#define STEERING_MIN_POSITION 50  // turning maximum left
#define STEERING_MAX_POSITION 130  // turning maximum right
#define STEERING_CENTER 80
#define STEERING_MIN_POSITION_CMD 0  // turning maximum left
#define STEERING_MAX_POSITION_CMD 100  // turning maximum right
#define STEERING_CENTER_CMD 50

// traction
#define TRACTION_PHASE_PIN 8
#define TRACTION_ENABLE_PIN 6

#define DIRECTION_FORWARD LOW
#define DIRECTION_BACKWARD HIGH
unsigned char cmd = 0 ;
unsigned char data_0 = 0;
unsigned char data_1 = 0;


unsigned char battery_counter = 0;

void setup() 
{
    pinMode(TRACTION_PHASE_PIN, OUTPUT);
    pinMode(TRACTION_ENABLE_PIN, OUTPUT);
    pinMode(STEERING_PIN, OUTPUT);
    digitalWrite(TRACTION_PHASE_PIN, LOW);
    digitalWrite(TRACTION_ENABLE_PIN, LOW);
    digitalWrite(STEERING_PIN, LOW);

    steering.attach(STEERING_PIN);
    center_steering();
  
    init_serial_interface();
    clear_serial_buffer();

    pinMode(IR_PIN_LEFT, INPUT);
    pinMode(IR_PIN_CENTER, INPUT);
    pinMode(IR_PIN_RIGHT, INPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    pinMode(buzzerPin, OUTPUT); //Set buzzerPin as output
}


void loop() 
{
  listen_serial();
  delay(25);
  beep();
//  battery_counter ++;
//  if(battery_counter >= 40)
//  {
//    Battery();
//    battery_counter = 0;
//    sendData();
//  }
  
  
  //lineFollower();
}

void Obstacle_Detect()
{
  // Write a pulse to the HC-SR04 Trigger Pin
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the response from the HC-SR04 Echo Pin
 
  duration = pulseIn(echoPin, HIGH);
  
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  
  distance = (duration / 2) * 0.0343;
  
  // Send results to Serial Monitor

 // Serial.print("Distance = ");
//  if (distance >= 400 || distance <= 2) {
//     Serial.println("Out of range");
//  }
//  else {
//    Serial.print(distance);
//    Serial.println(" cm");
//    delay(500);
//  }

   if (distance <= 15) 
    {
      
      if((data_0 % 2) == 0)
      {
        car_stop();
        
      }
    }
  
  
}
void beep()
{ 
  Obstacle_Detect();
  if(distance < 30 && distance > 15)
  {
    need_beep = 1;
    
  }
  else
  {
    need_beep = 0;
    
  }
  if(need_beep == 1)
  {
    if((beep_counter % 2) == 1)
    {
      analogWrite(buzzerPin, 20); //Setting pin to low
      analogWrite(buzzerPin, 0); //Setting pin to low
    }
    else
    {
      analogWrite(buzzerPin, 0); //Setting pin to low
    }
    beep_counter++;
  }
  else
  {
      analogWrite(buzzerPin, 0); //Setting pin to low
  }
}

void Battery()
{
  digitalVal_b1 = analogRead(A0);// read the value from the analog channel
  //convert digital value to analog voltage
  Battery1_voltage = (digitalVal_b1 * 5.00)/1023.00;
  procent1 = (Battery1_voltage /4.2)*100;
    //battery 2
  digitalVal_b2 = analogRead(Battery_2);// read the value from the analog channel
  //convert digital value to analog voltage
  Battery2_voltage = (digitalVal_b2 * 5.00)/1023.00;
  procent2 = (Battery2_voltage /4.2)*100;
  //battery 3
  digitalVal_b3 = analogRead(Battery_3);// read the value from the analog channel
  //convert digital value to analog voltage
  Battery3_voltage = (digitalVal_b3 * 5.00)/1023.00;
  procent3 = (Battery3_voltage /4.2)*100;
  
}
void sendData()
{
 
  Serial.write(procent1);
  Serial.write(procent2);
  Serial.write(procent3);
 
  
  
}
void lineFollower()
{
  if(!LF_STARTED)
  {
    LF_STARTED = 1;
    car_forward(140);
  }
  IR_LEFT = digitalRead(IR_PIN_LEFT);
  IR_CENTER = digitalRead(IR_PIN_CENTER);
  IR_RIGHT = digitalRead(IR_PIN_RIGHT);

  IR_TOTAL = 0;
  IR_TOTAL = IR_LEFT;
  IR_TOTAL <<= 1;
  IR_TOTAL |= IR_CENTER;
  IR_TOTAL <<= 1;
  IR_TOTAL |= IR_RIGHT;

  switch(IR_TOTAL)
  {
    case FULL_BLACK_STOP:
    
    {
      full_white_or_black_stop_counter ++;
      if(full_white_or_black_stop_counter > 100)
      {
        car_stop();
      }
      else
      {
        if(LF_LAST_STEER == left_hard)
        {
          
        }
        else if(LF_LAST_STEER == right_hard)
        {
          
        }
        else
        {
          car_stop();
        }
      }
      break;
    }
    case STEER_RIGHT_SOFT:
    {
      steering.write(right_soft);
      full_white_or_black_stop_counter = 0;
      LF_LAST_STEER = right_soft;
      break;
    }
    case NOT_PLAUSIBLE_STOP:
    {
      car_stop();
      break;
    }
    case STEER_RIGHT_HARD:
    {
      steering.write(right_hard);
      full_white_or_black_stop_counter = 0;
      LF_LAST_STEER = right_hard;
      break;
    }
    case STEER_LEFT_SOFT:
    {
      steering.write(left_soft);
      
      full_white_or_black_stop_counter = 0;
      LF_LAST_STEER = left_soft;
      break;
    }
    case NO_STEERING:
    {
      steering.write(center);
      
      full_white_or_black_stop_counter = 0;
      LF_LAST_STEER = center;
      break;
    }
    case STEER_LEFT_HARD:
    {
      steering.write(left_hard);
      full_white_or_black_stop_counter = 0;
      LF_LAST_STEER = left_hard;
      break;
    }
    case FULL_WHITE_STOP:
    {
      full_white_or_black_stop_counter ++;
      if(full_white_or_black_stop_counter > 100)
      {
        car_stop();
      }
      else
      {
        if(LF_LAST_STEER == left_hard)
        {
          
        }
        else if(LF_LAST_STEER == right_hard)
        {
          
        }
        else
        {
          car_stop();
        }
      }
      break;
    }
    default:
    {
      car_stop();
      break;
    }
  }
  

}

void init_serial_interface(void)
{
  Serial.begin(SERIAL_BAUDRATE);  // opens serial port, sets baudrate to 9600 bps
}

void clear_serial_buffer(void)
{
  while(Serial.available() > 0)
  {
    Serial.read();
  }
}

void listen_serial(void)
{
  if(Serial.available() != 3)
  {
    clear_serial_buffer();
    return;
  }
  else
  {
    cmd = Serial.read();
    data_0 = Serial.read();
    data_1 = Serial.read();
    execute_command(cmd, data_0, data_1);
    return;
  }
}

void execute_command(unsigned char cmd, unsigned char data_0, unsigned char data_1)
{
  switch(cmd)
  {
    case CMD_STOP:
    {
      car_stop();
      break;
    }
    case CMD_DRIVE:
    {
      car_drive_cmd(data_0, data_1);
      break;
    }
  }
}

void car_drive_cmd(unsigned char traction_speed, unsigned char steering_position)
{
  if(traction_speed == 0)
  {
    car_stop();
    return;
  }

  if(steering_position == STEERING_CENTER_CMD)
  {
    center_steering();
  }
  else
  {
    unsigned char steering_real_position = steering_cmd_to_real(steering_position);
    car_steer(steering_real_position);
  }
  
  unsigned char traction_direction = traction_speed & 0x01;
  if(traction_direction == DIRECTION_FORWARD)
  {
    car_forward(traction_speed);
  }
  else
  {
    car_backward(traction_speed);
  }
}

void car_stop(void)
{
  digitalWrite(TRACTION_PHASE_PIN, LOW);
  analogWrite(TRACTION_ENABLE_PIN, 0);
}

void car_forward(unsigned char speed)
{
  digitalWrite(TRACTION_PHASE_PIN, DIRECTION_FORWARD);
  analogWrite(TRACTION_ENABLE_PIN, speed);
}

void car_backward(unsigned char speed)
{
  digitalWrite(TRACTION_PHASE_PIN, DIRECTION_BACKWARD);
  analogWrite(TRACTION_ENABLE_PIN, speed);
}

void center_steering(void)
{
  steering.write(STEERING_CENTER);
}

void car_steer(unsigned char position)
{
  if(position < STEERING_MIN_POSITION)
  {
    steering.write(STEERING_MIN_POSITION);
    return;
  }
  else if (position > STEERING_MAX_POSITION)
  {
    steering.write(STEERING_MAX_POSITION);
    return;
  }
  else
  {
    steering.write(position);
    return;
  }
}

int steering_cmd_to_real(unsigned char steering_cmd)
{
  // [A, B] --> [a, b]
  int value = steering_cmd;
  int A, B, a, b;
  int real_value;
  if(value <= STEERING_CENTER_CMD)
  {
    A = STEERING_MIN_POSITION_CMD;
    B = STEERING_CENTER_CMD;
    a = STEERING_MIN_POSITION;
    b = STEERING_CENTER;
  }
  else
  {
    A = STEERING_CENTER_CMD;
    B = STEERING_MAX_POSITION_CMD;
    a = STEERING_CENTER;
    b = STEERING_MAX_POSITION;
  }
  real_value = (value - A) * (b - a) / (B - A) + a;
  return real_value;
}
