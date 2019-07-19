#include <Servo.h>
#include <NewPing.h>
Servo steering;
Servo fancy_buzzer;

#define IR_PIN_LEFT 2
#define IR_PIN_CENTER 3
#define IR_PIN_RIGHT 4
unsigned char IR_LEFT, IR_CENTER, IR_RIGHT, IR_TOTAL, LF_STARTED = 0;
#define LFEP A5

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
#define left_hard 52 
#define right_soft 100
#define right_hard 130
#define speed_lf 
unsigned char LF_LAST_STEER = center, full_white_or_black_stop_counter = 0;

#define NO_CMD_RECEIVED_STOP_THRESHOLD 4  // stop car after # loops with no cmd received from phone

#define trigPin1 12
#define echoPin1 11
#define trigPin2 7
#define echoPin2 5
float duration1, distance1, duration2, distance2;


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

char CAR_STOPPED = 0;
char no_cmd_received_counter = 0;
// serial commands
#define CMD_STOP 10
#define CMD_DRIVE 11

// steering
#define STEERING_PIN 13

// valori de steering reale, specifice servo motorului
#define STEERING_MIN_POSITION 50  // turning maximum left
#define STEERING_MAX_POSITION 130  // turning maximum right
#define STEERING_CENTER 80

// valori de steering ideale primite de la telefon
#define STEERING_MIN_POSITION_CMD 0  // turning maximum left
#define STEERING_MAX_POSITION_CMD 100  // turning maximum right
#define STEERING_CENTER_CMD 50

#define BUZZER_PIN 10

// traction
#define TRACTION_PHASE_PIN 8
#define TRACTION_ENABLE_PIN 6

#define DIRECTION_FORWARD LOW
#define DIRECTION_BACKWARD HIGH
unsigned char cmd = 0 ;
unsigned char data_0 = 0;
unsigned char data_1 = 0;

unsigned char ALARM_FWD_ON = 0;
unsigned char ALARM_BKWD_ON = 0;

unsigned char FIRST_TIME_ALARM_FWD_TRIGGERED = 0;
unsigned char FIRST_TIME_ALARM_BKWD_TRIGGERED = 0;

unsigned char alarm_counter = 0;
#define ALARM_THRESHOLD 5
unsigned char battery_counter = 0;


unsigned char DRIVE_FWD_ALLOWED = 1;
unsigned char DRIVE_BKWD_ALLOWED = 1;

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
    
    pinMode(LFEP, INPUT_PULLUP);

    //pinMode(9, OUTPUT);
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
     pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    //pinMode(buzzerPin, OUTPUT); //Set buzzerPin as output

}

unsigned char x = 0;
void loop() 
{                                                                                                                                                                                           

  if(digitalRead(LFEP))
  {
    Obstacle_Detect_Front();
    Obstacle_Detect_Back();
    listen_serial();
    CAR_STOPPED = 0;
    LF_STARTED = 0;
    battery_counter ++;
    if(battery_counter >= 40)
    {
      Battery();
      battery_counter = 0;
      sendData();
    }
  }
  else
  {
    if(CAR_STOPPED == 0)
    {
      lineFollower();
    }
  }

  if(ALARM_FWD_ON || ALARM_BKWD_ON)
  {
    alarm_counter ++;
    if(alarm_counter >= ALARM_THRESHOLD)
    {
      alarm_counter = 0;
      digitalWrite(BUZZER_PIN, HIGH);
    }
  }
  delay(23);
  digitalWrite(BUZZER_PIN, LOW);

}

void Obstacle_Detect_Front()
{
  // Write a pulse to the HC-SR04 Trigger Pin
  
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  
  // Measure the response from the HC-SR04 Echo Pin
 
  duration1 = pulseIn(echoPin1, HIGH);
  //duration2 = pulseIn(echoPin2, HIGH);
  
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  
  distance1 = (duration1 / 2) * 0.0343;
  if(distance1 <= 25 && distance1 > 15)
  {
      ALARM_FWD_ON = 1;
      FIRST_TIME_ALARM_FWD_TRIGGERED = 0;
  }
  else if (distance1 <= 15) 
  {
    if(!FIRST_TIME_ALARM_FWD_TRIGGERED)
    {
      FIRST_TIME_ALARM_FWD_TRIGGERED = 1;
      car_stop();
    }
    ALARM_FWD_ON = 0;
    DRIVE_FWD_ALLOWED = 0;
  }
  else
  {
    ALARM_FWD_ON = 0;
    DRIVE_FWD_ALLOWED = 1;
    FIRST_TIME_ALARM_FWD_TRIGGERED = 0;
  }
}

void Obstacle_Detect_Back()
{
  // Write a pulse to the HC-SR04 Trigger Pin
  
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  
  // Measure the response from the HC-SR04 Echo Pin
 
  duration2 = pulseIn(echoPin2, HIGH);
  
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  
  distance2 = (duration2 / 2) * 0.0343;
  if(distance2 <= 25 && distance2 > 15)
  {
      FIRST_TIME_ALARM_BKWD_TRIGGERED = 0;
      ALARM_BKWD_ON = 1;
  }
  else if (distance2 <= 15) 
  {
    if(!FIRST_TIME_ALARM_BKWD_TRIGGERED)
    {
      FIRST_TIME_ALARM_BKWD_TRIGGERED = 1;
      car_stop();
    }
    ALARM_BKWD_ON = 0;
    DRIVE_BKWD_ALLOWED = 0;
  }
  else
  {
    ALARM_BKWD_ON = 0;
    DRIVE_BKWD_ALLOWED = 1;
    FIRST_TIME_ALARM_BKWD_TRIGGERED = 0;
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
        LF_STARTED = 0;
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
          LF_STARTED = 0;
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
      LF_STARTED = 0;
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
        LF_STARTED = 0;
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
          LF_STARTED = 0;
        }
      }
      break;
    }
    default:
    {
      car_stop();
      LF_STARTED = 0;
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
  if(Serial.available() == 0)
  {
      no_cmd_received_counter ++;
      if(no_cmd_received_counter > NO_CMD_RECEIVED_STOP_THRESHOLD)
      {
          no_cmd_received_counter = 0;
          car_stop();
      }
  }
  
  if(Serial.available() != 3)
  {
    clear_serial_buffer();
    no_cmd_received_counter = 0;
    return;
  }
  else
  {
    no_cmd_received_counter = 0;
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
    if(DRIVE_FWD_ALLOWED)
    {
      car_forward(traction_speed);
    }
    else
    {
      car_stop();
    }
  }
  else
  {
    if(DRIVE_BKWD_ALLOWED)
    {
      car_backward(traction_speed);
    }
    else
    {
      car_stop();
    }
  }
}

void car_stop(void)
{
  digitalWrite(TRACTION_PHASE_PIN, LOW);
  analogWrite(TRACTION_ENABLE_PIN, 0);
  CAR_STOPPED = 1;
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
  // se transforma intervalul [0, 100] primit de la telefon in intervalul [50, 130], specific pentru servo motor
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
