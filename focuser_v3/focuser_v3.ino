/*
 * Implementation of the moonlite protocol for a NEMA stepper or 28BYJ-48.
 * Attempt at following the specs froms
 * http://www.indilib.org/media/kunena/attachments/1/HighResSteppermotor107.pdf
 * (C) Aris Adamantiadis 2023-
 */

// #define NEMA

#ifdef NEMA
const int pin_dir = 9;
const int pin_step = 8;
const int pin_sleep = 7;
const int pin_reset = 6;
const int pin_m2 = 5;
const int pin_m1 = 4;
const int pin_m0 = 3;

const int pin_enable = 2;
const int pin_fault = 10;

const int gpio1 = 13;
const int gpio2 = 12;
const int gpio3 = 11;

// Mode means half/quarter/eights/... steps.
const int mode = 4; //0 - 5
//const int stepsPerRevolution = 200 * (1 << mode);
const int stepsPerRevolution = 2;

const int stepSize = 1; //1 for mode=5,2 for mode=4, 4 for mode=3, ...
const uint8_t out_pins[] = { pin_dir, pin_step, pin_sleep, pin_reset, pin_enable, pin_m2, pin_m1, pin_m0 };

#else

#include <Stepper.h>
const int stepsPerRevolution = 200;
const int pin1 = 5;
const int pin2 = 3;
const int pin3 = 4;
const int pin4 = 2;
int backlash=18;
Stepper myStepper(stepsPerRevolution, pin1, pin2, pin3, pin4);
int current_direction = 0;
const int stepSize = 10;
const int speedtable[]={10,20,30,40,50};
#endif

/* the stepper's position */
uint16_t position = 32000;
uint16_t new_position = position;
uint8_t speed = 0x20;
uint8_t half_step = 0;
bool moving = false;
//#define DEBUG


void setup() {
#ifdef NEMA
  for (int i = 0; i < (sizeof(out_pins) / sizeof(out_pins[0])); ++i) {
    pinMode(out_pins[i], OUTPUT);
  }
  pinMode(pin_fault, INPUT);
  digitalWrite(pin_sleep, HIGH);
  digitalWrite(pin_reset, HIGH);
  digitalWrite(pin_m2, mode & 0x4);
  digitalWrite(pin_m1, mode & 0x2);
  digitalWrite(pin_m0, mode & 0x1);
  digitalWrite(pin_enable, HIGH);

  // Shared with internal led, disabled
  //pinMode(gpio1, INPUT_PULLUP);
  pinMode(gpio2, INPUT_PULLUP);
  pinMode(gpio3, INPUT_PULLUP);
#else
  //myStepper.setSpeed(10);
  set_speed();
#endif
  Serial.begin(115200);
  //blink(10);
}

void blink(int n) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i < n; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    if (i != n - 1) {
      delay(50);
    }
  }
  // put back led 13 in input mode
  //pinMode(gpio1, INPUT_PULLUP);
}

/** read the value of a pin, assuming pull-up, and wait until the key is unpressed
    @returns 1 if the key is pressed.
*/
int read_rebound(int pin) {
  if (digitalRead(pin) == LOW) {
    while (1) {
      if (digitalRead(pin) == HIGH) {
        return 1;
      }
    }
  } else {
    return 0;
  }
}

static void hold(){
#ifdef ENABLE
  digitalWrite(pin_enable, LOW);
#endif
}

/* It's important to disable the stepped when not in use as it's drawing a
 *  considerable amount of current in idle position. Only for NEMA
 */
static void release(){
#ifdef NEMA
  digitalWrite(pin_enable, HIGH);
#else
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);
#endif
}

void move() {
  int dir;
  if (new_position == position){
    moving = false;
    return;
  }
  if(new_position > position){
    dir = 1;
  } else {
    dir = 0;
  }
#ifdef NEMA 
  digitalWrite(pin_dir, dir);
  hold();
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(pin_step, HIGH);  
  delay(1);
  digitalWrite(pin_step, LOW);
  delay(1);
  digitalWrite(LED_BUILTIN, LOW);
  if (dir){
    position += stepSize;
  } else {
    position -= stepSize;
  }
#else

  if(dir !=  current_direction){
    // We have to fix backlash first
    if(dir){
      myStepper.step(backlash);
    } else {
      myStepper.step(-backlash);
    }
    current_direction = dir;
  }
  int step = new_position - position;
  if (step > 0 && step > stepSize){
    step = stepSize;
  } else if(step < 0 && -step > stepSize){
    step = -stepSize;
  } 
  myStepper.step(step);
  position += step;
  delay(1);
#endif

}

void set_speed(){
  // one of 0x02, 0x04, 0x08, 0x10, 0x20
  int index=0;
  switch(speed){
    case 0x2:
      index=0;
      break;
    case 0x4:
      index=1;
      break;
    case 0x8:
      index=2;
      break;
    case 0x10:
      index=3;
      break;
    case 0x20:
      index=4;
      break;
    default:
      index=0;
  }
#ifdef NEMA
// not implemented
#else
  myStepper.setSpeed(speedtable[index]);
#endif
}

void readcmd() {
  int inbyte;
  char buffer[10]="";
  int i=0;
  int len;
  if (Serial.available() <= 0){
    return;
  }
  while(Serial.read() != ':')
    ;
  memset(buffer, 0, sizeof(buffer));
  for(i=0; i<sizeof(buffer)-1;++i){
    while(Serial.available() <= 0)
      ;
    buffer[i]=Serial.read();
    if(buffer[i]=='#'){
      buffer[i] = 0;
      len=i;
      break;
    }
  }
#ifdef DEBUG
  Serial.print("Received: ");
  Serial.println(buffer);
#endif

  if (len < 1)
    return;
  
  switch(buffer[0]){
    case 'C':
      // Initiate temperature conversion
    case '+':
      // Activate temperature compensation focusing
    case '-':
      // Disable temperature compensation focusing
      return;
  }
  
  if(len < 2){
    Serial.print("Command too short(");
    Serial.print(len);
    Serial.print("): ");
    Serial.println(buffer);
    return;
  }
  switch(buffer[0]){
    case 'P':
      switch(buffer[1]){
        case 'O':
          // adjust temperature offset
        case 'S':
          // adjust temperature scale
        case 'R':
          // adjust red backlight
        case 'G':
          // adjust green backlight
        case 'B':
          // adjust blue backlight
        case 'C':
          // adjust LCD contrast
        case 'X':
          // adjust scale for motor 1
        case 'y':
          // adjust scale for motor 2
        case 'H':
          // find home for motor
          // no action
          break;
        default:
          goto invalidcmd;
      }
      break;
    case 'G':
      switch(buffer[1]){
        case 'P':
          // Get current motor 1 position
          sprintf(buffer, "%.4X#", position);
          Serial.print(buffer);
          break;
        case 'N':
          // Get the New Motor 1 Position, Unsigned Hexadecimal
          sprintf(buffer, "%.4X#", new_position);
          Serial.print(buffer);
          break;
        case 'T':
          // Get the Current Temperature, Signed Hexadecimal
          Serial.print("00#");
          break;
        case 'D':
          // Get the Motor 1 speed, valid options are “02, 04, 08, 10, 20”
          sprintf(buffer, "%.2X#", speed);
          Serial.print(buffer);
          break;
        case 'H':
          // “FF” if half step is set, otherwise “00”
          if (half_step){
            Serial.print("FF#");
          } else {
            Serial.print("00#");
          }
          break;
        case 'I':
          // “01” if the motor is moving, otherwise “00”
          if (moving)
            Serial.print("01#");
          else
            Serial.print("00#");
          break;
        case 'B':
          // The current RED Led Backlight value, Unsigned Hexadecimal
          Serial.print("00#");
          break;
        case 'V':
          // Code for current firmware version
          Serial.print("42#");
          break;
        default:
          goto invalidcmd;
      }
      break;
    case 'S':
      switch(buffer[1]){
        case 'P':
          // Set the Current Motor 1 Position, Unsigned Hexadecimal
          if(len < 6)
            goto invalidcmd;
          sscanf(&buffer[2], "%4hx", &position);
#ifdef DEBUG
          Serial.print("New position: ");
          Serial.println(position);
#endif
          break;
        case 'N':
          // Set the New Motor 1 Position, Unsigned Hexadecimal
          if(len < 6)
            goto invalidcmd;
          sscanf(&buffer[2], "%4hx", &new_position);
          break;
        case 'F':
          // Set Motor 1 to Full Step
          half_step = false;
          break;
        case 'H':
          // Set Motor 1 to Half Step
          half_step = true;
          break;
        case 'D':
          // Set the Motor 1 speed, valid options are “02, 04, 08, 10, 20”
          if(len < 4)
            goto invalidcmd;
          uint8_t tmp_speed=0xff;
          sscanf(&buffer[2], "%2hhx", &tmp_speed);
          if(tmp_speed==0x02 || tmp_speed == 0x04 || tmp_speed == 0x08 || tmp_speed == 0x10 || tmp_speed == 0x20){
            speed = tmp_speed;
            set_speed();
          } else {
            Serial.println("Invalid speed!");
          }
          break;
        default:
          goto invalidcmd;
      }
      break;
    case 'F':
      switch(buffer[1]){
        case 'G':
          // Start a Motor 1 move, moves the motor to the New Position.
          moving=true;
          break;
        case 'Q':
          // Halt Motor 1 move, position is retained, motor is stopped.
          moving=false;
          break;
        default:
          goto invalidcmd;
      }
      break;
    default:
      goto invalidcmd;
  }
  return;
  invalidcmd:
  Serial.print("unknown command: ");
  Serial.println(buffer);
}

void loop() {
  readcmd();
  if(moving){
    move();
  } else {
    release();
  }
}
