

#include "CompositeVideo/CompositeGraphics.h"
#include "CompositeVideo/Image.h"
#include "CompositeVideo/CompositeOutput.h"

#include "CompositeVideo/luni.h"
#include "CompositeVideo/font6x8.h"


#include <soc/rtc.h>

//colors
#define WHITE 50
#define BLACK 0

//#include <Adafruit_SSD1306.h>
#define BUTTON0 34
#define BUTTON1 0
#define BUTTON2 35
//Adafruit_SSD1306 lcd(128, 64); // create display object

#include "IMU/I2Cdev.h"
#include "IMU/MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
uint8_t broadcastAddress[] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF};
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//PAL MAX, half: 324x268 full: 648x536
//NTSC MAX, half: 324x224 full: 648x448
const int XRES = 320;
const int YRES = 220;

//Graphics using the defined resolution for the backbuffer
CompositeGraphics graphics(XRES, YRES);
//Composite output using the desired mode (PAL/NTSC) and twice the resolution. 
//It will center the displayed image automatically
CompositeOutput composite(CompositeOutput::NTSC, XRES * 2, YRES * 2);

//image and font from the included headers created by the converter. Each iamge uses its own namespace.
Image<CompositeGraphics> luni0(luni::xres, luni::yres, luni::pixels);

//font is based on ASCII starting from char 32 (space), width end height of the monospace characters. 
//All characters are staored in an image vertically. Value 0 is background.
Font<CompositeGraphics> font(6, 8, font6x8::pixels);


void draw_logo() {
  //H
  graphics.fillRect(125, 70, 5, 29, WHITE);
  graphics.fillRect(130, 86, 10, 5, WHITE);
  graphics.fillRect(140, 70, 5, 29, WHITE);

  //T
  graphics.fillRect(156, 70, 20, 5, WHITE);
  graphics.fillRect(163, 75, 6, 24, WHITE);

  //3
  graphics.fillRect(188, 70, 12, 5, WHITE);
  graphics.fillRect(192, 82, 8, 5, WHITE);
  graphics.fillRect(188, 94, 12, 5, WHITE);
  graphics.fillRect(200, 70, 5, 29, WHITE);

  graphics.setCursor(125, 100);
  graphics.print("HAND TILT 3000");
}

int snake[300][2]; // snake coordinate array

int snake_size = 30; // initialize snake size to 10
int high_score = 0;

int apple[2] = {random(11, 309), random(11, 209)};

byte prev_state0 = 1, prev_state1 = 1, prev_state2 = 1;
int button_pressed = 0;
void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(188); //220
  mpu.setYGyroOffset(89); //76
  mpu.setZGyroOffset(7); //-85
  mpu.setZAccelOffset(1568); // 1688 factory default for my test chip // 1788

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      /*mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();*/
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  snake[0][0] = 64;
  snake[0][1] = 32;
  for (int i = 1 ; i < 300 ; i++) {
    snake[i][0] = 0;
    snake[i][1] = 0;
  }

  //highest clockspeed needed
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_240M);
  
  //initializing DMA buffers and I2S
  composite.init();
  //initializing graphics double buffer
  graphics.init();
  //select font
  graphics.setFont(font);

  //running composite output pinned to first core
  xTaskCreatePinnedToCore(compositeCore, "c", 1024, NULL, 1, NULL, 0);
  //rendering the actual graphics in the main loop is done on the second core by default
  
  // initialize buttons
  pinMode(BUTTON0, INPUT_PULLUP); 
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  
  
  graphics.begin(0);
  draw_logo();

  // snake option
  graphics.setCursor(136, 115);
  graphics.print("0   SNAKE");
  
  graphics.end();
  while (1) {
    byte curr_state = digitalRead(BUTTON0);
    if (prev_state0 == 1 && curr_state == 0) {
      break;
    }
    prev_state0 = curr_state;
    delay(10);
  }
  
}

void compositeCore(void *data)
{
  while (true)
  {
    //just send the graphics frontbuffer whithout any interruption 
    composite.sendFrameHalfResolution(&graphics.frame);
  }
}

// rolls over any out of bounds coordinate
void roll_over(int coord[2]) {

  if (coord[0] >= 320) {
    coord[0] = 0;
  } else if (coord[0] < 0) {
    coord[0] = 319;
  }
  if (coord [1] >= 220) {
    coord[1] = 0;
  } else if (coord[1] < 0) {
    coord[1] = 219;
  }
}


// max - 4 because its a square
void move_apple() {
  apple[0] = random(11, 309);
  apple[1] = random(11, 209);
  //apple = {random(0, 124), random(0, 60)};
}

bool hits_apple(int coord[2]) {
  return (coord[0] >= apple[0] && coord[0] <= apple[0]+10) && (coord[1] >= apple[1] && coord[1] <= apple[1]+10);
}


// returns true if the snake collides with itself
bool is_gameover(int coord[2]) {
  for (int i = 0 ; i < snake_size; i++) {
    if (snake[i][0] == coord[0] && snake[i][1] == coord[1]) {
      return true;
    }
  }
  return false;
}

// prints the players current score and high score
void print_score() {

  // print current score
  graphics.setTextColor(50);
  graphics.setCursor(10,0);
  graphics.print("SCORE: ");
  graphics.print(snake_size);

  // print high score
  graphics.setCursor(10, 10);
  graphics.print("HIGH SCORE: ");
  graphics.print(high_score);
}

void game_over_screen() {
  graphics.begin(0);
  graphics.end();
  graphics.begin(0);
  graphics.setCursor(140, 92);
  graphics.print("GAME OVER");
  graphics.setCursor(138, 100);
  graphics.print("SCORE: ");
  graphics.print(snake_size);
  graphics.setCursor(60, 108);
  graphics.print("PRESS TOP BUTTON TO GO BACK TO TITLE");
  graphics.end();
  snake_size = 10;
  while (1) {
    byte curr_state = digitalRead(BUTTON0);
    if (prev_state0 == 1 && curr_state == 0) {
      break;
    }
    prev_state0 = curr_state;
    delay(10);
  }
  setup();
}


int curr_direction = 0; // current direction the snake is going, 0 left 1 down 2 right 3 up
int imu_direction = 0; // direction read by imu
float p_curr = 0; // pitch value
float r_curr = 0; // roll value




// indicate imu direction based on pitch and roll
void direc_deter(float p,float r){
   if(p <= -30 && r < 30 && r > -30){imu_direction = 1;} // down
   if(p >=  30 && r < 30 && r > -30){imu_direction = 3;} // up
   if(r <= -30 && p < 30 && p > -30){imu_direction = 0;} //left
   if(r >=  30 && p < 30 && p > -30){imu_direction = 2;} // right
}

void loop() {
  
  // IMU

  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    mpu.resetFIFO();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if(fifoCount < packetSize){
            //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
        // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));
        return;

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      
        // read a packet from FIFO
        while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
        }
       #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            p_curr = ypr[1] * 180/M_PI;
            Serial.print(p_curr);
            Serial.print("\t");
            r_curr = ypr[2] * 180/M_PI;
            Serial.print(r_curr);
            Serial.print("\t");
            direc_deter(p_curr, r_curr);
            if(imu_direction == 1){Serial.print("DOWN");}
            else if(imu_direction == 3){Serial.print("UP");}
            else if(imu_direction == 0){Serial.print("LEFT");}
            else if(imu_direction == 2){Serial.print("RIGHT");}  
            
            //Serial.print(direc);
            Serial.print("\n");
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
  // wipe lcd and begin writing to the lcd
  graphics.begin(0);
  
  // draw apple
  graphics.fillRect(apple[0], apple[1], 10, 10, WHITE); 

//  // check if button 0 is pressed
//  byte curr_state = digitalRead(BUTTON0);
//  if (prev_state0 == 1 && curr_state == 0) {
//    button_pressed = 0;
//  }
//  prev_state0 = curr_state;
//
//  // check if button 1 is pressed
//  curr_state = digitalRead(BUTTON1);
//  if (prev_state1 == 1 && curr_state == 0) {
//    button_pressed = 1;
//  }
//  prev_state1 = curr_state;
//
//  // check if button 2 is pressed
//  curr_state = digitalRead(BUTTON2);
//  if (prev_state2 == 1 && curr_state == 0) {
//    button_pressed = 2;
//  }
//  prev_state1 = curr_state;

  
  int new_coord[2]; // the new coordinate for the head of the snake
  if (imu_direction != curr_direction) {
    //redraw snake 
    curr_direction = imu_direction;
    if(imu_direction == 0) { // left
      new_coord[0] = snake[0][0] - 1;
      new_coord[1] = snake[0][1]; 
    } else if (imu_direction == 1) { // down
      new_coord[0] = snake[0][0];
      new_coord[1] = snake[0][1] + 1;
    } else if (imu_direction == 2) { // right
      new_coord[0] = snake[0][0] + 1;
      new_coord[1] = snake[0][1];
    } else if (imu_direction == 3) { // up
      new_coord[0] = snake[0][0];
      new_coord[1] = snake[0][1] - 1;
    }
  } else { // keep going in the current direction
    if (curr_direction == 0) { // left
      new_coord[0] = snake[0][0] - 1;
      new_coord[1] = snake[0][1]; 
    } else if (curr_direction == 1) { // down
      new_coord[0] = snake[0][0];
      new_coord[1] = snake[0][1] + 1;
    } else if (curr_direction == 2) { // right
      new_coord[0] = snake[0][0] + 1;
      new_coord[1] = snake[0][1];
    } else if (curr_direction == 3) { // up
      new_coord[0] = snake[0][0];
      new_coord[1] = snake[0][1] - 1;
    }
  }

  // check if player loses
  if (is_gameover(new_coord)) {
    game_over_screen();
    return;

    
  }
  
  //print score
  print_score();

  
  // check if player hits apple >>>> also should make adjustment to count apple as being hit if it spawns on part of the already existing snake
  if (hits_apple(new_coord)) {

    // grow snake by 20
    snake_size +=20;

    // update high score if current score > high score
    if (snake_size > high_score) {
      high_score = snake_size;
    }
    
    // move apple
    move_apple();
    graphics.fillRect(apple[0], apple[1], 10, 10, WHITE);
  }
  
  roll_over(new_coord);
  
  //shift snake
  for (int i = snake_size-1; i > -1 ; i--) {
    snake[i][0] = snake[i-1][0];
    snake[i][1] = snake[i-1][1];
  }
  snake[0][0] = new_coord[0];
  snake[0][1] = new_coord[1];

  //draw snake
  for (int i = 0 ; i < snake_size ; i++) {
    //graphics.dotFast(snake[i][0], snake[i][1], WHITE);
    graphics.fillRect(snake[i][0], snake[i][1], 5, 5, WHITE);
  }
  graphics.end();
}
