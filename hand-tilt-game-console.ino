#include "CompositeVideo/CompositeGraphics.h"
#include "CompositeVideo/Image.h"
#include "CompositeVideo/CompositeOutput.h"

#include "CompositeVideo/luni.h"
#include "CompositeVideo/viper.h"
#include "CompositeVideo/viper_transparent1.h"
#include "CompositeVideo/viper_transparent2.h"
#include "CompositeVideo/viper_transparent3.h"
#include "CompositeVideo/viper_transparent4.h"
#include "CompositeVideo/viper_transparent5.h"
#include "CompositeVideo/viper_transparent6.h"
#include "CompositeVideo/viper_transparent7.h"
#include "CompositeVideo/viper_transparent8.h"
#include "CompositeVideo/font6x8.h"

#include <soc/rtc.h>

//colors
#define WHITE 50
#define GRAY 30
#define BLACK 255

#define STARTING_FROG_TOP_X 158
#define STARTING_FROG_TOP_Y 203
#define STARTING_FROG_LEFT_X 154
#define STARTING_FROG_LEFT_Y 210
#define STARTING_FROG_RIGHT_X 162
#define STARTING_FROG_RIGHT_Y 210

#define BUTTON0 34
#define BUTTON1 0
#define BUTTON2 35

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

// viper picture
Image<CompositeGraphics> viper0(viper::xres, viper::yres, viper::pixels);
Image<CompositeGraphics> viper1(viper_transparent1::xres, viper_transparent1::yres, viper_transparent1::pixels);
Image<CompositeGraphics> viper2(viper_transparent2::xres, viper_transparent2::yres, viper_transparent2::pixels);
Image<CompositeGraphics> viper3(viper_transparent3::xres, viper_transparent3::yres, viper_transparent3::pixels);
Image<CompositeGraphics> viper4(viper_transparent4::xres, viper_transparent4::yres, viper_transparent4::pixels);
Image<CompositeGraphics> viper5(viper_transparent5::xres, viper_transparent5::yres, viper_transparent5::pixels);
Image<CompositeGraphics> viper6(viper_transparent6::xres, viper_transparent6::yres, viper_transparent6::pixels);
Image<CompositeGraphics> viper7(viper_transparent7::xres, viper_transparent7::yres, viper_transparent7::pixels);
Image<CompositeGraphics> viper8(viper_transparent8::xres, viper_transparent8::yres, viper_transparent8::pixels);

//font is based on ASCII starting from char 32 (space), width end height of the monospace characters. 
//All characters are staored in an image vertically. Value 0 is background.
Font<CompositeGraphics> font(6, 8, font6x8::pixels);


void draw_logo(int color) {
  //H
  graphics.fillRect(125, 70, 5, 29, color);
  graphics.fillRect(130, 86, 10, 5, color);
  graphics.fillRect(140, 70, 5, 29, color);

  //T
  graphics.fillRect(156, 70, 20, 5, color);
  graphics.fillRect(163, 75, 6, 24, color);

  //3
  graphics.fillRect(188, 70, 12, 5, color);
  graphics.fillRect(192, 82, 8, 5, color);
  graphics.fillRect(188, 94, 12, 5, color);
  graphics.fillRect(200, 70, 5, 29, color);

  graphics.setTextColor(color);
  graphics.setCursor(125, 100);
  graphics.print("HAND TILT 3000");
  graphics.setTextColor(WHITE);
}

void title_screen(int color) {
  draw_logo(color);

  graphics.setTextColor(color);
  
  // game options
  graphics.setCursor(136, 115);
  graphics.print("0   SNAKE");
  graphics.setCursor(136, 125);
  graphics.print("1   DODGER");

  graphics.setTextColor(WHITE);
}

void fade_out_title_screen() {
  for (int i = WHITE ; i > 0 ; i = floor(i/1.5)) {
    graphics.begin(BLACK);
    title_screen(i);
    graphics.end();
    delay(50);
    Serial.printf("fade out color %i\n", i);
  }
  graphics.begin(0);
  graphics.end();
  delay(50);
}

void fade_in_title_screen() {
  delay(50);
  for(int i = 2; i < WHITE ; i *=1.5) {
    graphics.begin(BLACK);
    title_screen(i);
    graphics.end();
    delay(50);
  }
  graphics.begin(BLACK);
  title_screen(WHITE);
  graphics.end();
  delay(50);
}

int snake[300][2]; // snake coordinate array
int obstacles[100][3]; // dodge obstactles coordinate array

int frog[3][2]; // starts in the middle on the bottom
int obstacle_size = 3;
int snake_size = 30; // initialize snake size to 10
int score = 0;
int difficulty = 0;

int apple[2] = {random(11, 309), random(11, 209)};

void compositeCore(void *data)
{
  while (true)
  {
    //just send the graphics frontbuffer whithout any interruption 
    composite.sendFrameHalfResolution(&graphics.frame);
  }
}

void reset_frog() {
  frog[0][0] = STARTING_FROG_LEFT_X; // bottom left x
  frog[0][1] = STARTING_FROG_LEFT_Y; // bottom left y
  frog[1][0] = STARTING_FROG_RIGHT_X; // bottom right x
  frog[1][1] = STARTING_FROG_RIGHT_Y; // bottom right y
  frog[2][0] = STARTING_FROG_TOP_X; // top x
  frog[2][1] = STARTING_FROG_TOP_Y; // top y
}

void show_difficulty_screen(int color) {
  
  graphics.setTextColor(color);
  graphics.setCursor(136, 100);
  graphics.print("DIFFICULTY");
  graphics.setCursor(136, 115);
  graphics.print("0   EASY");
  graphics.setCursor(136, 125);
  graphics.print("1   MEDIUM");
  graphics.setCursor(136, 135);
  graphics.print("2   HARD");

  graphics.setTextColor(WHITE);
}

void fade_in_difficulty_screen() {
  delay(50);
  int index = 0;
  for (int i = 2 ; i < WHITE ; i *=2) {
    graphics.begin(0);
    show_difficulty_screen(i);
    if (index == 1) {
      viper1.draw(graphics, 133, 25);
    } else if (index == 2) {
      viper2.draw(graphics, 133, 25);
    } else if (index == 3) {
      viper3.draw(graphics, 133, 25);
    } else if (index == 4) {
      viper4.draw(graphics, 133, 25);
    } else if (index == 5) {
      viper5.draw(graphics, 133, 25);
    } else if (index == 6) {
      viper6.draw(graphics, 133, 25);
    } else if (index == 7) {
      viper7.draw(graphics, 133, 25);
    } else if (index == 8) {
      viper8.draw(graphics, 133, 25);
    }
    index++;
    graphics.end();
    delay(50);
  }
  graphics.begin(0);
  show_difficulty_screen(WHITE);
  viper0.draw(graphics, 133, 25);
  graphics.end();
  delay(50);
}

void fade_out_difficulty_screen() {
//  int index = 9;
  graphics.begin(0);
  graphics.end();
  for (int i = WHITE ; i > 0 ; i = floor(i/1.5)) {
    graphics.begin(0);
    show_difficulty_screen(i);
//    if (index == 9) {
//      viper0.draw(graphics, 133, 25);
//    } else if (index == 8) {
//      viper8.draw(graphics, 133, 25);
//    } else if (index == 7) {
//      viper7.draw(graphics, 133, 25);
//    } else if (index == 6) {
//      viper6.draw(graphics, 133, 25);
//    } else if (index == 5) {
//      viper5.draw(graphics, 133, 25);
//    } else if (index == 4) {
//      viper4.draw(graphics, 133, 25);
//    } else if (index == 3) {
//      viper3.draw(graphics, 133, 25);
//    } else if (index == 2) {
//      viper2.draw(graphics, 133, 25);
//    } else if (index == 1) {
//      viper1.draw(graphics, 133, 25);
//    }
//    index--;
    graphics.end();
    delay(50);
  }
  graphics.begin(0);
  graphics.end();
  delay(50);
}

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
  
  graphics.setTextColor(WHITE);
  // wait for a second after turning on before fading in the title screen
  delay(1000);
  
  fade_in_title_screen();
  
  while (1) {
    byte curr_state = digitalRead(BUTTON0);
    if (prev_state0 == 1 && curr_state == 0) {
      button_pressed = 0;
      prev_state0 = curr_state;
  
      // initialize snake
      snake[0][0] = 64;
      snake[0][1] = 32;
      for (int i = 1 ; i < 300 ; i++) {
        snake[i][0] = -10;
        snake[i][1] = -10;
      }
      
      fade_out_title_screen();
      
      fade_in_difficulty_screen();

      while (1) {
        byte curr_state = digitalRead(BUTTON0);
        if (prev_state0 == 1 && curr_state == 0) {
          difficulty = 1;
          prev_state0 = curr_state;
          fade_out_difficulty_screen();
          Serial.println("test1");
          break;
        }
        prev_state0 = curr_state;
        
        curr_state = digitalRead(BUTTON1);
        if (prev_state1 == 1 && curr_state == 0) {
          difficulty = 2;
          prev_state1 = curr_state;
          fade_out_difficulty_screen();
          break;
        }
        prev_state1 = curr_state;

        curr_state = digitalRead(BUTTON2);
        if (prev_state2 == 1 && curr_state == 0) {
          difficulty = 3;
          prev_state2 = curr_state;
          fade_out_difficulty_screen();
          break;
        }
        prev_state2 = curr_state;
        
      }
      Serial.println("test2");
      break;
    }
    prev_state0 = curr_state;

    curr_state = digitalRead(BUTTON1);
    if (prev_state1 == 1 && curr_state == 0) {
      button_pressed = 1;
      prev_state1 = curr_state;

      // initilize dodger
      reset_frog();

      // initialize obstacles, obstacle size is 3
      for (int i = 0 ; i < obstacle_size ; i++) {
        obstacles[i][0] =  random(0, 300); // x
        obstacles[i][1] = random(0, 170); // y
        obstacles[i][2] = random(10, 20); // length/width of box
      }
      fade_out_title_screen();
      break;
    }
    prev_state1 = curr_state;

    curr_state = digitalRead(BUTTON2);
    if (prev_state2 == 1 && curr_state == 0) {
      graphics.begin(BLACK);
      luni0.draw(graphics, 70, 65);
      luni0.draw(graphics, 210, 65);
      
      title_screen(WHITE);
      
      graphics.end();
      delay(5000);
      graphics.begin(BLACK);
      
      title_screen(WHITE);
      
      graphics.end();
    }
    delay(10);
  }
  
  
}



// rolls over any out of bounds coordinate
void roll_over(int coord[2]) {

  if (coord[0] >= 315) {
    coord[0] = 0;
  } else if (coord[0] < 0) {
    coord[0] = 314;
  }
  if (coord [1] >= 215) {
    coord[1] = 0;
  } else if (coord[1] < 0) {
    coord[1] = 214;
  }
}


// max - 4 because its a square
void move_apple() {
  apple[0] = random(11, 309);
  apple[1] = random(11, 209);
}

bool hits_apple(int coord[2]) {
  return (coord[0]+2 >= apple[0] && coord[0]-2 <= apple[0]+10) && (coord[1]+2 >= apple[1] && coord[1]-2 <= apple[1]+10);
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
  graphics.setCursor(10,0);
  graphics.print("SCORE: ");
  graphics.print(score);
}

void game_over_screen() {
  graphics.setCursor(140, 92);
  graphics.print("GAME OVER");
  graphics.setCursor(138, 100);
  graphics.print("SCORE: ");
  graphics.print(score);
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

void snake_game() {
  // draw apple
  graphics.fillRect(apple[0], apple[1], 10, 10, WHITE); 
  
  int new_coord[2]; // the new coordinate for the head of the snake
  if (imu_direction != curr_direction) { // change direction
    curr_direction = imu_direction;
    if(imu_direction == 0) { // left
      new_coord[0] = snake[0][0] - difficulty;
      new_coord[1] = snake[0][1]; 
    } else if (imu_direction == 1) { // down
      new_coord[0] = snake[0][0];
      new_coord[1] = snake[0][1] + difficulty;
    } else if (imu_direction == 2) { // right
      new_coord[0] = snake[0][0] + difficulty;
      new_coord[1] = snake[0][1];
    } else if (imu_direction == 3) { // up
      new_coord[0] = snake[0][0];
      new_coord[1] = snake[0][1] - difficulty;
    }
  } else { // keep going in the current direction
    if (curr_direction == 0) { // left
      new_coord[0] = snake[0][0] - difficulty;
      new_coord[1] = snake[0][1]; 
    } else if (curr_direction == 1) { // down
      new_coord[0] = snake[0][0];
      new_coord[1] = snake[0][1] + difficulty;
    } else if (curr_direction == 2) { // right
      new_coord[0] = snake[0][0] + difficulty;
      new_coord[1] = snake[0][1];
    } else if (curr_direction == 3) { // up
      new_coord[0] = snake[0][0];
      new_coord[1] = snake[0][1] - difficulty;
    }
  }

  // check if player loses
  if (is_gameover(new_coord)) {
    
    // flash snake
    for (int i = 0 ; i < 5 ; i++ ) {
      graphics.begin(0);
      for (int j = 0 ; j < snake_size ; j++) {
        graphics.fillRect(snake[j][0], snake[j][1], 5, 5, BLACK);
      }
      graphics.end();
      delay(50);
      graphics.begin(0);
      for (int j = 0 ; j < snake_size ; j++) {
        graphics.fillRect(snake[j][0], snake[j][1], 5, 5, WHITE);
      }
      graphics.end();
      delay(50);
    }

    //fade snake and apple to half
    int i;
    for (i = WHITE ; i > WHITE/2.5 ; i = floor(i/1.5)) {
      graphics.begin(0);
      graphics.fillRect(apple[0], apple[1], 10, 10, i); 
      for (int j = 0 ; j < snake_size ; j++) {
        graphics.fillRect(snake[j][0], snake[j][1], 5, 5, i);
      }
      graphics.end();
      delay(50);
    }
    graphics.begin(0);
    graphics.fillRect(apple[0], apple[1], 10, 10, i); 
    for (int j = 0 ; j < snake_size ; j++) {
      graphics.fillRect(snake[j][0], snake[j][1], 5, 5, i);
    }
    
    
    game_over_screen();
    return;
  }
  
  //print score
  print_score();

  bool contact_apple = hits_apple(new_coord);
  if (!contact_apple) { // if the head didn't hit the apple, check the rest of the snake
    for (int i = snake_size -2 ; i >= 0 ; i--) {
      if (hits_apple(snake[i])) {
        contact_apple = true;
      }
    }
  }

  if (contact_apple) {
      // grow snake by 20
      snake_size +=20;
      
      // increment score
      score = snake_size - 30;
      
      
      // move apple
      move_apple();
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
    graphics.fillRect(snake[i][0], snake[i][1], 5, 5, WHITE);
  }
}

// checks if the frog is contacting any obstacles
bool is_gameover_dodger() {
  for (int i = 0; i < obstacle_size ; i++) { //  for each obstacle
    for (int j = 0 ; j < 3 ; j++) { // for each frog
      for (int x = 0; x < 5 ; x++) { // all coordinates in each frog
        for (int y = 0; y < 5; y++) { // all coordinates in each frog
          if (frog[j][0] + x >= obstacles[i][0] && frog[j][0] + x <= obstacles[i][0] + obstacles[i][2]) { // if within the x bounds of an object
            if (frog[j][1] + y >= obstacles[i][1] && frog[j][1] + y <= obstacles[i][1] + obstacles[i][2]) { // if within the y bounds of an object
              return true;
            }
          } 
        }  
      }
    }
  }
  return false;
}

// moves the obstacles across the screen.
void move_obstacles() {
  
  for (int i = 0; i< obstacle_size ; i++) {
    int new_coord[2];
    new_coord[0] = obstacles[i][0] - 1;
    new_coord[1] = obstacles[i][1];

    roll_over(new_coord); // rolls over the obstacle if it reaches the left hand side
    obstacles[i][0] = new_coord[0];
  }
}

// if the frog reaches the top of the screen
bool level_completed() {
  if (frog[2][1] <= 0) {
    return true;
  }
  return false;
}

// prevent frog from leaving screen
bool border_check(int coord[2]) {
  if (coord[0] >= 315) { // right
    coord[0] -= 2;
    return true;
  } else if (coord[0] <= 5) { // left 
    coord[0] += 2;
    return true;
  }
  if (coord[1] >= 215) { // bottom
    coord[1] -= 2;
    return true;
  }
  return false;
}

// main dodger game code
void dodger_game() {
  // display score
  print_score();
  
  // draw player
  for (int i = 0 ; i < 3 ; i++) {
    graphics.fillRect(frog[i][0], frog[i][1], 5, 5, WHITE);
  }

  // draw obstacles
  for (int i = 0 ; i < obstacle_size ; i ++) {
    graphics.fillRect(obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][2], GRAY);
  }

  if (is_gameover_dodger()){
    // flash frog
    for (int i = 0 ; i < 5 ; i++ ) {
      graphics.begin(0);
      for (int j = 0 ; j < obstacle_size ; j ++) {
        graphics.fillRect(obstacles[j][0], obstacles[j][1], obstacles[j][2], obstacles[j][2], GRAY);
      }
      for (int i = 0 ; i < 3 ; i++) {
        graphics.fillRect(frog[i][0], frog[i][1], 5, 5, BLACK);
      }
      graphics.end();
      delay(50);
      graphics.begin(0);
      for (int j = 0 ; j < obstacle_size ; j ++) {
        graphics.fillRect(obstacles[j][0], obstacles[j][1], obstacles[j][2], obstacles[j][2], GRAY);
      }
      for (int i = 0 ; i < 3 ; i++) {
        graphics.fillRect(frog[i][0], frog[i][1], 5, 5, WHITE);
      }
      graphics.end();
      delay(50);
    }

    //fade snake and apple to half
    int i;
    for (i = GRAY ; i > GRAY/2.5 ; i = floor(i/1.5)) {
      graphics.begin(0);
      for (int j = 0 ; j < obstacle_size ; j ++) {
        graphics.fillRect(obstacles[j][0], obstacles[j][1], obstacles[j][2], obstacles[j][2], i);
      }
      for (int j = 0 ; j < 3 ; j++) {
        graphics.fillRect(frog[j][0], frog[j][1], 5, 5, i);
      }
      graphics.end();
      delay(50);
    }
    graphics.begin(0);
    for (int j = 0 ; j < obstacle_size ; j ++) {
        graphics.fillRect(obstacles[j][0], obstacles[j][1], obstacles[j][2], obstacles[j][2], i);
    }
    for (int j = 0 ; j < 3 ; j++) {
      graphics.fillRect(frog[j][0], frog[j][1], 5, 5, i);
    }
    game_over_screen();
    return;
  }

  // check if the level has been complete
  if (level_completed()) {
    // increase score
    score += 10;
    
    // increase number of obstacles
    for (int i = obstacle_size-1; i < obstacle_size+3; i++) {
      obstacles[i][0] =  random(0, 300); // x
      obstacles[i][1] = random(0, 170); // y
      obstacles[i][2] = random(10, 20); // length/width of box
    }
    obstacle_size += 1;
    
    // put frog back at bottom
    reset_frog();
    curr_direction = 0;
  
    
  }
  
  

  if (imu_direction != curr_direction) {
    curr_direction = imu_direction;
    if(imu_direction == 0) { // left
      frog[0][0] -= 2;
      if (!border_check(frog[0])) { // prevent leaving screen
        frog[1][0] -= 2;
        frog[2][0] -= 2; 
      }
      
    } else if (imu_direction == 2) { // right
      frog[1][0] += 2;
      if (!border_check(frog[1])) { // prevent leaving screen
        frog[0][0] += 2;
        frog[2][0] += 2;
      }
    } else if (imu_direction == 3) { // up
      frog[0][1] -= 2;
      frog[1][1] -= 2;
      frog[2][1] -= 2;
    } else if (imu_direction == 1) { // down
      frog[1][1] += 2;
      if (!border_check(frog[1])) { // prevent leaving screen
        frog[0][1] += 2;
        frog[2][1] += 2;
      }
      
    }
  } else { // if the direction hasn't changed then continue the same direction
    if(curr_direction == 0) { // left
      frog[0][0] -= 2;
      if (!border_check(frog[0])) { // prevent leaving screen
        frog[1][0] -= 2;
        frog[2][0] -= 2; 
      }
    } else if (curr_direction == 2) { // right
      frog[1][0] += 2;
      if (!border_check(frog[1])) { // prevent leaving screen
        frog[0][0] += 2;
        frog[2][0] += 2;
      }
    } else if (curr_direction == 3) { // up
      frog[0][1] -= 2;
      frog[1][1] -= 2;
      frog[2][1] -= 2;
    } else if (curr_direction == 1) { // down
      frog[1][1] += 2;
      if (!border_check(frog[1])) { // prevent leaving screen
        frog[0][1] += 2;
        frog[2][1] += 2;
      }
    }
  }

  move_obstacles();
}

void imu_collect_direction() {
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
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
}

void loop() {

  // get the direction from the imu
  imu_collect_direction();

  
  // wipe lcd and begin writing to the lcd
  graphics.begin(BLACK);
  
  if (button_pressed == 0) { // if snake selected
    snake_game();
  } else if(button_pressed == 1) { // if dodger selected
    dodger_game();
  }
   
  graphics.end();
}
