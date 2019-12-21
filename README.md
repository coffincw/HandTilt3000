# Hand Tilt 3000
Hand tilt controlled arcade gaming system powered by an Arduino ESP32. 

## How to Run
Simply connect the MPU sensor to the board according to the pin setup pictured below, connect the board to the LCD screen using two wires included in the pin set up, power up the chip, install the Electronic Cats MPU6050 library, and play!

## Pin Setup
![ESP32 Wire Setup](https://i.imgur.com/IgPsVIW.jpg)

## Games
- Snake
    - Classic snake with three difficulty modes and border roll over modifications.
- Dodger
    - Simplified version of frogger, simply make it to the top of the screen while avoiding the obstacles.

## Dependencies
- MPU6050 by Electronic Cats Version 0.0.2
- Bitluni's ESP32 Composite Video Library (INCLUDED IN CODE)
    - Documentation: [Project Page](https://bitluni.net/esp32-composite-video "ESP32 Composite Video")

## Parts Needed to Build
- Arduino ESP32
- [MPU-6050 Sensor](https://www.amazon.com/Ximimark-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B07MMZ37PT/ref=asc_df_B07MMZ37PT/?tag=hyprod-20&linkCode=df0&hvadid=309793588525&hvpos=1o1&hvnetw=g&hvrand=14953648762274415848&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9001878&hvtargid=pla-643815931576&psc=1&tag=&ref=&adgrpid=60862048759&hvpone=&hvptwo=&hvadid=309793588525&hvpos=1o1&hvnetw=g&hvrand=14953648762274415848&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9001878&hvtargid=pla-643815931576)
- AV Video Splitter
    - We took a regular AV video coord and split and soldered it ourselves
- Glove (optional)
    - Needed to attach sensor to hand (can do with out simply by using rubber bands)
- x5 Female to Male Wires

## Game Demonstrations
![Snake](https://github.com/coffincw/HandTilt3000/blob/master/snake-video.gif)
<img src="https://github.com/coffincw/HandTilt3000/blob/master/dodger-video.gif" width="272" height="332" />
