+++
bg_image = "images/banners/banner_sandtable.webp"
image = "images/projects/sandtable/SandTableNoBackground.png"
title = "Sand Table"
date = "2021-05-20"
description = "This project aims to create a coffee table with a transparent top that reveals a thin layer of sand, where a steel ball draws geometric patterns. The design and construction of the hidden mechanism used to move the ball using a magnet is the core focus of the project."

link = "projects/sandtable/"
+++

# Introduction

The goal of this project is to build a coffee table featuring a mechanism that enables the creation of geometric patterns in sand using a steel ball. This sand table is inspired by similar projects such as <a href="https://www.instructables.com/Sand-Table/" target="_blank">this one</a> or <a href="https://sisyphus-industries.com/" target="_blank">this one</a>.

The core of the project lies in designing the magnetic guidance mechanism for the ball, constructing it, and integrating it into a custom-built table. The electronics and software algorithms also need to be developed.

# Mechanism Design

The sand tray must cover most of the table’s surface, and the ball must be able to reach any point. For these reasons, and for aesthetic purposes, a circular design was chosen for the table. It consists of three distinct parts:

- **The sand tray:** The top visible part, made with a transparent PMMA plate. It contains sand and a steel ball.

- **The intermediate section:** The central part of the table, housing the ball-driving mechanism. It is hidden from the user.

- **The lower section:** This contains the electronics and motors, also hidden from view.

The motion system (intermediate section) is based on a threaded rod rotating around the center of the table, which moves a magnet along a radial axis. The first version of this mechanism uses two motors, M1 and M2 (to provide the two necessary degrees of freedom), positioned as shown in the diagram below. A represents the magnet, and R a free-spinning wheel.

<center>
<img src="/images/projects/sandtable/schema_axe.webp" alt="Image not found !" width="80%"/>
</center>

&nbsp;

However, this version has a major drawback: it does not account for cable twisting needed to power and control M2. To solve this, M2 was moved to the lower section, making it fixed to the chassis. This required shifting M1 away from the center to make room for M2, which then drives radial movement using bevel gears. The offset of M1 from the center is achieved using a pulley and belt system. All parts were modeled in Onshape, as shown below.

<center>
<img src="/images/projects/sandtable/CAD_partial.png" alt="Image not found !" width="75%"/>
<img src="/images/projects/sandtable/bewel_gear_front.png" alt="Image not found !" width="20%"/>
</center>
&nbsp;

# Construction

The first component assembled was the magnet arm, built from laser-cut PMMA parts, an 8mm threaded rod with a brass nut, and an 8mm steel rod. These components were salvaged from a dismantled 3D printer. The assembled arm is shown below.

<center>
<img src="/images/projects/sandtable/shaft.jpg" alt="Image not found !" width="80%"/>
</center>

&nbsp;

The table frame was made from MDF, based on the earlier design diagrams. The curved parts — the table’s rim and sand tray — were also made from MDF using the kerf cutting technique, which allows it to bend. The MDF was glued with wood glue and held with clamps while drying.

The intermediate and lower sections are joined, with side access to the bottom section. To attach the sand tray (top section) to the intermediate part, a latch system was used. This system keeps the tray in place while allowing easy access for maintenance. The latches, shown below, lock in place with a small rotation.

<center>
<img src="/images/projects/sandtable/lock_m.jpg" alt="Image not found !" width="39%" style="margin-right: 2%;"/>
<img src="/images/projects/sandtable/lock_f.jpg" alt="Image not found !" width="39%"/>
</center>

&nbsp;

The steps of construction up to final assembly are shown below. Note the addition of a belt tensioner to ensure smooth operation of motor M1’s drive system.

<center>
<img src="/images/projects/sandtable/timing_belt.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<center>
<img src="/images/projects/sandtable/inside.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<center>
<img src="/images/projects/sandtable/open_1.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<center>
<img src="/images/projects/sandtable/open_2.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<center>
<img src="/images/projects/sandtable/finished.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

# Control

To calibrate the magnet’s position, a limit switch was initially used. However, mounting it on the rotating part caused issues with cable twisting. It was replaced by a Reed Switch, which detects the passing magnet. Positioned on the table’s central axis (fixed to the chassis), it allows for detection without being on a moving part.

The control board used is an Arduino Uno, connected to a stepper motor driver shield. This shield controls stepper motors M1 and M2 that move the magnet. The control code is written in C++ using the AFMotor library.

Two drawing patterns have been implemented. The `spiral` function draws a spiral by rotating both motors at different speeds, while the `zigzag` function draws wavy lines by rotating M1 at constant speed and stepping M2 at regular intervals. Here is the control code:

```
#include <AFMotor.h>
#include <time.h>

#define STEPSPERREVOLUTION 200.0
#define MAXREVARM 150.0
#define CENTERREDUCTION 2.8

// Connect motors to port #1 (M1 and M2)
AF_Stepper stepperArm(STEPSPERREVOLUTION, 2);
AF_Stepper stepperCenter(STEPSPERREVOLUTION, 1);

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper test!");

  stepperArm.setSpeed(100);  // 100 rpm
  stepperCenter.setSpeed(30);  // 30 rpm
}

void spiral(int revNum) {
  double stepsArm = STEPSPERREVOLUTION*MAXREVARM; 
  double stepsCenter = STEPSPERREVOLUTION*CENTERREDUCTION*revNum;
  AF_Stepper stepperMax = stepperArm;
  AF_Stepper stepperMin = stepperCenter;
  double nMax = stepsArm;
  double nMin = (stepsCenter < stepsArm) ? stepsCenter : stepsArm; //nombre de steps min
  if (nMin == stepsArm){
    stepperMax = stepperCenter;
    stepperMin = stepperArm;
    nMax = stepsCenter;
  }
  int stepPerLoop = (int) round(nMax/nMin);
  Serial.println(stepPerLoop);
  Serial.println(nMax);
  Serial.println(nMin);
  for (int i = 0; i < nMin; i ++){
    stepperMin.step(1, FORWARD, SINGLE);
    stepperMax.step(stepPerLoop, FORWARD, SINGLE);
  }
}

void zigzag(int revNum, int spacing, int amplitude){
  double stepsCenter = STEPSPERREVOLUTION*CENTERREDUCTION*revNum;
  bool direction = 0;
  for (int i = 0; i < stepsCenter; i ++){
    stepperCenter.step(1, FORWARD, SINGLE);
    if (i % spacing == 0){
      direction = !direction;
    }
    if (direction){
      stepperArm.step(amplitude, FORWARD, SINGLE);
    }
    else{
      stepperArm.step(amplitude, FORWARD, SINGLE);
    }
  }
}

void loop() {
  spiral(1);
}
```

&nbsp;

A demo video of the table in operation with a circular pattern is shown below.

<center>
<video width="80%" controls>
  <source src="/images/projects/sandtable/sandtable.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</center>

&nbsp;

# Possible Improvements

The main shortcomings of the final version are noise (due to vibrations, multiple bearings, and plastic parts), and inconsistent ball movement (when the sand layer is too thick or movements are too abrupt). Improvements could include:

**Vibrations and noise:** Better stepper motor control, greater center gear reduction, and use of machined instead of 3D-printed gears.

**Ball movement:** Stronger magnet, smoother sand (e.g., fine beach sand without irregular grains).

It would also be beneficial to add more drawing modes (e.g., rosettes, basic shapes), and to provide a dedicated power supply for the Arduino and motor shield, turning it into a more autonomous and user-friendly device.

