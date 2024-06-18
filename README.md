# Side Project: PID controlled Quad-wing Elevator

In the project, we attempt to build a simplest possible quadcopterd, our goal is to able it to elevate itself for half a meter, and stay that altitude for 10 sec.

In order to achieve the goal, we divide the objective into two smaller goals: hardware assembling and PID algorithm designing.

## Hardware Assembling

We draft the simplest possible plan for building a quadcopter. 

**3D Printed Framework**: 3-inch micro quadcopter frame, in worst case, we can build one out of wood plate or popsickle stick.

**Motors**: brushless motor is a common choice for quadcopters as it can achieve a greater torque within its size limit, Its worth mention here that it can only be functional when attahced to an adition vital hardware -- the ESC (electronic speed controller). Therefore, we choose to use coreless motors in this project, since it is a much cheaper alternative and doesn't need an ESC to function. Furthermore, our project is not designed for performance, but just to stuy the methodology of building an PID controller.

**Flight Controller**: Common practice in the community simpley use a out-of-box commercial flight controller to perform mature controls on the quadcopter, including translation and even acrobatics. They alread has PID algorithm built-in, with all the control paradim implemented. In this project, we focus on study the implementation of a simple PID controller that only sustains the flight, thus we won't choose a flight controller, but a plain ESP32 chip (similar thing to arduino, its a programmable electronic board).

**remote controller**: we can embbed this functionality in the ESP32 mentioned previously too. We will try to implement a PC program that send instructions to the board in suitable communication protocols.

**soldering**: connecting circuits on ESP32 is a bit different from Arduino, as you have to soldering the wire on PINs.

## Software

**PID Controller**: this will be the topic we focus the most in this project, Heming can't give detailed explanation on it right now, as he is not familier with the topic neither.


## Statement

This project is Jesse and Isaac's independent work, aided financially by their parents, we recognize our love towards them. 


