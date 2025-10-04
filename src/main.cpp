/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10/1/2025, 2:15:24 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "MCEC_Objects.h"

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;
vex::inertial inertial = vex::inertial(vex::PORT2);
vex::motor motor1 = vex::motor(vex::PORT10);
vex::controller controller = vex::controller();

// define your global instances of motors and other devices here

MCEC::Joystick lStick, rStick;

MCEC::Drivetrain8 drivetrain(
    vex::PORT6 , vex::PORT7 , vex::PORT8 , vex::PORT9 , 
    vex::PORT11, vex::PORT12, vex::PORT13, vex::PORT14
);



void ReadController(){
    lStick.Set(
        controller.Axis1.position(), 
        controller.Axis2.position() 
    );
    rStick.Set(
        controller.Axis3.position(), 
        controller.Axis4.position()
    );
}

int main() {
    inertial.calibrate(3);
    char rotation[60], joystick[60];

    inertial.resetHeading();
    while(inertial.isCalibrating());
    inertial.setHeading(0, vex::deg);
    inertial.setRotation(0, vex::deg);
    controller.rumble("...");

    while(1) {
        if(!inertial.isCalibrating()){

            ReadController();

            drivetrain.Drive(lStick.y, rStick.x);
            
            sprintf(rotation, "rotation:%.2f, heading:%.2f", inertial.rotation(), inertial.heading());
            Brain.Screen.printAt(10, 50, rotation);
            
            sprintf(
                joystick, 
                "LX:%d, LY:%d, RX:%d, RY:%d     ",
                lStick.x, lStick.y,
                rStick.x, rStick.y
            );
            Brain.Screen.printAt(10, 100, joystick);
        }
        
        vex::this_thread::sleep_for(10);
    }
}
