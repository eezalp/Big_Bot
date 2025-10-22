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

vex::rotation fbRot = vex::rotation(vex::PORT5);
vex::rotation lrRot = vex::rotation(vex::PORT3);

// define your global instances of motors and other devices here

MCEC::Joystick lStick, rStick;

MCEC::Drivetrain8 drivetrain(
    vex::PORT6 , vex::PORT7 , vex::PORT8 , vex::PORT9 , 
    vex::PORT11, vex::PORT12, vex::PORT13, vex::PORT14
);

#define TRACKING_WHEEL_RADIUS        1.625f // in inches
#define TRACKING_WHEEL_CIRCUMFERENCE (2 * TRACKING_WHEEL_RADIUS * M_PI) // in inches

bool ReadController(){
    rStick.Set(
        controller.Axis1.position(), 
        controller.Axis2.position()
    );
    lStick.Set(
        controller.Axis4.position(), 
        controller.Axis3.position()
    );

    return (lStick.x != 0 || lStick.y != 0 || rStick.x != 0 || rStick.y != 0);
}

void ReadPosition(){
    char location[60];
    sprintf(
        location, 
        "fb:%f, rl:%f    ",
        TRACKING_WHEEL_CIRCUMFERENCE * fbRot.position(vex::rotationUnits::rev), 
        TRACKING_WHEEL_CIRCUMFERENCE * lrRot.position(vex::rotationUnits::rev)
    );
    Brain.Screen.printAt(10, 130, location);
}

int main(){
    inertial.calibrate(3);
    char rotation[60], joystick[60];

    inertial.resetHeading();
    while(inertial.isCalibrating());
    inertial.setHeading(0, vex::deg);
    inertial.setRotation(0, vex::deg);
    controller.rumble("...");

    fbRot.resetPosition();
    lrRot.resetPosition();

    while(1) {
        if(!inertial.isCalibrating()){

            if(ReadController()){
                drivetrain.Drive(lStick.y, rStick.x);
            }else{
                drivetrain.Stop();
            }

            
            sprintf(rotation, "rotation:%.2f, heading:%.2f", inertial.rotation(), inertial.heading());
            Brain.Screen.printAt(10, 50, rotation);
            
            sprintf(
                joystick, 
                "LX:%d, LY:%d, RX:%d, RY:%d     ",
                lStick.x, lStick.y,
                rStick.x, rStick.y
            );
            Brain.Screen.printAt(10, 100, joystick);

            ReadPosition();
        }
        
        vex::this_thread::sleep_for(10);
    }
}
