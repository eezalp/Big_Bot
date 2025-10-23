/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10/1/2025, 2:15:24 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "MCEC_Objects.h"
#include <cmath>

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

float xOff = 0, yOff = 0;
float initialHeading = 0;

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
    yOff += 
        TRACKING_WHEEL_CIRCUMFERENCE * 
        fbRot.position(vex::rotationUnits::rev) * 
        std::sin((-inertial.yaw() + 90) * (M_PI / 180.0));
    xOff += 
        TRACKING_WHEEL_CIRCUMFERENCE * 
        fbRot.position(vex::rotationUnits::rev) * 
        std::cos((-inertial.yaw() + 90) * (M_PI / 180.0));
    yOff += 
        TRACKING_WHEEL_CIRCUMFERENCE * 
        lrRot.position(vex::rotationUnits::rev) * 
        std::sin((-inertial.yaw() + 180) * (M_PI / 180.0));
    xOff +
        TRACKING_WHEEL_CIRCUMFERENCE * 
        lrRot.position(vex::rotationUnits::rev) * 
        std::cos((-inertial.yaw() + 180) * (M_PI / 180.0));

    fbRot.resetPosition();
    lrRot.resetPosition();
}

int main(){
    inertial.calibrate(3);
    char rotation[60], joystick[60];

    inertial.resetHeading();
    while(inertial.isCalibrating());
    inertial.setHeading(0, vex::deg);
    inertial.setRotation(0, vex::deg);
    controller.rumble("-.. .. .");

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
                "xOff:%.2f, yOff:%.2f     ",
                xOff, yOff
            );
            Brain.Screen.printAt(10, 100, joystick);

            ReadPosition();
        }
        
        vex::this_thread::sleep_for(10);
    }
}
