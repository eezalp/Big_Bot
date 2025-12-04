/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10/1/2025, 2:15:24 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------
|                 Brain Port Assignments
|     Right Motor 1: PORT 1 | PORT 11: Intake Bottom Motor
|     Right Motor 2: PORT 2 | PORT 12: Intake Top Motor
|     Right Motor 3: PORT 3 | PORT 13: Sorter Motor Driver
|     Right Motor 4: PORT 4 | PORT 14: Sorter Door Driver
|      Left Motor 5: PORT 5 | PORT 15: Turret Driver
|      Left Motor 6: PORT 6 | PORT 16: Turret Rollers
|      Left Motor 7: PORT 7 | PORT 17: 
|      Left Motor 8: PORT 8 | PORT 18: Color Sensor
|   X-axis odometry: PORT 9 | PORT 19: Radio
|Intake Midsection: PORT 10 | PORT 20: Inertial
----------------------------------------------------------------*/


#include "MCEC_Objects.h"
#include <cmath>

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;
vex::inertial inertial = vex::inertial(vex::PORT20);
vex::controller controller = vex::controller();

vex::rotation fbRot = vex::rotation(vex::PORT9);
vex::rotation lrRot = vex::rotation(vex::PORT10);

vex::optical ballOptical = vex::optical( vex::PORT14 );
vex::distance tripwire = vex::distance( vex::PORT15 );

// define your global instances of motors and other devices here

MCEC::Joystick lStick, rStick;

MCEC::Drivetrain8 drivetrain(
    vex::PORT1 , vex::PORT2 , vex::PORT3 , vex::PORT4 , 
    vex::PORT5, vex::PORT6, vex::PORT7, vex::PORT8
);

vex::motor intakeMid    = vex::motor(vex::PORT10);
vex::motor intakeFront  = vex::motor(vex::PORT11);
vex::motor intakeBack   = vex::motor(vex::PORT12);
vex::motor sorterMotor  = vex::motor(vex::PORT13);
vex::motor sorterDoor   = vex::motor(vex::PORT14);
vex::motor turretDriver = vex::motor(vex::PORT15);

#define TRACKING_WHEEL_RADIUS        1.625f // in inches
#define TRACKING_WHEEL_CIRCUMFERENCE (2 * TRACKING_WHEEL_RADIUS * M_PI) // in inches

float xOff = 0, yOff = 0;
float initialHeading = 0;
bool intakeIn = false, intakeOut = false;

bool xDown, yDown, aDown, bDown;

bool isOpen = false;

void ColorDoorOpen(){
    // sorterDoor.spinTo(0, vex::degrees, 10, vex::rpm);
    sorterDoor.spinFor(vex::reverse, 360, vex::deg);
    isOpen = true;
}
void ColorDoorClose(){
    // sorterDoor.spinTo(120, vex::degrees, 10, vex::rpm);
    sorterDoor.spinFor(vex::forward, 360, vex::deg);
    isOpen = false;
}

bool ReadController(){
    rStick.Set(
        controller.Axis1.position(), 
        controller.Axis2.position()
    );
    lStick.Set(
        controller.Axis4.position(), 
        controller.Axis3.position()
    );

    intakeIn  = controller.ButtonR2.pressing();
    intakeOut = controller.ButtonL2.pressing();

    xDown = controller.ButtonX.pressing();
    yDown = controller.ButtonY.pressing();
    bDown = controller.ButtonB.pressing();
    aDown = controller.ButtonA.pressing();
    
    return (lStick.x != 0 || lStick.y != 0 || rStick.x != 0 || rStick.y != 0);
}

void ColorRead(){
    char buffer[64];

    double hue = ballOptical.hue();
    double brightness = ballOptical.brightness();
    vex::color detectedColor = ballOptical.color();
    bool isNear = ballOptical.isNearObject();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);

    if(isNear){
        if((0xFF0000 & (uint32_t)detectedColor) >> 16 == 0xff){
            if(detectedColor == 0xFF0000)
                Brain.Screen.print("Red Red");
            else
                Brain.Screen.print("Redish");
        }else if((0x00FF00 & (uint32_t)detectedColor) >> 8 == 0xff){
            if(detectedColor == 0x00ff00)
                Brain.Screen.print("Green Green");
            else
                Brain.Screen.print("Greenish");
        }else if((0x0000FF & (uint32_t)detectedColor) == 0xff){
            if(detectedColor == 0x0000FF)
                Brain.Screen.print("Blue Blue");
            else
                Brain.Screen.print("Blueish");
        }else{
            Brain.Screen.print("Color: Unknown");
        }

        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("%06x", (uint32_t)detectedColor);
        Brain.Screen.setCursor(3,1);
        Brain.Screen.print("Hue: %.2f", hue);
    }
    
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print(tripwire.objectDistance(vex::distanceUnits::mm));
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
    xOff +=
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
    controller.rumble("... -.. .. -.-- -... -");

    fbRot.resetPosition();
    lrRot.resetPosition();

    while(1) {
        if(!inertial.isCalibrating()){

            if(ReadController()){
                drivetrain.Drive(lStick.y, rStick.x);
            }else{
                drivetrain.Stop();
            }

            if(xDown && !isOpen){
                ColorDoorOpen();
            }
            if(yDown && isOpen){
                ColorDoorClose();
            }
            if(aDown){
                sorterMotor.spin(vex::forward, 300, vex::rpm);
            }else{
                sorterMotor.stop();
            }

            if(intakeIn && !intakeOut){ // Intake in
                intakeFront.spin(vex::forward, 300, vex::rpm);
                intakeBack.spin(vex::reverse, 300, vex::rpm);
                intakeMid.spin(vex::forward, 300, vex::rpm);
            }else if(!intakeIn && intakeOut){ // Intake out
                intakeFront.spin(vex::reverse, 300, vex::rpm);
                intakeBack.spin(vex::forward, 300, vex::rpm);
                intakeMid.spin(vex::reverse, 300, vex::rpm);
            }else{
                intakeFront.stop();
                intakeBack.stop();
                intakeMid.stop();
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
