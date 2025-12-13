/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10/1/2025, 2:15:24 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------
|                 Brain Port Assignments                         |
|     Right Motor 1: PORT 1 | PORT 11: Intake Mid Motor          |
|     Right Motor 2: PORT 2 | PORT 12: Intake Top Motor          |
|     Right Motor 3: PORT 3 | PORT 13: Sorter Motor Driver       |
|     Right Motor 4: PORT 4 | PORT 14: Sorter Door Driver        |
|      Left Motor 5: PORT 5 | PORT 15: Turret Driver             |
|      Left Motor 6: PORT 6 | PORT 16: Turret Rollers            |
|      Left Motor 7: PORT 7 | PORT 17: Shiv Motor                |
|      Left Motor 8: PORT 8 | PORT 18: Color Sensor              |
|          Inertial: PORT 9 | PORT 19: None                      |
|       Intake Top: PORT 10 | PORT 20: None                      |
|                     PORT 21: Radio                             |
-----------------------------------------------------------------*/

/*--------------------
 | Brain 3-Wire Ports |
 |   A: Turret Pot    |
 |   B: None          |
 |   C: None          |
 |   D: None          |
 |   E: None          |
 |   F: None          |
 |   G: None          |
 |   H: None          |
 ---------------------*/



/*---------------------------------------------------------------- 
|                     Controls                                   |
| L2: Outtake                            R2: Intake              |
| L1: None                               R1: None                |
|                                                                |
| Ls3: F/B                               Rs2: L/R                |
| Ls4: None                              Rs1: None               |
|       Up: None                 X:None                          |
| Left: None  Right: None    Y:None   A:Sort Wheel Go            |
|       Down: None               B:None                          |
----------------------------------------------------------------*/

#include "MCEC_Objects.h"
#include <cmath>

#define TURRET_MAX_ANGLE 23
#define TURRET_MIN_ANGLE 11

vex::brain Brain;
vex::inertial inertial = vex::inertial(vex::PORT9);
vex::controller controller = vex::controller();

vex::optical ballOptical = vex::optical(vex::PORT18);

// define your global instances of motors and other devices here

MCEC::Joystick lStick, rStick;

MCEC::Drivetrain8 drivetrain(
    vex::PORT1 , vex::PORT2 , vex::PORT3 , vex::PORT4 , 
    vex::PORT5, vex::PORT6, vex::PORT7, vex::PORT8
);

// Motors
vex::motor intakeMid    = vex::motor(vex::PORT12);
vex::motor intakeFront  = vex::motor(vex::PORT10);
vex::motor intakeBack   = vex::motor(vex::PORT11);
vex::motor sorterMotor  = vex::motor(vex::PORT13);
vex::motor sorterDoor   = vex::motor(vex::PORT14);
vex::motor turretDriver = vex::motor(vex::PORT15);
vex::motor turretRoller = vex::motor(vex::PORT16);
vex::motor shivMotor    = vex::motor(vex::PORT17);


// Three Wires
vex::pot turretPos = vex::pot(Brain.ThreeWirePort.A);

#define TRACKING_WHEEL_RADIUS        1.625f // in inches
#define TRACKING_WHEEL_CIRCUMFERENCE (2 * TRACKING_WHEEL_RADIUS * M_PI) // in inches

#define IS_RED(color)  (0xFF0000 & (uint32_t)detectedColor) >> 16 == 0xff
#define IS_BLUE(color) (0x0000FF & (uint32_t)detectedColor) == 0xff
#define IS_NOT_MINE(color) IS_RED(color)

float xOff = 0, yOff = 0;
float initialHeading = 0;
bool intakeIn = false, intakeOut = false;

bool xDown, yDown, aDown, bDown;

bool isOpen = false, isIntake = false;



enum TurretStates{LowGoal, Raising, HighGoal, Lowering};
TurretStates turretState = TurretStates::LowGoal;

void TurretUpdate(){
    Brain.Screen.setCursor(1, 1);
    switch(turretState){
        case TurretStates::LowGoal:
            if(xDown){
                turretDriver.spin(vex::reverse, 120, vex::rpm);
                turretState = TurretStates::Raising;
            }
            Brain.Screen.print("LowGoal   ");
            break;
        case TurretStates::Raising:
            if(turretPos.angle(vex::percent) >= TURRET_MAX_ANGLE){
                turretDriver.stop(vex::brakeType::brake);
                turretState = TurretStates::HighGoal;
            }
            Brain.Screen.print("Raising   ");
            break;
        case TurretStates::HighGoal:
            if(xDown){
                turretDriver.spin(vex::forward, 120, vex::rpm);
                turretState = TurretStates::Lowering;
            }
            Brain.Screen.print("HighGoal   ");
            break;
        case TurretStates::Lowering:
            if(turretPos.angle(vex::percent) <= TURRET_MIN_ANGLE){
                turretDriver.stop(vex::brakeType::hold);
                turretState = TurretStates::LowGoal;
            }
            Brain.Screen.print("Lowering   ");
            break;
    }
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print(turretPos.angle(vex::percent));
    Brain.Screen.print("     ");
}

void ColorDoorOpen(){
    sorterDoor.spinToPosition(5, vex::degrees, 10, vex::rpm, false);
    // sorterDoor.spinFor(vex::reverse, 360, vex::deg);
    isOpen = true;
}
void ColorDoorClose(){
    sorterDoor.spinToPosition(95, vex::degrees, 10, vex::rpm, false);
    // sorterDoor.spinFor(vex::forward, 360, vex::deg);
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
    if(!intakeIn){ 
        ballOptical.setLight(vex::ledState::off);
        return;
    }
    ballOptical.setLight(vex::ledState::on);
    

    double hue = ballOptical.hue();
    double brightness = ballOptical.brightness();
    vex::color detectedColor = ballOptical.color();
    bool isNear = ballOptical.isNearObject();


    
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print(hue);
    Brain.Screen.print("       ");

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print(brightness);
    Brain.Screen.print("       ");

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("%X", detectedColor.rgb());
    Brain.Screen.print("       ");

    Brain.Screen.setCursor(6, 1);

    if(isNear){
        if(IS_NOT_MINE(detectedColor)){
            ColorDoorOpen();
            Brain.Screen.print("Not Mine");
        }else{
            ColorDoorClose();
            Brain.Screen.print("Mine    ");
        }

    }
}

void ReadPosition(){
    // yOff += 
    //     TRACKING_WHEEL_CIRCUMFERENCE * 
    //     fbRot.position(vex::rotationUnits::rev) * 
    //     std::sin((-inertial.yaw() + 90) * (M_PI / 180.0));
    // xOff += 
    //     TRACKING_WHEEL_CIRCUMFERENCE * 
    //     fbRot.position(vex::rotationUnits::rev) * 
    //     std::cos((-inertial.yaw() + 90) * (M_PI / 180.0));
    // yOff += 
    //     TRACKING_WHEEL_CIRCUMFERENCE * 
    //     lrRot.position(vex::rotationUnits::rev) * 
    //     std::sin((-inertial.yaw() + 180) * (M_PI / 180.0));
    // xOff +=
    //     TRACKING_WHEEL_CIRCUMFERENCE * 
    //     lrRot.position(vex::rotationUnits::rev) * 
    //     std::cos((-inertial.yaw() + 180) * (M_PI / 180.0));

    // fbRot.resetPosition();
    // lrRot.resetPosition();
}

void IntakeGo(){
    intakeFront.spin(vex::forward, 300, vex::rpm);
    intakeBack.spin(vex::reverse, 300, vex::rpm);
    intakeMid.spin(vex::forward, 300, vex::rpm);

    turretRoller.spin(vex::forward, 300, vex::rpm);

    
    sorterMotor.spin(vex::forward, 300, vex::rpm);
}

void IntakeNotGo(){
    intakeFront.spin(vex::reverse, 300, vex::rpm);
    intakeBack.spin(vex::forward, 300, vex::rpm);
    intakeMid.spin(vex::reverse, 300, vex::rpm);

    turretRoller.spin(vex::reverse, 300, vex::rpm);

    sorterMotor.spin(vex::reverse, 300, vex::rpm);
}

void IntakeStop(){
    intakeFront.stop();
    intakeBack.stop();
    intakeMid.stop();

    turretRoller.stop();

    
    sorterMotor.stop();
}

void DriverLoop(){
    if(!inertial.isCalibrating()){
        if(ReadController()){
            drivetrain.Drive(-lStick.y, lStick.x);
        }else{
            drivetrain.Stop();
        }


        // if(aDown){
        //     sorterMotor.spin(vex::forward, 300, vex::rpm);
        // }else{
        //     sorterMotor.stop();
        // }

        if(intakeIn && !intakeOut){ // Intake in
            IntakeGo();
            isIntake = true;
        }else if(!intakeIn && intakeOut){ // Intake out
            IntakeNotGo();
            isIntake = false;
        }else{
            IntakeStop();
            isIntake = false;
        }

        // if(isIntake){
        //     sorterDoor.spinToPosition(0);
        // }

        ColorRead();

        TurretUpdate();

        ReadPosition();
    }
    vex::this_thread::sleep_for(10);
}

int main(){
    inertial.calibrate(3);

    inertial.resetHeading();
    while(inertial.isCalibrating());
    inertial.setHeading(0, vex::deg);
    inertial.setRotation(0, vex::deg);
    // controller.rumble("... -.. .. -.-- -... -");
    controller.rumble("...");

    // fbRot.resetPosition();
    // lrRot.resetPosition();

    ballOptical.integrationTime(50);
    ballOptical.setLightPower(100, vex::percent);
    ballOptical.setLight(vex::ledState::off);

    sorterDoor.resetPosition();

    while(1) {
        DriverLoop();
    }
}
