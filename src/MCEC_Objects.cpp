#include "MCEC_Objects.h"

float MCEC::Lerp(float a, float b, float t){
    return ((1 - t) * a) + (b * t);
}

void MCEC::Drivetrain8::SetInertial(vex::inertial* _inertial){
  inertial = _inertial;
}

// @param joyX: input from -100 to 100
// @param joyY: input from -100 to 100
void MCEC::Drivetrain8::Drive(int joyX, int joyY){
    // Ensure the joystick inputs are within the valid range
    if(abs(joyX) > 100 || abs(joyY) > 100){
        return;
    }

    // Apply a deadzone to the joystick inputs
    #define DEADZONE 5
    joyX = (abs(joyX) < DEADZONE) ? 0 : joyX;
    joyY = (abs(joyY) < DEADZONE) ? 0 : joyY;

    // Setup the influence of each joystick axis
    float totalPower = abs(joyX) + abs(joyY);
    // Handle case where joysticks are both pointed in the wrong axis
    if(totalPower == 0){
        return;
    } 
    float py = (joyY / 100.0f) * (joyY < 0 ? -1 : 1);
    float px = (joyX / 100.0f) * (joyX < 0 ? -1 : 1);
    float tpx = (joyX / totalPower) * (joyX < 0 ? -1 : 1);
    float tpy = (joyY / totalPower) * (joyY < 0 ? -1 : 1);

    // Apply a scaling factor to each axis
    float xPower = (joyX * 6 * px * tpx);
    float yPower = (joyY * 6 * py * tpy);

    int Lpower = (xPower + yPower);
    int Rpower = (xPower - yPower);

    // Apply the power to the motors
    ApplyPower(Lpower, Rpower);
}

void MCEC::Drivetrain8::ApplyPower(int lPow, int rPow){
    curPowerL = MCEC::Lerp(curPowerL, lPow, 0.1f);
    curPowerR = MCEC::Lerp(curPowerR, rPow, 0.05f);

    _mR1.spin(vex::forward, curPowerR, vex::rpm);
    _mR2.spin(vex::forward, curPowerR, vex::rpm);
    _mR3.spin(vex::forward, curPowerR, vex::rpm);
    _mR4.spin(vex::forward, curPowerR, vex::rpm);

    _mL1.spin(vex::reverse, curPowerL, vex::rpm);
    _mL2.spin(vex::reverse, curPowerL, vex::rpm);
    _mL3.spin(vex::reverse, curPowerL, vex::rpm);
    _mL4.spin(vex::reverse, curPowerL, vex::rpm);
}


void MCEC::Drivetrain8::Stop(){
    curPowerL = 0;
    curPowerR = 0;

    _mR1.stop(vex::brakeType::brake);
    _mR2.stop(vex::brakeType::brake);
    _mR3.stop(vex::brakeType::brake);
    _mR4.stop(vex::brakeType::brake);

    _mL1.stop(vex::brakeType::brake);
    _mL2.stop(vex::brakeType::brake);
    _mL3.stop(vex::brakeType::brake);
    _mL4.stop(vex::brakeType::brake);
}

void MCEC::Drivetrain8::Rotate(int deg){
  while(inertial->heading() < deg){
    Drive(0, 30);
  }
}

void MCEC::Drivetrain8::Spin(float revs){
  _mR1.spinTo(revs, vex::rotationUnits::rev, false);
  _mR2.spinTo(revs, vex::rotationUnits::rev, false);
  _mR3.spinTo(revs, vex::rotationUnits::rev, false);
  _mR4.spinTo(revs, vex::rotationUnits::rev, false);

  _mL1.spinTo(revs, vex::rotationUnits::rev, false);
  _mL2.spinTo(revs, vex::rotationUnits::rev, false);
  _mL3.spinTo(revs, vex::rotationUnits::rev, false);
  _mL4.spinTo(revs, vex::rotationUnits::rev, true);
}

void MCEC::Controller::Set(){
    rStick.Set(
        controller.Axis1.position(), 
        controller.Axis2.position()
    );
    lStick.Set(
        controller.Axis4.position(), 
        controller.Axis3.position()
    );

    r1Down = controller.ButtonR1.pressing();
    l1Down = controller.ButtonL1.pressing();
    r2Down = controller.ButtonR2.pressing();
    l2Down = controller.ButtonL2.pressing();

    xDown = controller.ButtonX.pressing();
    yDown = controller.ButtonY.pressing();
    bDown = controller.ButtonB.pressing();
    aDown = controller.ButtonA.pressing();

    leftDown = controller.ButtonLeft.pressing();
    rightDown = controller.ButtonRight.pressing();
    downDown = controller.ButtonDown.pressing();
    upDown = controller.ButtonUp.pressing();
}