#include "MCEC_Objects.h"

float MCEC::Lerp(float a, float b, float t){
    return ((1 - t) * a) + (b * t);
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

    _mR1.stop();
    _mR2.stop();
    _mR3.stop();
    _mR4.stop();

    _mL1.stop();
    _mL2.stop();
    _mL3.stop();
    _mL4.stop();
}
