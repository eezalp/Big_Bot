#include "MCEC_Objects.h"
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
    float totalPower = joyX + joyY;
    // Handle case where joysticks are both pointed in the wrong axis
    if(totalPower == 0) return; 
    float py = (joyY / totalPower);
    float px = (joyX / totalPower);

    // Apply a scaling factor to each axis
    float xPower = (joyX * 6 * px);
    float yPower = (joyY * 6 * py);

    int Lpower = (xPower + yPower);
    int Rpower = (xPower - yPower);

    // Apply the power to the motors
    _mR1.spin(vex::forward, Rpower, vex::rpm);
    _mR2.spin(vex::forward, Rpower, vex::rpm);
    _mR3.spin(vex::forward, Rpower, vex::rpm);
    _mR4.spin(vex::forward, Rpower, vex::rpm);

    _mL1.spin(vex::reverse, Lpower, vex::rpm);
    _mL2.spin(vex::reverse, Lpower, vex::rpm);
    _mL3.spin(vex::reverse, Lpower, vex::rpm);
    _mL4.spin(vex::reverse, Lpower, vex::rpm);
}

void MCEC::Drivetrain8::Stop(){
    _mR1.stop();
    _mR2.stop();
    _mR3.stop();
    _mR4.stop();

    _mL1.stop();
    _mL2.stop();
    _mL3.stop();
    _mL4.stop();
}
