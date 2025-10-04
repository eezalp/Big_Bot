#include "MCEC_Objects.h"

void MCEC::Drivetrain8::Drive(int powerX, int powerY){
    int Lpower = (powerX / 2) + (powerY / 2);
    int Rpower = (powerX * 3) - (powerY * 3);
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
