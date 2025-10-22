
#ifndef MCEC_Objects_h
#define MCEC_Objects_h
#include "vex.h"
namespace MCEC{
    
    class Drivetrain8{
        public:
            Drivetrain8(
                int32_t portR1, int32_t portR2, 
                int32_t portR3, int32_t portR4, 
                int32_t portL1, int32_t portL2,
                int32_t portL3, int32_t portL4
            ) : 
            _mR1(portR1, vex::ratio6_1), _mR2(portR2, vex::ratio6_1), _mR3(portR3, vex::ratio6_1), _mR4(portR4, vex::ratio6_1),
            _mL1(portL1, vex::ratio6_1), _mL2(portL2, vex::ratio6_1), _mL3(portL3, vex::ratio6_1), _mL4(portL4, vex::ratio6_1)
            {}
            // @param joyX: input from -100 to 100
            // @param joyY: input from -100 to 100
            void Drive(int joyX, int joyY);
            void ApplyPower(int lPow, int rPow);
            void Stop();
            float curPowerR, curPowerL;
        private:
            vex::motor _mR1, _mR2, _mR3, _mR4;
            vex::motor _mL1, _mL2, _mL3, _mL4;
    };
    
    struct Joystick{
        int x, y;
        void Set(int _x, int _y){
            SetX(_x);
            SetY(_y);
        }
        void SetX(int _x){
            x = _x;
        }
        void SetY(int _y){
            y = _y;
        }
    };

    float Lerp(float a, float b, float t);
}
#endif