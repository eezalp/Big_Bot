
#ifndef MCEC_Objects_h
#define MCEC_Objects_h
#include "vex.h"

#include <functions.h>

#define IS_RED(color)  (0xFF0000 & (uint32_t)detectedColor) >> 16 == 0xff
#define IS_BLUE(color) (0x0000FF & (uint32_t)detectedColor) == 0xff

#define ABS(num) ((num < 0) ? (num * -1) : num)

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
            void DriveDist(int dL, int dR);
            void ApplyPower(int lPow, int rPow);
            void Stop();
            void UpdateHeading();
            void Spin(float revs);
            void Rotate(int);
            void SetInertial(vex::inertial* _inertial);
            float curPowerR, curPowerL;
        private:
            vex::motor _mR1, _mR2, _mR3, _mR4;
            vex::motor _mL1, _mL2, _mL3, _mL4;
            vex::inertial* inertial;
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

        bool isMoved(){
          return (x != 0 || y != 0);
        }
    };

    class Button{
        public:
            void SetValue(bool);
            void SetOnPress(std::function<void()>);
            void SetOnRelease(std::function<void()>);
        private:
            bool _last, _cur;
            std::function<void()> _onPress, _onRelease;
    };

    class Controller{
      bool aDown, bDown, xDown, yDown;
      bool r1Down, r2Down, l1Down, l2Down;
      bool leftDown, rightDown, upDown, downDown;

      Joystick lStick, rStick;
      vex::controller controller;
      Controller() : controller() {}
      void Set();
    };

    float Lerp(float a, float b, float t);
}
#endif