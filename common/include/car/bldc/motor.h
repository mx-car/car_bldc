#ifndef CAR_BLDC_MOTOR_H
#define CAR_BLDC_MOTOR_H

#include <cstdint>
#include <array>
#include <car/math/measurement.h>
#include <car/math/angle.h>

namespace car {
namespace bldc {


class Motor {
    public:
        Motor(std::array<uint8_t, 3> _pins_INH, std::array<uint8_t, 3> _pins_PWN, std::array<uint8_t, 3> _pins_IS, uint8_t _pin_CS) 
        : pins_INH(_pins_INH) 
        , pins_PWN(_pins_PWN)
        , pins_IS(_pins_IS)
        , pin_CS(_pin_CS) 
        , coupled(false){
            sprintf(info, "INH: <%d, %d, %d>,  PWM: <%d, %d, %d>,  IS: <%d, %d, %d>, CS: %d",
            pins_INH[0], pins_INH[1], pins_INH[2], 
            pins_PWN[0], pins_PWN[1], pins_PWN[2], 
            pins_IS[0], pins_IS[1], pins_IS[2], pin_CS);
        }
        std::array<uint8_t, 3> pins_INH;
        std::array<uint8_t, 3> pins_PWN;
        std::array<uint8_t, 3> pins_IS;
        uint8_t pin_CS;
        bool coupled;  /// on true H-Bridges are enabled 

        char info[200];

        float target_PWN[3];
        car::math::Measurement<car::math::Angle14Bit> position_delta;
        car::math::Measurement<car::math::Angle14Bit> position_current;
        float rpm;
        float flux_offest;
        float flux_angle;
        
        volatile uint32_t* timer_register[3];
        void update(const car::math::Measurement<car::math::Angle14Bit> &motor_position){
            position_delta = motor_position - position_current;
            position_current = motor_position;
            rpm = position_delta.value.get_norm() / position_delta.stamp_as_sec();
        }
};

}
}
#endif // CAR_BLDC_MOTOR_H
