#ifndef CAR_BLDC_MOTOR_H
#define CAR_BLDC_MOTOR_H

#include <cstdint>
#include <array>
#include <car/math/measurement.h>
#include <car/math/angle.h>
#include <functional>

namespace car {
namespace bldc {


class Motor {
    public:
        Motor(std::array<uint8_t, 3> _pins_INH, std::array<uint8_t, 3> _pins_PWN, std::array<uint8_t, 3> _pins_IS, uint8_t _pin_CS, car::math::Direction encoder_direction) 
        : pins_INH(_pins_INH) 
        , pins_PWN(_pins_PWN)
        , pins_IS(_pins_IS)
        , pin_CS(_pin_CS) 
        , encoder_direction(encoder_direction) 
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
        car::math::Direction encoder_direction;
        bool coupled;  /// on true H-Bridges are enabled 

        char info[200];

        float target_PWN[3];
        car::math::Measurement<car::math::Angle14Bit> position_delta;
        car::math::Measurement<car::math::Angle14Bit> position;
        car::math::AngleDeg phase_angle;
        car::math::AngleDeg pwm_angle;
        float rpm;
        
        volatile uint32_t* timer_register[3];
        /**
         * function to update wheel state
         **/
        void update_position(){
            car::math::Measurement<car::math::Angle14Bit> measurement; // encoder measurement
            read(pin_CS, measurement.value(), measurement.stamp);      // read angle encoder 

            /// fix encoder mount angle
            if(encoder_direction == car::math::Direction::COUNTERCLOCKWISE) measurement.value() = measurement.value.max() - measurement.value();
            int16_t phase_measurement_max = measurement.value.max() / 11;
            int16_t phase_measurement = measurement.value() % phase_measurement_max;
            phase_angle.set(phase_measurement * phase_angle.max() / phase_measurement_max);
            position_delta = measurement - position;
            position = measurement;
            rpm = position_delta.value.get_norm() / position_delta.stamp_as_sec();
        }

        /**
         * function pointer to read moder angle encoder
         **/
        std::function<void (uint8_t cs, int16_t &value, uint32_t &stamp)> read;
};

}
}
#endif // CAR_BLDC_MOTOR_H
