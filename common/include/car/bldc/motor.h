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
    private:
        int16_t nr_of_coils_;                       /// number of coils;
        int16_t phase_measurement_max_;             /// maximum number of encoder clicks per phase

        std::function<void (uint8_t cs, int16_t &value, uint32_t &stamp)> read_encoder; /// function pointer to read moder angle encoder
        std::function<void (volatile uint32_t* timer_register[3], float target[3])> update_pwm; /// function pointer to set pwm values
    public:
        Motor(std::array<uint8_t, 3> _pins_INH, std::array<uint8_t, 3> _pins_PWN, std::array<uint8_t, 3> _pins_IS, uint8_t _pin_CS, car::math::Direction encoder_direction) 
        : pins_INH(_pins_INH) 
        , pins_PWN(_pins_PWN)
        , pins_IS(_pins_IS)
        , pin_CS(_pin_CS) 
        , encoder_direction(encoder_direction) 
        , coupled(false)
        , target_power(0){
        }

        /**
        * init motor parameters
        * @param nr_of_coils number of coils;
        * @param phase_offset phase offset between pwm signal and phase_angle in encoder clicks
        **/
        void init(int nr_of_coils, const car::math::AngleDeg &phase_offset, std::function<void (uint8_t cs, int16_t &value, uint32_t &stamp)> fnc_read_encoder, std::function<void (volatile uint32_t* timer_register[3], float target[3])>  fnc_update_pwm){
            nr_of_coils_ = nr_of_coils;
            phase_offset_ = phase_offset;
            phase_offset_.normalize();
            read_encoder = fnc_read_encoder;
            update_pwm = fnc_update_pwm;
            phase_measurement_max_ = car::math::Angle14Bit::max() / nr_of_coils_;
        }

        std::array<uint8_t, 3> pins_INH;
        std::array<uint8_t, 3> pins_PWN;
        std::array<uint8_t, 3> pins_IS;
        uint8_t pin_CS;
        car::math::Direction encoder_direction;
        bool coupled;  /// on true H-Bridges are enabled 
        float target_power;

        float target_PWN[3];
        car::math::Measurement<car::math::Angle14Bit> measurement; // encoder measurement
        car::math::Measurement<car::math::Angle14Bit> position_control;
        car::math::Measurement<car::math::Angle14Bit> position_delta;
        car::math::AngleDeg phase_angle;
        car::math::AngleDeg pwm_angle;
        car::math::AngleDeg phase_offset_;          /// phase offset between pwm signal and phase_angle in encoder clicks
        float rpm;
        
        volatile uint32_t* timer_register[3];
        /**
         * function to update wheel state
         **/
        void update(){
            read_encoder(pin_CS, measurement.value(), measurement.stamp);      // read angle encoder 
            /// fix encoder counting direction
            if(encoder_direction == car::math::Direction::COUNTERCLOCKWISE) measurement.value() = measurement.value.max() - measurement.value();


            /// compute encoder angle per coil    
            int16_t phase_measurement = measurement.value() % phase_measurement_max_;
            phase_angle.set(phase_measurement * phase_angle.max() / phase_measurement_max_);
            phase_angle.normalize();

            /// fix flux offest
            pwm_angle = phase_angle - phase_offset_;
            pwm_angle.normalize();

            target_PWN[0] = (pwm_angle +   0).get_cos() * 0.5 + 0.5 * target_power;
            target_PWN[1] = (pwm_angle + 120).get_cos() * 0.5 + 0.5 * target_power;
            target_PWN[2] = (pwm_angle + 240).get_cos() * 0.5 + 0.5 * target_power;


            /// compute delta since last update  
            update_pwm(timer_register, target_PWN);
        }
        /**
         * function to update wheel state
         **/
        void control(){
            /// compute delta since last update  
            position_delta.stamp = measurement.stamp - position_control.stamp;
            position_delta.value = car::math::Angle14Bit::difference(measurement.value, position_control.value);
            position_control = measurement;
            rpm = position_delta.value.get_norm() / position_delta.stamp_as_sec();
        }
    
};

}
}
#endif // CAR_BLDC_MOTOR_H
