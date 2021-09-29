/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_BLDC_MOTOR_H
#define CAR_BLDC_MOTOR_H

#include <cstdint>
#include <array>
#include <car/math/value.h>
#include <car/math/angle.h>
#include <functional>

namespace car
{
    namespace bldc
    {

        class Motor
        {
            friend class Driver;

        public:
            /**
             * Constructor
             **/
            Motor(std::array<uint8_t, 3> _pins_INH, std::array<uint8_t, 3> _pins_PWN, std::array<uint8_t, 3> _pins_IS, uint8_t _pin_CS, car::math::Direction encoder_direction)
                : pins_INH_(_pins_INH), pins_PWN_(_pins_PWN), pins_IS_(_pins_IS), pin_CS_(_pin_CS), encoder_direction_(encoder_direction), target_power_(0)
            {
            }

            /**
             * init motor parameters
             * @param nr_of_coils number of coils;
             * @param phase_offset phase offset between pwm signal and phase_angle in encoder clicks
             **/
            void init(int16_t nr_of_coils, const std::array<car::math::AngleDeg, 2> &phase_offsets, std::function<void(uint8_t cs, int16_t &value, uint32_t &stamp)> fnc_read_encoder, std::function<void(std::array<volatile uint32_t *, 3>, const std::array<float, 3> &)> fnc_update_pwm, std::function<void(const std::array<uint8_t, 3> &, bool)> fnc_couple_pwm)
            {
                nr_of_coils_ = nr_of_coils;
                phase_offsets_ = phase_offsets;
                phase_offsets_[0].normalize();
                phase_offsets_[1].normalize();
                fnc_read_encoder_ = fnc_read_encoder;
                fnc_update_pwm_ = fnc_update_pwm;
                fnc_couple_pwm_ = fnc_couple_pwm;
                phase_measurement_max_ = car::math::Angle14Bit::max() / nr_of_coils_;
            }

            /**
             * returns the last postion measurement (pwm update)
             * @return position 
             **/ 
            const car::math::Value<car::math::Angle14Bit> &position() const
            {
                return measurement_;
            }

            /**
             * returns speed
             * @return speed in rotation per secound 
             **/ 
            float rps() const
            {
                return rps_control_.value;
            }

            /**
             * returns phase offsets for 
             * @return speed in rotation per secound 
             **/ 
            const std::array<car::math::AngleDeg, 2> &phase_offset() const
            {
                return phase_offsets_;
            }

            /**
             * returns speed with time stamp
             * @param speed in rotation per secound 
             **/ 
            void rps(car::math::Value<float> &speed) const
            {
                speed = rps_control_;
            }

            /**
             * returns chip select pin on the position encoder
             * @return  chip select pin
             **/ 
            uint8_t pin_encoder_cs() const
            {
                return pin_CS_;
            }

            /**
             * returns inhibitor pins on the h-bridges
             * @return  inhibitor pins
             **/
            const std::array<uint8_t, 3> &pin_inhibitor() const
            {
                return pins_INH_;
            }

            /**
             * set power value
             * @param power
             **/
            void set_power(float power)
            {
                if(power > +1)  power = 1;
                if(power < -1) power = -1;
                target_power_ = power;
            }

            car::math::AngleDeg compute_phase_angle(){
                /// compute encoder angle per coil
                int16_t phase_measurement = measurement_.value() % phase_measurement_max_;
                car::math::AngleDeg phase_angle(phase_measurement * phase_angle.max() / phase_measurement_max_);

                /// fix flux offest
                phase_angle += phase_offsets_[target_power_ < 0];

                return phase_angle.normalize();
            }
            
            /**
             * function to set sinoid pwm angle
             * @param pwm_angle
             **/
            void set_pwm(car::math::AngleDeg &pwm_angle)
            {
                std::array<float, 3> target_PWN;
                pwm_angle.normalize();
                float scale = 0.5 * fabs(target_power_);
                target_PWN[0] = (pwm_angle +   0).get_cos() * scale  + 0.5;
                target_PWN[1] = (pwm_angle + 120).get_cos() * scale + 0.5;
                target_PWN[2] = (pwm_angle + 240).get_cos() * scale + 0.5;

                /// compute delta since last update
                fnc_update_pwm_(pwm_timer_register_, target_PWN);
            }

            /**
             * function to update the pwm for a space vector modulation
             * @param apply_pwm on true it will also apply the computed pwm values
             * @see 
             **/
            void update_pwm(bool apply_pwm = true)
            {
                fnc_read_encoder_(pin_CS_, measurement_.value(), measurement_.stamp); // read angle encoder
                /// fix encoder counting direction
                if (encoder_direction_ == car::math::Direction::COUNTERCLOCKWISE){
                    measurement_.value() = measurement_.value.max() - measurement_.value();
                }


                car::math::AngleDeg pwm_angle = compute_phase_angle();

                if(apply_pwm) set_pwm(pwm_angle);
            }


            /**
             * function apply motor controls and to comptue speed
             * @see Motor::rps()
             **/
            void update_control()
            {
                /// compute delta since last update
                position_delta_.stamp = measurement_.stamp - position_control_.stamp;
                position_delta_.value = car::math::Angle14Bit::difference(measurement_.value, position_control_.value);
                position_control_ = measurement_;
                rps_control_.stamp = measurement_.stamp;
                rps_control_.value = position_delta_.value.get_norm() / position_delta_.stamp_as_sec();
            }

            /**
             * function couple or decouple motor
             * @param on couble on true
             **/
            void couple(bool on)
            {
                fnc_couple_pwm_(pins_INH_, on);
            }

        private:
            std::array<uint8_t, 3> pins_INH_;                                                                      /// H-Bridge inhibitor pins
            std::array<uint8_t, 3> pins_PWN_;                                                                      /// H-Bridge power (pwm) pins
            std::array<uint8_t, 3> pins_IS_;                                                                       /// H-Bridge sense pins
            uint8_t pin_CS_;                                                                                       /// SPI chip select pin
            car::math::Direction encoder_direction_;                                                               /// encoder counting direction
            int16_t nr_of_coils_;                                                                                  /// number of coils;
            int16_t phase_measurement_max_;                                                                        /// maximum number of encoder clicks per phase
            float target_power_;                                                                                   /// power on the pwm modulation
            car::math::Value<car::math::Angle14Bit> measurement_;                                                  /// encoder measurement update on every pwm update
            car::math::Value<car::math::Angle14Bit> position_control_;                                             /// motor position, update on every control update
            car::math::Value<car::math::Angle14Bit> position_delta_;                                               /// motor position change since last control update
            car::math::Value<float> rps_control_;                                                                  /// motor speed in rotiations per second update on every control update
            std::array<car::math::AngleDeg, 2> phase_offsets_;                                                     /// phase offset between pwm signal and phase_angle in encoder clicks
            std::array<volatile uint32_t *, 3> pwm_timer_register_;                                                /// timer registers to update on every pwm update for to directions
            std::function<void(uint8_t cs, int16_t &value, uint32_t &stamp)> fnc_read_encoder_;                    /// function pointer to read moder angle encoder
            std::function<void(std::array<volatile uint32_t *, 3>, const std::array<float, 3> &)> fnc_update_pwm_; /// function pointer to set pwm register values
            std::function<void(const std::array<uint8_t, 3> &, bool)> fnc_couple_pwm_;                             /// function to couple or decouble (inhibitor) the motor
        };

    } // namespace bldc
} // namespace car
#endif // CAR_BLDC_MOTOR_H
