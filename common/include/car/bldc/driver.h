/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 */

#ifndef CAR_BLDC_DRIVER_H
#define CAR_BLDC_DRIVER_H

#include <cstdint>

#include <car/bldc/motor.h>

namespace car {
namespace bldc {

class Driver {
    public:
        static const uint32_t PWMFrequency = 15000;
        Driver();
        void init(Motor *motor0, Motor *motor1 = NULL);   
        void update_pwm(std::array<volatile uint32_t*, 3>, const std::array<float,3> &target); 
        void couple_pwm(const std::array<uint8_t, 3> &pins_INH, bool on);    
    protected:
        void init_inhibit(Motor *motor);
        void init_timer();     
        volatile uint32_t* get_register_pwm(uint8_t pin);
        uint32_t timer_modulo_;


};
}
}
#endif // CAR_BLDC_DRIVER_H
