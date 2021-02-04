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
        void update_PWM(volatile uint32_t* timer_register[3], float target[3]);    
        void couple(Motor *motor);
        void decouple(Motor *motor);

    protected:
        void init_inhibit(Motor *motor);
        void init_timer();     
        volatile uint32_t* get_register_pwm(uint8_t pin);
        Motor *motor0_;
        Motor *motor1_;
        uint32_t timer_modulo_;

};
}
}
#endif // CAR_BLDC_DRIVER_H
