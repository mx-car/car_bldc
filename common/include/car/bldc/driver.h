#ifndef CAR_BLDC_DRIVER_H
#define CAR_BLDC_DRIVER_H

#include <cstdint>

#include <car/bldc/motor.h>

namespace car {
namespace bldc {

class Driver {
    static const uint32_t PWMFrequency = 20000;
    public:
        Driver();
        void init(Motor *motor0, Motor *motor1 = NULL);   
        void update_PWM(Motor *motor, float power = 1.0);    
        void couple(Motor *motor);
        void decouple(Motor *motor);

    protected:
        void init_inhibit(Motor *motor);
        void init_timer();     

        Motor *motor0_;
        Motor *motor1_;
        uint16_t *timer_modulo_;
};
}
}
#endif // CAR_BLDC_DRIVER_H
