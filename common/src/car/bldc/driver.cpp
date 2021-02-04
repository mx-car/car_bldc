
#include <cstdlib>
#include <cstring>


#include <Arduino.h>
#include <car/bldc/driver.h>

using namespace car::bldc;



Driver::Driver()  {
}

void Driver::init(Motor *motor0, Motor *motor1){
    if(motor0) {
        init_inhibit(motor0);
    }
    if(motor1) {
        init_inhibit(motor1);
    }
    init_timer();
}

void Driver::init_inhibit(Motor *motor){
    pinMode(motor->pins_INH_[0], OUTPUT);
    pinMode(motor->pins_INH_[1], OUTPUT);
    pinMode(motor->pins_INH_[2], OUTPUT);
    motor->pwm_timer_register_[0] = get_register_pwm(motor->pins_PWN_[0]);
    motor->pwm_timer_register_[1] = get_register_pwm(motor->pins_PWN_[1]);
    motor->pwm_timer_register_[2] = get_register_pwm(motor->pins_PWN_[2]);

}

void Driver::couple_pwm(const std::array<uint8_t, 3> &pins_INH, bool on){
    if(on){
        digitalWriteFast(pins_INH[0], HIGH);
        digitalWriteFast(pins_INH[1], HIGH);
        digitalWriteFast(pins_INH[2], HIGH);
    } else {
        digitalWriteFast(pins_INH[0], LOW);
        digitalWriteFast(pins_INH[1], LOW);
        digitalWriteFast(pins_INH[2], LOW);
    }
}

volatile uint32_t* Driver::get_register_pwm(uint8_t pin){
    if(pin == 10) return &FTM0_C3V; // Teensy pin 10 -> FTM0_CH3pardom
    if(pin == 22) return &FTM0_C1V; // Teensy pin 22 (A8) -> FTM0_CH0
    if(pin == 23) return &FTM0_C0V; // Teensy pin 23 (A9) -> FTM0_CH1
    if(pin == 5) return &FTM0_C7V;  // Teensy pin 5 -> FTM0_CH7
    if(pin == 6) return &FTM0_C4V;  // Teensy pin  6 -> FTM0_CH4
    if(pin == 9) return &FTM0_C2V;  // Teensy pin 9 -> FTM0_CH2
    return NULL;
}

void Driver::init_timer(){
    
    FTM0_SC = 0; // required for other setup

    FTM0_CONF = 0xC0; //set up BDM in 11, FTM Counter functional -> 0000 1101 0000 0000
    FTM0_FMS = 0x00; //clear the WPEN (Write protection disabled) so that WPDIS is set in FTM0_MODE

    //FTM0_MODE|=0x05; // 0000 0101
    // This register contains the global enable bit for FTM-specific features and the control bits
    //used to configure:
    FTM0_MODE = 0b00000110; // 00000111

    //The Modulo register contains the modulo value for the FTM counter. After the FTM
    //counter reaches the modulo value, the overflow flag (TOF) becomes set at the next clock
    this->timer_modulo_ = (F_BUS / PWMFrequency) / 2;
    FTM0_MOD = this->timer_modulo_;
    // FTM0_C6SC |= FTM_CSC_CHIE

    /// Motor 0
    FTM0_C3SC = 0b00101000;
    FTM0_C3V = 0; //50%
    PORTC_PCR4 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 10 -> FTM0_CH3

    FTM0_C0SC = 0b00101000;
    FTM0_C0V = 0; //50%
    PORTC_PCR1 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 22 (A8) -> FTM0_CH0

    FTM0_C1SC = 0b00101000;
    FTM0_C1V = 0; //50%
    PORTC_PCR2 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 23 (A9) -> FTM0_CH1

    /// Motor 1
    FTM0_C7SC = 0b00101000;
    FTM0_C7V = 0; //50%
    PORTD_PCR7 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 5 (A8) -> FTM0_CH7

    FTM0_C4SC = 0b00101000;
    FTM0_C4V = 0; //50%
    PORTD_PCR4 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 6 -> FTM0_CH4

    FTM0_C2SC = 0b00101000;
    FTM0_C2V = 0;
    PORTC_PCR3 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 9 -> FTM0_CH2
    

    FTM0_CNTIN = 0x00;

    FTM0_SC = 0b01101000; //CPWM MODE 0x48 EPWM -> 0x68 0110 1000

    FTM0_SC = 0b01101000; //CPWM MODE 0x48 EPWM -> 0x68 0110 1000 -> TOF interrupt disabled
    //FTM0_SC = 0b00101000; //CPWM MODE 0x48 EPWM -> 0x68 0110 1000
}

void Driver::update_pwm(std::array<volatile uint32_t*, 3>  timer_register, const std::array<float,3> &target){

    *timer_register[0] = (target[0] * (float) this->timer_modulo_);  
    *timer_register[1] = (target[1] * (float) this->timer_modulo_);  
    *timer_register[2] = (target[2] * (float) this->timer_modulo_); 

} 