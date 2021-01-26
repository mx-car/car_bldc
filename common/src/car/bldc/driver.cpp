
#include <cstdlib>
#include <cstring>


#include <Arduino.h>
#include <car/bldc/driver.h>

using namespace car::bldc;



Driver::Driver() 
: motor0_(NULL) 
, motor1_(NULL)  {
}

void Driver::init(Motor *motor0, Motor *motor1){
    motor0_ = motor0;
    motor1_ = motor1;
    if(motor0_) {
        init_inhibit(motor0_);
    }
    if(motor1_) {
        init_inhibit(motor1_);
    }
    init_timer();
}

void Driver::init_inhibit(Motor *motor){
    pinMode(motor->pins_INH[0], OUTPUT);
    pinMode(motor->pins_INH[1], OUTPUT);
    pinMode(motor->pins_INH[2], OUTPUT);
    motor->timer_register[0] = get_register_pwm(motor->pins_PWN[0]);
    motor->timer_register[1] = get_register_pwm(motor->pins_PWN[1]);
    motor->timer_register[2] = get_register_pwm(motor->pins_PWN[2]);

}

void Driver::couple(Motor *motor){
    digitalWriteFast(motor->pins_INH[0], HIGH);
    digitalWriteFast(motor->pins_INH[1], HIGH);
    digitalWriteFast(motor->pins_INH[2], HIGH);
    motor->coupled = true;
}

void Driver::decouple(Motor *motor){
    digitalWriteFast(motor->pins_INH[0], LOW);
    digitalWriteFast(motor->pins_INH[1], LOW);
    digitalWriteFast(motor->pins_INH[2], LOW);
    motor->coupled = false;
}

uint32_t* Driver::get_register_pwm(uint8_t pin){
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

void Driver::update_PWM(Motor *motor, float power) {

    uint32_t power_promille = power * 1000.0;
    uint16_t max_count = power_promille * this->timer_modulo_ / 1000;
    //uint16_t max_count = this->timer_modulo_;
    for(int i = 0; i < 3; i++){
        uint32_t v = motor->target_PWN[i] * max_count;
        if( v < 150) v = 0;  
        *motor->timer_register[i] = v; 
    }
    /*
        FTM0_C3V = timer_count[0];  // Teensy pin 10 -> FTM0_CH3pardom
        FTM0_C1V = timer_count[1];  // Teensy pin 22 (A8) -> FTM0_CH0
        FTM0_C0V = timer_count[2];  // Teensy pin 23 (A9) -> FTM0_CH1

        FTM0_C7V = this->timer_modulo_ / 2;  // Teensy pin 5 -> FTM0_CH7
        FTM0_C4V = this->timer_modulo_ / 2;  // Teensy pin  6 -> FTM0_CH4
        FTM0_C2V = this->timer_modulo_ / 2;  // Teensy pin 9 -> FTM0_CH2
        */
}