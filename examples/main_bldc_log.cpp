/*
  ENCODER
  @author Markus Bader
 */

#include <Arduino.h>
#include <vector>
#include <array>
#include <vector>
#include <numeric>
#include <limits> // std::numeric_limits
#include <algorithm>

#include <car/bldc/driver.h>
#include <car/encoder/encoder_as5048a.h>
#include <car/math/angle.h>
#include <car/time/cycle_rate.h>

int SPI_SCK = 13;
int SPI_CS0 = 2;
int SPI_CS1 = 14;

car::encoder::Encoder *encoder;
car::bldc::Driver driver;
car::bldc::Motor motor0(std::array<uint8_t, 3>({33, 26, 31}),
                        std::array<uint8_t, 3>({10, 22, 23}),
                        std::array<uint8_t, 3>({A15, A16, A17}), 2, car::math::Direction::COUNTERCLOCKWISE);
car::bldc::Motor motor1(std::array<uint8_t, 3>({28, 8, 25}),
                        std::array<uint8_t, 3>({5, 6, 9}),
                        std::array<uint8_t, 3>({A15, A16, A17}), 14, car::math::Direction::COUNTERCLOCKWISE);
int loop_count = 0;

int32_t resolution_phase;
car::math::AngleDeg pwm_angle(0);
std::array<car::math::AngleDeg, 2> motor0_flux_offset( {-65   , -65-90});
std::array<car::math::AngleDeg, 2> motor1_flux_offset( {-80+90, -80   });
uint32_t timer_count;
car::time::CycleRate cycle_pwm(2);
car::time::CycleRate cycle_control(10);
car::time::CycleRate cycle_compute(200);

struct Log{
  car::math::AngleDeg pwm_angle;
  car::math::Value<car::math::Angle14Bit> m0_position;
  car::math::Value<car::math::Angle14Bit> m1_position;
  car::math::AngleDeg motor0_angle;
  car::math::AngleDeg motor1_angle;
};

size_t log_idx = 0;
std::array<Log, 360*2> LogEntries;

char msg[0xff];
// the setup routine runs once when you press reset:
void setup()
{
  car::math::AngleDeg::init();
  driver.init(&motor0, &motor1);
  encoder = new car::encoder::AS5048A(std::array<uint8_t, 2>({motor0.pin_encoder_cs(), motor1.pin_encoder_cs()}), SPI_SCK);

  Serial.begin(115200); /// init serial
  while (!Serial)
    ;
  Serial.println("BLDC");

  motor0.init(11, motor0_flux_offset, 
  std::bind(&car::encoder::Encoder::read, encoder, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), 
  std::bind(&car::bldc::Driver::update_pwm, &driver, std::placeholders::_1, std::placeholders::_2),
  std::bind(&car::bldc::Driver::couple_pwm, &driver, std::placeholders::_1, std::placeholders::_2));
  motor0.couple(true);
  motor0.set_power(0.5);

  motor1.init(11, motor1_flux_offset, 
  std::bind(&car::encoder::Encoder::read, encoder, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), 
  std::bind(&car::bldc::Driver::update_pwm, &driver, std::placeholders::_1, std::placeholders::_2),
  std::bind(&car::bldc::Driver::couple_pwm, &driver, std::placeholders::_1, std::placeholders::_2));
  motor1.couple( true);
  motor1.set_power(0.5);
}
// the loop routine runs over and over again forever:
void loop()
{
  if (cycle_pwm.passed()){
    motor0.update_pwm(false);
    motor0.set_pwm(pwm_angle);
    motor1.update_pwm(false);
    motor1.set_pwm(pwm_angle);
    pwm_angle -= 1;
    pwm_angle.normalize();
    if(pwm_angle() % 5 == 0){
      Log &log = LogEntries[log_idx++];
      log.m0_position = motor0.position();
      log.m1_position = motor1.position();
      log.pwm_angle = pwm_angle;
      log.motor0_angle = motor0.compute_phase_angle();
      log.motor1_angle = motor1.compute_phase_angle();
    }
  }
  if (cycle_control.passed()){
  }
  if(log_idx == LogEntries.size()){
    for(const auto &log: LogEntries){
      sprintf(msg, "%8lu, %5d, %5d, %5d, %5d, %5d", log.m0_position.stamp, log.pwm_angle(), log.m0_position.value(), log.motor0_angle(), log.m1_position.value(), log.motor1_angle());
      Serial.println(msg);
    }
    Serial.println("----");
    motor0.couple( false);
    motor1.couple( false);
    log_idx = 0;

  }

  loop_count++;
}