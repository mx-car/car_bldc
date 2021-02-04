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
car::math::AngleDeg flux_offset_int = 65;
uint32_t timer_count;
car::time::CycleRate cycle_pwm(2);
car::time::CycleRate cycle_control(100);
car::time::CycleRate cycle_verify(200);

char msg[0xff];
// the setup routine runs once when you press reset:
void setup()
{
  car::math::AngleDeg::init();
  driver.init(&motor0, &motor1);
  encoder = new car::encoder::AS5048A(std::array<uint8_t, 2>({motor0.pin_CS, motor1.pin_CS}), SPI_SCK);

  Serial.begin(115200); /// init serial
  while (!Serial)
    ;
  Serial.println("BLDC");

  driver.couple(&motor0);
  motor0.init(11, flux_offset_int, std::bind(&car::encoder::Encoder::read, encoder, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), std::bind(&car::bldc::Driver::update_PWM, &driver, std::placeholders::_1, std::placeholders::_2));
  motor0.target_power = 0.5;

  driver.couple(&motor1);
  motor1.init(11, -flux_offset_int(), std::bind(&car::encoder::Encoder::read, encoder, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), std::bind(&car::bldc::Driver::update_PWM, &driver, std::placeholders::_1, std::placeholders::_2));
  motor1.target_power = 0.5;
}
// the loop routine runs over and over again forever:
void loop()
{
  if (cycle_pwm.passed()){
    motor0.update();
    motor1.update();
  }
  if (cycle_control.passed())
  {
    motor0.control();
    motor1.control();
    sprintf(msg, "%4.3frpm, %4.3frpm", motor0.rpm, motor1.rpm);
    Serial.println(msg);
  }
  loop_count++;
}