/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
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

std::array<car::math::AngleDeg, 2> motor0_flux_offset( {-120   , -120-90});
std::array<car::math::AngleDeg, 2> motor1_flux_offset( {-120+90, -120   });
uint32_t timer_count;
car::time::CycleRate cycle_pwm(5);
car::time::CycleRate cycle_control(100);

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

  motor0.init(11, motor0_flux_offset, std::bind(&car::encoder::Encoder::read, encoder, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), 
  std::bind(&car::bldc::Driver::update_pwm, &driver, std::placeholders::_1, std::placeholders::_2),
  std::bind(&car::bldc::Driver::couple_pwm, &driver, std::placeholders::_1, std::placeholders::_2));
  motor0.couple(true);
  motor0.set_power(-0.5);

  motor1.init(11, motor1_flux_offset, 
  std::bind(&car::encoder::Encoder::read, encoder, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), 
  std::bind(&car::bldc::Driver::update_pwm, &driver, std::placeholders::_1, std::placeholders::_2),
  std::bind(&car::bldc::Driver::couple_pwm, &driver, std::placeholders::_1, std::placeholders::_2));
  motor1.couple( true);
  motor1.set_power(-0.5);
}
// the loop routine runs over and over again forever:
void loop()
{
  if (cycle_pwm.passed()){
    motor0.update_pwm();
    motor1.update_pwm();
  }
  if (cycle_control.passed())
  {
    motor0.update_control();
    motor1.update_control();
    sprintf(msg, "%4.3frps, %4.3frps", motor0.rps(), motor1.rps());
    Serial.println(msg);
  }
  loop_count++;
}