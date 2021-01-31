/*
  ENCODER
  @author Markus Bader
 */

#include <Arduino.h>
#include <vector>
#include <array>

#include <car/bldc/driver.h>
#include <car/encoder/encoder_as5048a.h>
#include <car/math/angle.h>

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

char txt[200];
struct Measurment
{
  car::math::Measurement<car::math::Angle14Bit> encoder_motor;
  car::math::AngleDeg motor_phase_angle;
  car::math::AngleDeg svm_phase_angle;
};
int32_t resolution_phase;
uint32_t timer_count;
Measurment m;
static const size_t buffer_size_max = 1000;
Measurment measurments[buffer_size_max];;
size_t buffer_size = 0;
// the setup routine runs once when you press reset:
void setup()
{
  car::math::AngleDeg::init();
  m.svm_phase_angle = 0;
  driver.init(&motor0, &motor1);
  encoder = new car::encoder::AS5048A(std::array<uint8_t, 2>({motor0.pin_CS, motor1.pin_CS}), SPI_SCK);

  Serial.begin(115200); /// init serial
  while (!Serial) ;
  Serial.println("BLDC");
  //Serial.println(motor0.info);
  //Serial.println(motor1.info);
  timer_count = (F_BUS / driver.PWMFrequency) / 2  ;
  
  driver.couple(&motor0);
  *motor0.timer_register[0] = 0;
  *motor0.timer_register[1] = 0;
  *motor0.timer_register[2] = 0;
  
  driver.decouple(&motor1);
  *motor1.timer_register[0] = 0;
  *motor1.timer_register[1] = 0;
  *motor1.timer_register[2] = 0;
  
  motor0.read = std::bind(&car::encoder::Encoder::read, encoder, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  motor1.read = std::bind(&car::encoder::Encoder::read, encoder, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  resolution_phase = encoder->resolution() / 11;
}
// the loop routine runs over and over again forever:
void loop()
{
  motor0.update_position();
  //car::math::Angle encoder_motor_angle = ((m.encoder_motor.value * 360l) / encoder->resolution());
  m.motor_phase_angle = motor0.phase_angle;
  m.svm_phase_angle = m.motor_phase_angle - 65;
  //m.svm_phase_angle.normalize();


  motor0.target_PWN[0] = (m.svm_phase_angle +   0).get_cos() * 0.5 + 0.5;
  motor0.target_PWN[1] = (m.svm_phase_angle + 120).get_cos() * 0.5 + 0.5;
  motor0.target_PWN[2] = (m.svm_phase_angle + 240).get_cos() * 0.5 + 0.5;
  driver.update_PWM(&motor0, 1.);


  motor1.target_PWN[0] = (m.svm_phase_angle +   0).get_cos() * 0.5 + 0.5;
  motor1.target_PWN[1] = (m.svm_phase_angle + 120).get_cos() * 0.5 + 0.5;
  motor1.target_PWN[2] = (m.svm_phase_angle + 240).get_cos() * 0.5 + 0.5;
  //driver.update_PWM(&motor1, 0.5);

  if (true && (loop_count % 100 == 0))
  {
    measurments[buffer_size++] = m;
    if(true){
      if(loop_count == 0)  Serial.println("# m.encoder_motor.stamp, m.encoder_motor.value, m.motor_phase_angle, m.svm_phase_angle()");
      sprintf(txt, "%8lu, %5u, %5d, %5d, %8lu, %5u, %5.4f", m.encoder_motor.stamp, motor0.position.value(), m.motor_phase_angle(), m.svm_phase_angle(), motor0.position_delta.stamp, motor0.position_delta.value(), motor0.rpm);
      Serial.println(txt);
    }
  }

  if (buffer_size == buffer_size_max)
  {    
    buffer_size = 0;

    if(false) {
      delay(10);
      Serial.println("decouple");
      driver.decouple(&motor0);
      for (size_t i = 0; i < buffer_size_max; i++)
      {
        m = measurments[i];
        if(loop_count == 0)  Serial.println("# m.encoder_motor.stamp, m.encoder_motor.value, m.motor_phase_angle, m.svm_phase_angle()");
        sprintf(txt, "%8lu, %5u, %5d, %5d", m.encoder_motor.stamp, motor0.position.value(), m.motor_phase_angle(), m.svm_phase_angle());
      
        Serial.println(txt);
        delay(1);
      }
    }
  }
  delay(1); // wait for a second
  loop_count++;
}