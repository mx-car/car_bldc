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
                        std::array<uint8_t, 3>({A15, A16, A17}), 2);
car::bldc::Motor motor1(std::array<uint8_t, 3>({28, 8, 25}),
                        std::array<uint8_t, 3>({5, 6, 9}),
                        std::array<uint8_t, 3>({A15, A16, A17}), 14);
int loop_count = 0;

char txt[200];
struct Measurment
{
  uint32_t t;
  car::math::Angle encoder_motor_angle;
  car::math::Angle encoder_phase_angle;
  car::math::Angle svm_phase_angle;
};

bool record = true;
Measurment m;
static const size_t buffer_size_max = 720;
Measurment measurments[buffer_size_max];
size_t buffer_size = 0;
// the setup routine runs once when you press reset:
void setup()
{
  car::math::Angle::init();
  m.svm_phase_angle = 0;
  driver.init(&motor0, &motor1);
  driver.couple(&motor0);
  encoder = new car::encoder::AS5048A(SPI_SCK, std::array<uint8_t, 2>({motor0.pin_CS, motor1.pin_CS}));

  Serial.begin(115200); /// init serial
  while (!Serial) ;
  Serial.println("BLDC");
  //Serial.println(motor0.info);
  //Serial.println(motor1.info);
}

// the loop routine runs over and over again forever:
void loop()
{
  int32_t ticks_motor_angle = encoder->get_raw(car::encoder::cs0);
  int32_t resolution_phase = encoder->resolution() / 11;
  int32_t ticks_phase_angle = ticks_motor_angle % resolution_phase;
  m.encoder_motor_angle = ((ticks_motor_angle * 360l) / encoder->resolution());
  m.encoder_phase_angle = ((ticks_phase_angle * 360l) / resolution_phase);
  m.svm_phase_angle += 1;


  motor0.target_PWN[0] = (m.svm_phase_angle +   0).get_cos() * 0.5 + 0.5;
  motor0.target_PWN[1] = (m.svm_phase_angle + 120).get_cos() * 0.5 + 0.5;
  motor0.target_PWN[2] = (m.svm_phase_angle + 240).get_cos() * 0.5 + 0.5;
  
  m.t = millis();
  driver.update_PWM(&motor0, 0.1);

  if (record && (loop_count % 1 == 0))
  {
    measurments[buffer_size++] = m;
  }

  if (buffer_size == buffer_size_max)
  {
    
    Serial.println("decouple");
    delay(10);
    driver.decouple(&motor0);
    for (size_t i = 0; i < buffer_size_max; i++)
    {
      m = measurments[i];
     sprintf(txt, "%8lu, %3d, %3d, %3d",
            m.t, m.encoder_motor_angle(), m.encoder_phase_angle(), m.svm_phase_angle());
    
      Serial.println(txt);
      delay(1);
    }
    buffer_size = 0;
  }

  delay(10); // wait for a second
  loop_count++;
}