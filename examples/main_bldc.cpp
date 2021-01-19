/*
  ENCODER
  @author Markus Bader
 */

#include <Arduino.h>

#include <car/bldc/driver.h>
#include <car/encoder/encoder_as5048a.h>

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
float angle = 0;
float angle_steps_size = M_PI_2 / 100.;

// the setup routine runs once when you press reset:
void setup()
{
  driver.init(&motor0, &motor1);

  Serial.begin(115200); /// init serial
  while (!Serial)
    ;
  Serial.println("BLDC \n");
  Serial.println(motor0.info);
  Serial.println(motor1.info);
}

// the loop routine runs over and over again forever:
void loop()
{
  angle = angle + angle_steps_size;
  while (angle > M_TWOPI)
    angle -= M_TWOPI;
  for (int i = 0; i < 3; i++)
    motor0.target_PWN[i] = cos(angle + M_TWOPI / 3. * (float)i) * 0.5 + 0.5;
  driver.update_PWM(&motor0, 0.1);
  if (loop_count % 100 == 0)
  {
    Serial.print(loop_count);
    Serial.print(" angle: ");
    Serial.print(angle);
    Serial.print(" rad =  ");
    Serial.print((int)180 / M_PI * angle);
    Serial.print(" deg. ");
    for (int i = 0; i < 3; i++)
    {
      Serial.print(", ");
      Serial.print(motor0.target_PWN[i]);
    }
    Serial.println();
  }
  delay(10); // wait for a second
  loop_count++;
}