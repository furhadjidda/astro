
/*
 *   This file is part of astro_core_ros.
 *
 *   astro_core_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   astro_core_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License v3.0
 *   along with astro_core_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "motor_control.hpp"
#include "pca9685.hpp"

motor_control motor;

void setup() {

    motor.Init();
    stdio_init_all();
}

void loop() {
    int encoder_tick = 0;
    // put your main code here, to run repeatedly:
    printf("Motor Start");
    // motor.MotorRun(MotorId::MotorB, MotorDirection::Forward, 100);
    //  motor.MotorStop();
    //  delay(5000);
    //  motor.Forward(100,(2 * M_PI * wheelRadius));
    motor.AngularVelocity(MotorId::MotorA);
    sleep_ms(2000);
    motor.AngularVelocity(MotorId::MotorB);

#if 0
  motor.Right(100);
  delay(2000);
  motor.Left(100);
  delay(2000);
#endif
    printf("Motor Stop");
    motor.MotorStop();
    sleep_ms(5000);
}

int main() {
    setup();
    while (1) {
        loop();
        sleep_ms(500);
    }
}