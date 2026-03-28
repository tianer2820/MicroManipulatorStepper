#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "config.h"

#include "rotation.h"
#include "temperature.h"
#include "i2c_parser.h"



// Vacuum runtime settings (tuneable by serial commands)
uint16_t vac_pull_duty = (uint16_t)(VAC_PWM_MAX * 1.00f); // 100%
uint16_t vac_hold_duty = (uint16_t)(VAC_PWM_MAX * 0.90f); // 90% default
uint32_t vac_pull_ms   = 800;                             // pulldown duration then switch to hold

enum VacMode : uint8_t { VAC_OFF=0, VAC_PULLDOWN=1, VAC_HOLD=2 };
volatile VacMode vac_mode = VAC_OFF;
uint32_t vac_mode_start_ms = 0;




// -------------------- ADDED: Vacuum functions --------------------
void vacuum_apply_pwm(uint16_t duty)
{
    duty = constrain(duty, 0, VAC_PWM_MAX - 1);
    analogWrite(VAC_PWM_PIN, duty);
}

void vacuum_off()
{
    vac_mode = VAC_OFF;
    vacuum_apply_pwm(0);
}

void vacuum_start_pulldown()
{
    vac_mode = VAC_PULLDOWN;
    vac_mode_start_ms = millis();
    vacuum_apply_pwm(vac_pull_duty);
}

void vacuum_hold()
{
    vac_mode = VAC_HOLD;
    vacuum_apply_pwm(vac_hold_duty);
}

void vacuum_update()
{
    if (vac_mode == VAC_PULLDOWN)
    {
        if ((millis() - vac_mode_start_ms) >= vac_pull_ms)
        {
            vacuum_hold();
        }
    }
}




void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    analogWriteFrequency(PWM_FREQ);
    analogWriteResolution(12);

    i2c_parser_init(
        (i2c_parser_callbacks){
            .home_rotation = rotation_home,
            .set_rotation_target = rotation_set,
            .get_rotation_target = rotation_get,
            .get_rotation = rotation_get,

            .set_temperature_setpoint = temperature_set,
            .get_temperature_setpoint = temperature_get,
            .get_temperature = temperature_get,

            .set_vacuum = nullptr,
            .get_vacuum = nullptr,
    });

    rotation_init();
    temperature_init();

    rotation_home();

    // buildAngleLookupTable();
    // printAngleLookupTable();
}


int i = 0;
uint32_t last_time = 0;
char input_buffer[16] = {0};
uint16_t input_index = 0;



void loop()
{
    // calculate dt

    uint32_t current_time = micros();
    uint32_t dt = 0;
    if (current_time < last_time) {
        // handle micros() overflow
        dt = (uint32_t)(current_time + (1<<32 - last_time));
    } else {
        dt = current_time - last_time;
    }
    last_time = current_time;

    // poll sensor and get angle
    rotation_poll(dt);


    if (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == '\n')
        {
            input_buffer[input_index] = '\0';

            // Minimal parsing
            if (input_index >= 2 && input_buffer[0] == 'V')
            {
                if (input_buffer[1] == '1')
                {
                    vacuum_start_pulldown();
                }
                else if (input_buffer[1] == '0')
                {
                    vacuum_off();
                }
                else if (input_buffer[1] == 'H')
                {
                    int pct = 0;
                    for (int n = 2; n < (int)input_index; n++) pct = pct * 10 + (input_buffer[n] - '0');
                    pct = constrain(pct, 0, 100);
                    vac_hold_duty = (uint16_t)(VAC_PWM_MAX * (pct / 100.0f));
                    if (vac_mode == VAC_HOLD) vacuum_apply_pwm(vac_hold_duty);
                }
                else if (input_buffer[1] == 'P')
                {
                    int pct = 0;
                    for (int n = 2; n < (int)input_index; n++) pct = pct * 10 + (input_buffer[n] - '0');
                    pct = constrain(pct, 0, 100);
                    vac_pull_duty = (uint16_t)(VAC_PWM_MAX * (pct / 100.0f));
                    if (vac_mode == VAC_PULLDOWN) vacuum_apply_pwm(vac_pull_duty);
                }
                else if (input_buffer[1] == 'T')
                {
                    int ms = 0;
                    for (int n = 2; n < (int)input_index; n++) ms = ms * 10 + (input_buffer[n] - '0');
                    vac_pull_ms = constrain(ms, 0, 10000);
                }
            } else {
                // input in degree
                // manual parse to avoid large size
                int temp = 0;
                for (int n = 0; n < input_index; n++)
                {
                    temp = temp * 10 + (input_buffer[n] - '0');
                }
                
                float input_angle = temp / 100.0;
    
                float rad_angle = input_angle / 180.0 * PI;
                while (rad_angle < 0)
                {
                    rad_angle += 2 * PI;
                }


                rotation_set(rad_angle);
            }
            input_index = 0;


        }
        else if (input_index < 15)
        {
            input_buffer[input_index] = c;
            input_index++;
        }
    }



    i += 1;
    if (i >= 50)
    {
        i = 0;
        // Serial.println(dt);
        // Serial.println(torque);
        // Serial.println((float)diff / MT6835_CPR * 360, 5);

        temperature_poll();
    }
}
