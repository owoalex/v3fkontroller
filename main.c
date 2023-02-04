/**
 * Copyright (c) 2022 Alex Baldwin.
 * 
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include "hardware/adc.h"

const uint64_t rotor_loops_per_full_rotation = 15;

const uint LED_A_PIN = 24;
const uint LED_B_PIN = 25;

const uint FAULT_PIN = 15;

const uint MOTOR_HI_U = 0;
const uint MOTOR_HI_V = 1;
const uint MOTOR_HI_W = 2;
const uint MOTOR_LO_U = 3;
const uint MOTOR_LO_V = 4;
const uint MOTOR_LO_W = 5;

const uint MOTOR_SENSE_U = 26;
const uint MOTOR_SENSE_V = 27;
const uint MOTOR_SENSE_W = 28;

void motor_output_step_0() {
    gpio_put(MOTOR_HI_U, 1); gpio_put(MOTOR_LO_U, 1);
    gpio_put(MOTOR_HI_V, 1); gpio_put(MOTOR_LO_V, 0);
    gpio_put(MOTOR_HI_W, 0); gpio_put(MOTOR_LO_W, 1);
}

void motor_output_step_1() {
    gpio_put(MOTOR_HI_U, 1); gpio_put(MOTOR_LO_U, 0);
    gpio_put(MOTOR_HI_V, 1); gpio_put(MOTOR_LO_V, 1);
    gpio_put(MOTOR_HI_W, 0); gpio_put(MOTOR_LO_W, 1);
}

void motor_output_step_2() {
    gpio_put(MOTOR_HI_U, 1); gpio_put(MOTOR_LO_U, 0);
    gpio_put(MOTOR_HI_V, 0); gpio_put(MOTOR_LO_V, 1);
    gpio_put(MOTOR_HI_W, 1); gpio_put(MOTOR_LO_W, 1);
}

void motor_output_step_3() {
    gpio_put(MOTOR_HI_U, 1); gpio_put(MOTOR_LO_U, 1);
    gpio_put(MOTOR_HI_V, 0); gpio_put(MOTOR_LO_V, 1);
    gpio_put(MOTOR_HI_W, 1); gpio_put(MOTOR_LO_W, 0);
}

void motor_output_step_4() {
    gpio_put(MOTOR_HI_U, 0); gpio_put(MOTOR_LO_U, 1);
    gpio_put(MOTOR_HI_V, 1); gpio_put(MOTOR_LO_V, 1);
    gpio_put(MOTOR_HI_W, 1); gpio_put(MOTOR_LO_W, 0);
}

void motor_output_step_5() {
    gpio_put(MOTOR_HI_U, 0); gpio_put(MOTOR_LO_U, 1);
    gpio_put(MOTOR_HI_V, 1); gpio_put(MOTOR_LO_V, 0);
    gpio_put(MOTOR_HI_W, 1); gpio_put(MOTOR_LO_W, 1);
}

void setup_gpio() {
    adc_init();
    adc_gpio_init(29);
    
    gpio_init(MOTOR_SENSE_U);
    gpio_set_dir(MOTOR_SENSE_U, GPIO_IN);
    gpio_set_pulls(MOTOR_SENSE_U, true, false);
    gpio_init(MOTOR_SENSE_V);
    gpio_set_dir(MOTOR_SENSE_V, GPIO_IN);
    gpio_set_pulls(MOTOR_SENSE_V, true, false);
    gpio_init(MOTOR_SENSE_W);
    gpio_set_dir(MOTOR_SENSE_W, GPIO_IN);
    gpio_set_pulls(MOTOR_SENSE_W, true, false);
    
    gpio_init(MOTOR_HI_U);
    gpio_set_dir(MOTOR_HI_U, GPIO_OUT);
    gpio_init(MOTOR_HI_V);
    gpio_set_dir(MOTOR_HI_V, GPIO_OUT);
    gpio_init(MOTOR_HI_W);
    gpio_set_dir(MOTOR_HI_W, GPIO_OUT);
    gpio_init(MOTOR_LO_U);
    gpio_set_dir(MOTOR_LO_U, GPIO_OUT);
    gpio_init(MOTOR_LO_V);
    gpio_set_dir(MOTOR_LO_V, GPIO_OUT);
    gpio_init(MOTOR_LO_W);
    gpio_set_dir(MOTOR_LO_W, GPIO_OUT);
    
    gpio_init(FAULT_PIN);
    gpio_set_dir(FAULT_PIN, GPIO_IN);
    gpio_set_pulls(FAULT_PIN, true, false); // Set pullup, unset pulldown
    
    gpio_init(LED_A_PIN);
    gpio_set_dir(LED_A_PIN, GPIO_OUT);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
}

void motor_output_off() {
    gpio_put(MOTOR_HI_U, 1); gpio_put(MOTOR_LO_U, 1);
    gpio_put(MOTOR_HI_V, 1); gpio_put(MOTOR_LO_V, 1);
    gpio_put(MOTOR_HI_W, 1); gpio_put(MOTOR_LO_W, 1);
}

void idle_loop() {
    gpio_put(LED_A_PIN, 1);
    gpio_put(LED_B_PIN, 0);
    sleep_ms(250);
    gpio_put(LED_A_PIN, 0);
    gpio_put(LED_B_PIN, 1);
    sleep_ms(250);
}

void motor_output_on(uint8_t step) {
    switch (step) {
        case 0:
            motor_output_step_0();
            break;
        case 1:
            motor_output_step_1();
            break;
        case 2:
            motor_output_step_2();
            break;
        case 3:
            motor_output_step_3();
            break;
        case 4:
            motor_output_step_4();
            break;
        case 5:
            motor_output_step_5();
            break;
        default:
            motor_output_off();
    }
}

double motor_duty_cycle = 0.0;

uint8_t rotor_last_step = 0;
uint64_t last_step_time = 0;
uint8_t steps_jumped = 1;
uint64_t time_since_last_step = 0;
uint8_t rotor_current_step = 0;

double rotor_phase_frequency = 0; // In Hz
double rotor_phase_position = 0; // In rad
double rotor_average_speed = 0; // In rads-1

void check_rotor_position() {
    bool parsed_u = gpio_get(MOTOR_SENSE_U);
    bool parsed_v = gpio_get(MOTOR_SENSE_V);
    bool parsed_w = gpio_get(MOTOR_SENSE_W);
    
    int8_t rotor_proposed_step = -1;
    
    if ((!parsed_u) && (parsed_v) && (parsed_w)) {
        rotor_proposed_step = 0;
    }
    if ((!parsed_u) && (parsed_v) && (!parsed_w)) {
        rotor_proposed_step = 1;
    }
    if ((parsed_u) && (parsed_v) && (!parsed_w)) {
        rotor_proposed_step = 2;
    }
    if ((parsed_u) && (!parsed_v) && (!parsed_w)) {
        rotor_proposed_step = 3;
    }
    if ((parsed_u) && (!parsed_v) && (parsed_w)) {
        rotor_proposed_step = 4;
    }
    if ((!parsed_u) && (!parsed_v) && (parsed_w)) {
        rotor_proposed_step = 5;
    }
    
    
    
    if (rotor_proposed_step != -1) {
        if (rotor_current_step != rotor_proposed_step) {
            if (rotor_proposed_step != rotor_last_step) { // Check we're not just oscillating
                steps_jumped = ((rotor_proposed_step - rotor_current_step) + 6) % 6;
                if (steps_jumped >= 3) {
                    steps_jumped = 6 - steps_jumped;
                }
                if (steps_jumped < 0) {
                    steps_jumped = 0;
                }
                rotor_last_step = rotor_current_step;
                rotor_current_step = rotor_proposed_step;
                last_step_time = time_since_last_step;
                
                if (time_since_last_step > 0) {
                    rotor_phase_frequency = ((1000000.0 / time_since_last_step) / 6.0) * steps_jumped;
                }
                
                time_since_last_step = 0;
                
                
            }
        }
        if (time_since_last_step > 1000000.0) {
            rotor_phase_frequency = 0;
        }
        rotor_average_speed = (rotor_phase_frequency / rotor_loops_per_full_rotation) * (M_PI * 2.0);
    }
}

void estimate_rotor_position() {
    double base_position = ((rotor_current_step / 6.0) * (M_PI * 2.0));
    double calc_offset = 0;
    double base_offset = -(M_PI / 6.0);
    if (rotor_phase_frequency > 1) {
        if (last_step_time > 0) {
            double ratio = (double) time_since_last_step / ((double) last_step_time / steps_jumped);
            calc_offset = ratio * (M_PI / 3.0);
        }
    }
    rotor_phase_position = base_position + calc_offset + base_offset;
    if (rotor_phase_position < 0.0) {
        rotor_phase_position = rotor_phase_position + (M_PI * 2.0);
    }
}


// CONTROL METHODDS GO HERE

double pwm_control_frequency = 175;

void pwm_control_loop() {
    check_rotor_position();
    //estimate_rotor_position();
    
    uint8_t rotor_pulse_step = 0;
    
    double angle_offset = ((M_PI * 2.0) / 6.0) * 1.0;
    rotor_pulse_step = round(((rotor_phase_position + angle_offset) / (M_PI * 2.0)) * 6.0);
    
    rotor_pulse_step = rotor_current_step;
    rotor_pulse_step += 2;
    rotor_pulse_step = rotor_pulse_step % 6;
    
    uint64_t step_time = (uint64_t) ((1.0 / pwm_control_frequency) * 1000000);
    uint64_t on_time = step_time * motor_duty_cycle;
    uint64_t sleep_time = step_time - on_time;
    
    if (on_time > 0) {
        motor_output_on(rotor_pulse_step);
        sleep_us(on_time);
    }
    motor_output_off();
    sleep_us(sleep_time);
    
    time_since_last_step += step_time;
}

bool r2sw = false;

uint64_t steps_since_last_report = 0;

int main() {
    setup_gpio();
    stdio_init_all();
    while (true) {
        adc_select_input(3);
        uint16_t throttle = adc_read();
        
        motor_duty_cycle = ((throttle - 1380) / 2808.0);
        if (motor_duty_cycle > 0.9) {
            motor_duty_cycle = 0.9;
        }
        
        if (motor_duty_cycle < 0.0) {
            motor_duty_cycle = 0.0;
        }
        
        steps_since_last_report++;
        if (steps_since_last_report > 100) {
            //check_rotor_position();
            //estimate_rotor_position();
            printf("%3.2f Hz; %3.2f rads-1; %d Hz PWM; S %d; THR %1.2f\n", rotor_phase_frequency, rotor_average_speed, (int) pwm_control_frequency, rotor_current_step, motor_duty_cycle);
            steps_since_last_report = 0;
        }
        
        pwm_control_frequency = 175;
        double rotor_output_multiplier = 15;
        if (rotor_phase_frequency > (0.6 * rotor_output_multiplier)) {
            pwm_control_frequency = 197;
        }
        if (rotor_phase_frequency > (1.2 * rotor_output_multiplier)) {
            pwm_control_frequency = 223;
        }
        if (rotor_phase_frequency > (1.8 * rotor_output_multiplier)) {
            pwm_control_frequency = 234;
        }
        if (rotor_phase_frequency > (2.4 * rotor_output_multiplier)) {
            pwm_control_frequency = 262;
        }
        if (rotor_phase_frequency > (3.0 * rotor_output_multiplier)) {
            pwm_control_frequency = 294;
        }
        if (rotor_phase_frequency > (3.6 * rotor_output_multiplier)) {
            pwm_control_frequency = 312;
        }
        if (rotor_phase_frequency > (4.2 * rotor_output_multiplier)) {
            pwm_control_frequency = 350;
        }
        if (rotor_phase_frequency > (4.8 * rotor_output_multiplier)) {
            pwm_control_frequency = 400;
        }
        /*
        175Hz       0.0Hz
        197Hz       0.6Hz
        223Hz       1.2Hz
        234Hz       1.8Hz
        262Hz       2.6Hz
        294Hz       3.2Hz
        312Hz       3.8Hz
        350Hz       4.4Hz
        400Hz       5.0Hz
        */
        
        //idle_loop();
        pwm_control_loop();
    }
}
