/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsMatrix.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix.h"

////////////

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif
/////////////////////

extern const AP_HAL::HAL& hal;

// Init
void AP_MotorsMatrix::Init()
{
    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate( uint16_t speed_hz )
{
    uint8_t i;

    // record requested speed
    _speed_hz = speed_hz;

    // check each enabled motor
    uint32_t mask = 0;
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
		mask |= 1U << i;
        }
    }
    rc_set_freq( mask, _speed_hz );
}

// set frame orientation (normally + or X)
void AP_MotorsMatrix::set_frame_orientation( uint8_t new_orientation )
{
    // return if nothing has changed
    if( new_orientation == _flags.frame_orientation ) {
        return;
    }

    // call parent
    AP_Motors::set_frame_orientation( new_orientation );

    // setup the motors
    setup_motors();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsMatrix::enable()
{
    int8_t i;

    // enable output channels
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_enable_ch(i);
        }
    }

    //NEW CODE
    rc_enable_ch(8);
    rc_enable_ch(9);
    rc_enable_ch(10);
    rc_enable_ch(11);
    rc_enable_ch(12);
    //NEW CODE
}

void AP_MotorsMatrix::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            // set motor output based on thrust requests
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = get_pwm_output_min();
                }
            }
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = calc_spin_up_to_pwm();
                }
            }
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
                }
            }
            break;
    }

    // send output to each motor
    hal.rcout->cork();
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }

    //NEW CODE
    //testing outputting on AUX channels
    //This allows up to pass through the throttle to the other channels
    float throttle_thrust = get_throttle();
    rc_write(8, calc_thrust_to_pwm(throttle_thrust));
    rc_write(9, calc_thrust_to_pwm(throttle_thrust));
    rc_write(10, calc_thrust_to_pwm(throttle_thrust));
    rc_write(11, calc_thrust_to_pwm(throttle_thrust));
    //NEW CODE

    hal.rcout->push();
}


// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsMatrix::get_motor_mask()
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            mask |= 1U << i;
        }
    }
    return rc_map_mask(mask);
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed_stabilizing()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   servo_rotation_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   rpy_low = 0.0f;             // lowest motor value
    float   rpy_high = 0.0f;            // highest motor value
    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
    float   unused_range;               // amount of yaw we can fit in the current channel
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
    yaw_thrust = _yaw_in * get_compensation_gain();

    // calculate roll and pitch for each motor
    // calculate the amount of yaw input that each motor can accept
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            //NEW CODE
            //servo effects roll
            if(fabsf(_roll_factor[i]) != 0) {
                //perform clac as the appropriate servo is selected
                if(((roll_thrust > 0) && (_roll_factor[i] < 0)) or ((roll_thrust < 0) && (_roll_factor[i] > 0))){
                    _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i];
                } else {
                    _thrust_rpyt_out[i] = 0.0f;
                }
                if(_spool_mode == THROTTLE_UNLIMITED) {
                    hal.console->printf("roll:%.6f  ", (float)_thrust_rpyt_out[i]);
                }
            }

            //do pitch calc
            if(fabsf(_pitch_factor[i]) != 0) {
                //perform clac as the appropriate servo is selected
                if(((pitch_thrust > 0) && (_pitch_factor[i] < 0)) or ((pitch_thrust < 0) && (_pitch_factor[i] > 0))){
                    _thrust_rpyt_out[i] = pitch_thrust * _pitch_factor[i];
                } else {
                    _thrust_rpyt_out[i] = 0.0f;
                }
                if(_spool_mode == THROTTLE_UNLIMITED) {
                    hal.console->printf("pitch:%.6f  ", (float)_thrust_rpyt_out[i]);
                }
            }

            //determines the minimum available range on all yaw servos that are appropriate
            if (yaw_thrust * _yaw_factor[i] > 0.0f) {
                unused_range = fabsf((1.0f - _thrust_rpyt_out[i])/_yaw_factor[i]);
                if (yaw_allowed > unused_range) {
                    yaw_allowed = unused_range;
                }
            } else {
                unused_range = fabsf(_thrust_rpyt_out[i]/_yaw_factor[i]);
                if (yaw_allowed > unused_range) {
                    yaw_allowed = unused_range;
                }
            }
        }
    }

    //constrain how much yaw we want to how much we can provide
    if (fabsf(yaw_thrust) > yaw_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw to intermediate numbers for each motor
    rpy_low = 0.0f;
    rpy_high = 0.0f;
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        //NEW CODE
        if((yaw_thrust > 0) && (_yaw_factor[i] > 0)) {
            //_thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];
        } else if ((yaw_thrust < 0) && (_yaw_factor[i] < 0)){
            //_thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];
        }
        //NEW CODE

        // record lowest roll+pitch+yaw command
        if (_thrust_rpyt_out[i] < rpy_low) {
            rpy_low = _thrust_rpyt_out[i];
        }
        // record highest roll+pitch+yaw command
        if (_thrust_rpyt_out[i] > rpy_high) {
            rpy_high = _thrust_rpyt_out[i];
        }
    }

    // check everything fits
    servo_rotation_best_rpy = ((rpy_low+rpy_high)/2.0);
    if (fabsf(rpy_high) <= 1){
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-servo_rotation_best_rpy/rpy_low, 0.0f, 1.0f);
    }

    ///////////////////////////////////////////
    //_thrust_rpyt_out[i]  ----> this has the motor thrust scale based on hover requirements -1 - +1

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

           ////////////////////////////////////////////////
          //This code causes motors to remain at defined position to test take off weights
          ////////////////////////////////////////////////
          /*if(i == 4) {
              _thrust_rpyt_out[i] = calc_pwm_to_angle((float)_SP_1_P);
              if(_spool_mode == THROTTLE_UNLIMITED) {
                // set motor output based on thrust requests
                //Gain DATA on the mixing numbers
                hal.console->printf("Motor: %1.f : %.4f : %.6f  ", (float)i, (float)_SP_1_P, (float)_thrust_rpyt_out[i]);

              }
          } else if(i == 5) {
              _thrust_rpyt_out[i] = calc_pwm_to_angle((float)_SP_2_P);
              if(_spool_mode == THROTTLE_UNLIMITED) {
                // set motor output based on thrust requests
                //Gain DATA on the mixing numbers
                hal.console->printf("Motor: %1.f : %.4f : %.6f  ", (float)i, (float)_SP_2_P, (float)_thrust_rpyt_out[i]);

              }
          } else if(i == 6) {
              _thrust_rpyt_out[i] = calc_pwm_to_angle((float)_SP_3_P);
              if(_spool_mode == THROTTLE_UNLIMITED) {
                // set motor output based on thrust requests
                //Gain DATA on the mixing numbers
                hal.console->printf("Motor: %1.f : %.4f : %.6f  ", (float)i, (float)_SP_3_P, (float)_thrust_rpyt_out[i]);

              }
          } else if (i == 7) {
              _thrust_rpyt_out[i] = calc_pwm_to_angle((float)_SP_4_P);
              if(_spool_mode == THROTTLE_UNLIMITED) {
                // set motor output based on thrust requests
                //Gain DATA on the mixing numbers
                hal.console->printf("Motor: %1.f : %.4f : %.6f  ", (float)i, (float)_SP_4_P, (float)_thrust_rpyt_out[i]);

              }
          }*/


           //ORIGINAL CODE
          // _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + rpy_scale*_thrust_rpyt_out[i];

            //NEW CODE
            //_thrust_rpyt_out[i] = servo_rotation_best_rpy + rpy_scale*_thrust_rpyt_out[i];

        }
    }

    if(_spool_mode == THROTTLE_UNLIMITED) {
       hal.console->printf("\n");
    }

    //NEW CODE
    // constrain all outputs to 0.0f to 1.0f
    // needs to convert
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], 0.0f, 1.0f);

        }
    }

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], 0.0f, 1.0f);

        }
    }
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMatrix::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // printf test
    hal.console->printf("pwm:");
    hal.console->printf("%.6f motor# %.6f \n", (float)pwm, (float)AP_MOTORS_MAX_NUM_MOTORS );
   // hal.console->printf("\n");

    // loop through all the possible orders spinning any motors that match that description
    hal.rcout->cork();
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {
            // turn on this motor
            rc_write(i, pwm);
        }
    }
    hal.rcout->push();
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // increment number of motors if this motor is being newly motor_enabled
        if( !motor_enabled[motor_num] ) {
            motor_enabled[motor_num] = true;
        }

        // set roll, pitch, thottle factors and opposite motor (for stability patch)
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // call parent class method
        add_motor_num(motor_num);
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0;
        _pitch_factor[motor_num] = 0;
        _yaw_factor[motor_num] = 0;
    }
}

// remove_all_motors - removes all motor definitions
void AP_MotorsMatrix::remove_all_motors()
{
    for( int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        remove_motor(i);
    }
}

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
void AP_MotorsMatrix::normalise_rpy_factors()
{
    float roll_fac = 0.0f;
    float pitch_fac = 0.0f;
    float yaw_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (roll_fac < fabsf(_roll_factor[i])) {
                roll_fac = fabsf(_roll_factor[i]);
            }
            if (pitch_fac < fabsf(_pitch_factor[i])) {
                pitch_fac = fabsf(_pitch_factor[i]);
            }
            if (yaw_fac < fabsf(_yaw_factor[i])) {
                yaw_fac = fabsf(_yaw_factor[i]);
            }
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (!is_zero(roll_fac)) {
                _roll_factor[i] = 0.5f*_roll_factor[i]/roll_fac;
            }
            if (!is_zero(pitch_fac)) {
                _pitch_factor[i] = 0.5f*_pitch_factor[i]/pitch_fac;
            }
            if (!is_zero(yaw_fac)) {
                _yaw_factor[i] = 0.5f*_yaw_factor[i]/yaw_fac;
            }
        }
    }
}


/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsMatrix::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
    }
}
