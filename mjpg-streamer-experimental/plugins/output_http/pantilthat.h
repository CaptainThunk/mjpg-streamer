/*  
    C++ interface for controlling a Pimoroni Pan-Tilt HAT device on a Raspberry Pi 3 B+.
    Class built upon SMBus and i2C standard APIs.

    Works with reference PIM183 (https://shop.pimoroni.com/products/pan-tilt-hat?variant=22408353287) ONLY.
    Available in France at https://www.kubii.fr/cartes-extension-cameras-raspberry-pi/1873-pan-tilt-hat-kit-kubii-3272496007277.html

    Stand-alone compilation
    -----------------------
    libi2c.so.0.1.0/0.1.1 must exist on the system.
    See build_cp.sh for building details.

    Copyright (C) 2019 Regis Corbel (regis dot corbel at wanadoo dot fr)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

/* Converted from C++ namespaced class to C functions by MCRWarrington 7/2021 for use with Mjpg Streamer */

#ifndef PANTILTHAT_H
#define PANTILTHAT_H

#include <stdint.h>  
#include <stdbool.h> 
#include <linux/types.h>
    
#define PWM	0
#define WS2812	1
#define RGB	0
#define GRB	1
#define RGBW	2
#define GRBW	3
#define MISSING_FUNC_1  "Error: Adapter does not have "
#define MISSING_FUNC_2  " capability.\n"

// Register semantics

// Configuration : 0x00
// Bit 7 - N/A
// Bit 6 - N/A
// Bit 5 - N/A
// Bit 4 - Light On
// Bit 3 - Light Mode: 0 = PWM, 1 = WS2812
// Bit 2 - Enable Lights
// Bit 1 - Enable Servo 2
// Bit 0 - Enable Servo 1

// Servo 1 (pan)
// Bytes 0x01 and 0x02
// Unsigned 16-bit little-endian value specifying
// how long (in microseconds) the servo 1 must run.

// Servo 2 (tilt)
// Bytes 0x03 and 0x04
// Unsigned 16-bit little-endian value specifying
// how long (in microseconds) the servo 2 must run.

#define REG_CONFIG	0x00
#define REG_SERVO1	0x01
#define REG_SERVO2	0x03
#define REG_WS2812	0x05
#define REG_UPDATE	0x4E
#define NUM_LEDS	24

    // Communicates with Pan-Tilt HAT over i2c
    // to control pan, tilt and light functions

	struct servominmax {
	    unsigned int valmin;
	    unsigned int valmax;
	};

	//public:
	void pantilthat_init();
	bool pantilthat_setup();
	void pantilthat_set_idle_timeout(char value);
	bool pantilthat_i2c_write_block(int daddr, unsigned char * data, int len);
	bool pantilthat_i2c_write_word(int reg, uint16_t w);
	bool pantilthat_i2c_write_byte(int reg, unsigned char c);
	bool pantilthat_i2c_read_byte(int reg, unsigned char * c);
	bool pantilthat_i2c_read_word(int reg, uint16_t * w);
	void pantilthat_clear();
	void pantilthat_light_mode(unsigned int mode);
	void pantilthat_light_type(unsigned int set_type);
	int  pantilthat_num_pixels();
	void pantilthat_set_brightness(char brightn);
	// void pantilthat_set_all(char red, char green, char blue, int white = -1);
	void pantilthat_set_all(char red, char green, char blue, int white);
	void pantilthat_set_pixel_rgbw(int index, char red, char green, char blue, int white);
	// bool pantilthat_set_pixel(int index, char red, char green, char blue, int white = -1);
	bool pantilthat_set_pixel(int index, char red, char green, char blue, int white);
	void pantilthat_show();
	bool pantilthat_servo_enable(char index, unsigned char state);
	bool pantilthat_is_servo_enabled(char index, unsigned char * state);
	bool pantilthat_servo_pulse_min(char index, unsigned int value);
	bool pantilthat_servo_pulse_max(char index, unsigned int value);
	int  pantilthat_get_servo(char index);
	void pantilthat_set_servo(char index, int angle);
	// Commented out because it prevents the servos from running.
	// May be useful someday.
	//bool servo_stop(char index);
	int  pantilthat_lsleep(long int ms);

	// Private:
	void pantilthat_set_config();
	bool pantilthat_check_int_range(int value, int value_min, int value_max);
	int pantilthat_servo_us_to_degrees(int us, int us_min, int us_max);
	int pantilthat_servo_degrees_to_us(int angle, int us_min, int us_max);
	bool pantilthat_servo_range(int servo_index, int * min, int * max);

#endif

