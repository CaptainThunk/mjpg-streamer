/*******************************************************************************
#                                                                              #
#      MJPG-streamer allows to stream JPG frames from an input-plugin          #
#      to several output plugins                                               #
#                                                                              #
#      Copyright (C) 2007 Tom Stöveken                                         #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; version 2 of the License.                      #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#ifndef MJPG_STREAMER_H
#define MJPG_STREAMER_H
#define SOURCE_VERSION "2.0"

/* FIXME take a look to the output_http clients thread marked with fixme if you want to set more then 10 plugins */
#define MAX_INPUT_PLUGINS 10
#define MAX_OUTPUT_PLUGINS 10
#define MAX_PLUGIN_ARGUMENTS 32

#include <linux/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>
#include <pthread.h>
#include <stdbool.h>

#ifdef DEBUG
#define DBG(...) fprintf(stderr, " DBG(%s, %s(), %d): ", __FILE__, __FUNCTION__, __LINE__); fprintf(stderr, __VA_ARGS__)
#else
#define DBG(...)
#endif

#define LOG(...) { char _bf[1024] = {0}; snprintf(_bf, sizeof(_bf)-1, __VA_ARGS__); fprintf(stderr, "%s", _bf); syslog(LOG_INFO, "%s", _bf); }

#include "plugins/input.h"
#include "plugins/output.h"

/* global variables that are accessed by all plugins */
typedef struct _globals globals;

/* an enum to identify the commands destination*/
typedef enum {
    Dest_Input = 0,
    Dest_Output = 1,
    Dest_Program = 2,
} command_dest;

/* commands which can be send to the input plugin */
//typedef enum _cmd_group cmd_group;
enum _cmd_group {
    IN_CMD_GENERIC =        0, // if you use non V4L2 input plugin you not need to deal the groups.
    IN_CMD_V4L2 =           1,
    IN_CMD_RESOLUTION =     2,
    IN_CMD_JPEG_QUALITY =   3,
    IN_CMD_PWC =            4,
};

typedef struct _control control;
struct _control {
    struct v4l2_queryctrl ctrl;
    int value;
    struct v4l2_querymenu *menuitems;
    /*  In the case the control a V4L2 ctrl this variable will specify
        that the control is a V4L2_CTRL_CLASS_USER control or not.
        For non V4L2 control it is not acceptable, leave it 0.
    */
    int class_id;
    int group;
};

struct _globals {
    int stop;

    /* input plugin */
    input in[MAX_INPUT_PLUGINS];
    int incnt;

    /* output plugin */
    output out[MAX_OUTPUT_PLUGINS];
    int outcnt;

    /* pointer to control functions */
    //int (*control)(int command, char *details);
    int isRecording;
    int debugCamMode;
    int debugContourMode;
    int motionDetectMode;
    int motionThresholdSlider;
    int contourThresholdSlider;
    
    //int pth_pan;
    //int pth_tilt;
    // int cap_fps;
    // int cap_brightness;
    // int cap_contrast;
    // int cap_saturation;
    // int cap_gain;
    // int cap_exposure;
};
typedef struct imu01cClass imu01cClass;

struct imu01cClass{
    int i2c_lsm303d_addr, i2c_l3gd20h_addr, _length;
    bool is_setup;
    int _i2c_retries;
    unsigned int _i2c_retry_time;
    double accel_xyz[3];
    int mag_xyz[3];
    int gyro_xyz[3];
    double heading;
    double headingDegrees;
    double tiltHeading;
    double tiltHeadingDegrees;
    int LSM303D_Temperature;
    double LSM303D_TemperatureDegrees;
    int L3GD20H_Temperature;
    double L3GD20H_TemperatureDegrees;
    
    bool (*setup)(imu01cClass *self);
    bool (*i2c_write_byte)(imu01cClass *self, int addr, int reg, unsigned char c);
    bool (*i2c_read_byte)(imu01cClass *self, int addr, int reg, unsigned char * c);
    bool (*check_int_range)(int value, int value_min, int value_max);
    int (*lsleep)(long int ms);
    int (*twos_comp)(int val, int bits);
    void (*getLSM303DTemperature)(imu01cClass *self);
    void (*getL3GD20HTemperature)(imu01cClass *self);
    void (*getHeading)(imu01cClass *self);
    void (*getTiltHeading)(imu01cClass *self);
    void (*update)(imu01cClass *self);
    void (*getMag)(imu01cClass *self);
    void (*getAccel)(imu01cClass *self);
    void (*getGyro)(imu01cClass *self);
    bool (*isMagReady)(imu01cClass *self);
};

struct imu01cClass imu01c;
#endif
