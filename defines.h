// Prevents multiple inclusion

#ifndef DEFINES_H
#define DEFINES_H

#include "defines_lcd.h"

// Reference voltage in mV
#define VREF 3240

// ... Buttons configuration ...
#define KEYPORT PINA >> 4

// Definitions of texts for LCD
#define txt_hello_line0 "MOTORBIKE"
#define txt_hello_line1 "CRASH DETECTOR"
#define txt_run_detector "Run Detector"
#define txt_calibration "Calibration"
#define txt_gps_data "GPS Data"
#define txt_gsm_module "GSM Module"
#define txt_set_the_origin "Set the origin"
#define txt_run_calibration "Run calibration"
#define txt_calibration_in_progress "in progress..."
#define txt_alarm_triggers "Alarm triggers"
#define txt_alarm_delay "Alarm delay"
#define txt_gps_data_time "UTC Time"
#define txt_gps_data_position "Position"
#define txt_gps_data_invalid "GPS disconnected"
#define txt_gsm_alarms_receiver "Alarms receiver"
#define txt_gsm_send_test_sms "Send test SMS"
#define txt_check_your_inbox "Check your inbox"

#endif // DEFINES_H