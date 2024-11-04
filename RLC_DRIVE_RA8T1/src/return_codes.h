/*
 * return_codes.h
 *
 *  Created on: 11 oct. 2023
 *      Author: Ch.Leclercq
 */

#ifndef RETURN_CODES_H_
#define RETURN_CODES_H_

#define F_RET_OK                                  0
#define F_RET_ERROR_GENERIC                  -10000

#define F_RET_MOTOR_STOP_FLAG_TIMEOUT        -10100
#define F_RET_MOTOR_BAD_TYPE                 -10101

#define F_RET_MOTOR_SEQUENCE_ERROR_START     -10200
#define F_RET_MOTOR_SEQUENCE_ERROR_RUN       -10201
#define F_RET_MOTOR_SEQUENCE_ERROR_TIMEOUT   -10202

#define F_RET_MOTOR_CHECK_ERROR              -10300

#define F_RET_MOTOR_INIT_DAMAGED_PANELS      -10301
#define F_RET_MOTOR_INIT_STRETCH             -10302
#define F_RET_MOTOR_INIT_TIMEOUT_BASE_L      -10303
#define F_RET_MOTOR_INIT_TIMEOUT_BASE_H      -10304
#define F_RET_MOTOR_INIT_UNEXPECTED_ERROR    -10305
#define F_RET_MOTOR_INIT_TIMEOUT_POSTER      -10306

#define F_RET_MOTOR_DRIVE_CANCELLED          -10400
#define F_RET_MOTOR_OVERCURRENT_VM           -10401
#define F_RET_MOTOR_ERROR_API_FSP            -10402
#define F_RET_MOTOR_ERROR_PULSESH            -10403
#define F_RET_MOTOR_ERROR_PULSESL            -10404

#define F_RET_MOTOR_AUTO_TIMEOUT_POSTER      -10500
#define F_RET_MOTOR_AUTO_OVERCURRENT         -10501
#define F_RET_MOTOR_AUTO_TIMEOUT_PULSES      -10502
#define F_RET_PANELS_DAMAGED                 -10503
#define F_RET_MOTOR_DRIVING_H                -10504
#define F_RET_MOTOR_DRIVING_L                -10505
#define F_RET_MOTOR_DRIVING_HL               -10506

#define F_RET_ERROR_VEE_OPENING              -11000
#define F_RET_ERROR_VEE_CLOSING              -11001
#define F_RET_ERROR_VEE_NOT_FOUND            -11002
#define F_RET_ERROR_VEE_WRITING_OPERATION    -11003
#define F_RET_ERROR_VEE_WRITING_TIMEOUT      -11004
#define F_RET_ERROR_VEE_READING              -11005
#define F_RET_ERROR_VEE_FORMATING            -11006
#define F_RET_ERROR_VEE_GETTING_STATUS       -11007
#define F_RET_ERROR_VEE_NOT_ERASED           -11008

#define F_RET_FLASH_OPEN                     -11050
#define F_RET_FLASH_CLOSE                    -11051
#define F_RET_FLASH_MOUNT                    -11052
#define F_RET_FLASH_FORMAT                   -11053
#define F_RET_FLASH_NOT_OPENED               -11054
#define F_RET_FLASH_MK_ROOT                  -11055
#define F_RET_FLASH_UNMOUNT                  -11056
#define F_RET_FLASH_FILE_OPEN                -11057
#define F_RET_FLASH_FILE_WRITE               -11058

#define F_RET_JSON_PARSE                     -11100
#define F_RET_JSON_FIND_OBJECT               -11101
#define F_RET_JSON_BAD_TYPE                  -11102
#define F_RET_JSON_RESPONSE_ERROR            -11103
#define F_RET_JSON_CREATE                    -11104
#define F_RET_JSON_NOT_FOUND                 -11105

#define F_RET_COMMS_OUT_TIMEOUT              -11200
#define F_RET_COMMS_OUT_BAD_RESPONSE         -11201
#define F_RET_COMMS_MQTT_LTE_NOT_CONNECTED   -11202
#define F_RET_COMMS_MQTT_BROK_NOT_CONNECTED  -11203
#define F_RET_COMMS_MQTT_TIMEOUT             -11204
#define F_RET_COMMS_MQTT_BUSY                -11205
#define F_RET_COMMS_MQTT_GENERIC             -11206
#define F_RET_COMMS_MQTT_NO_PANELNAME        -11207

#define F_RET_QUEUE_RECEIVE                  -11300
#define F_RET_RTC_NOT_CONFIGURED             -11301

#define F_RET_FS_INIT                        -11400
#define F_RET_FS_INIT_MEDIA                  -11401
#define F_RET_FS_FORMAT                      -11402
#define F_RET_FS_READ_DIRECTORY              -11403
#define F_RET_FS_CREATE_DIRECTORY            -11404
#define F_RET_FS_CREATE_FILE                 -11405
#define F_RET_FS_NO_MORE_ENTRIES             -11406
#define F_RET_FS_NOT_FOUND                   -11407
#define F_RET_FS_GENERIC                     -11408
#define F_RET_FS_SET_DIRECTORY               -11409
#define F_RET_FS_DELETE_DIRECTORY            -11410
#define F_RET_FS_DELETE_FILE                 -11411
#define F_RET_FS_CLOSE_FILE                  -11412
#define F_RET_FS_OPEN_FILE                   -11413
#define F_RET_FS_READ_FILE_EOF               -11414
#define F_RET_FS_READ_FILE                   -11415
#define F_RET_FS_WRITE_FILE                  -11416
#define F_RET_FS_RENAME_FILE                 -11417

#endif /* RETURN_CODES_H_ */
