#pragma once
#define CanRX GPIO_NUM_19
#define CanTX GPIO_NUM_13

#define REG_MOTOR_ASSIST_LEVEL                                 0x09 //[unit:%, range:-100..100, factor:1.5625] !!! signed !!!

#define REG_MOTOR_ASSIST_WALK_LEVEL                            0x0A //Top level when assisting in walk mode [unit:%, factor:1.5625]
#define REG_MOTOR_ASSIST_WALK_SPEED_DECREASE_START             0x0B //Speed from which the motor starts diminishing its assistance when using the "walk mode" [unit:km/h, factor:0.1]
#define REG_MOTOR_ASSIST_WALK_SPEED_DECREASE_END               0x0C //Speed at which the motor gives no more assistance when using the "walk mode" [unit:km/h, factor:0.1]
#define REG_MOTOR_ASSIST_WALK_LEVEL_MAX                        0x0D //Top level when assisting in walk mode [unit:%, factor:1.5625]

#define REG_MOTOR_STATUS_SPEED                                 0x11 //- [unit:rpm, factor:9.091]
#define REG_MOTOR_STATUS_POWER_METER                           0x14 //- [unit:%, factor:1.5625]

#define REG_MOTOR_STATUS_TEMPERATURE                           0x16 //- [unit:C]

#define REG_MOTOR_REV_HW                                       0x19 //hardware version
#define REG_MOTOR_REV_SW                                       0x20 //software version

#define REG_MOTOR_TORQUE_GAUGE_VALUE                           0x21 //- [unit:%, range:0?..100, factor:1.5625]

#define REG_MOTOR_REV_SUB                                      0x22 //software subversion

#define REG_MOTOR_CONFIG_COMMUNICATION_MODE_LO                 0x36 //- 8 bit until sw 83

#define REG_MOTOR_ASSIST_LOWSPEED_RAMP_FLAG                    0x40 //Enables a lower speed ramp. 0: Ramp disabled 1: Ramp enabled

#define REG_MOTOR_ASSIST_DIRECTION                             0x42 //-
#define REG_MOTOR_SN_STATOR_TYPE                               0x43 //-
#define REG_MOTOR_GEOMETRY_CIRC_HI                             0x44
#define REG_MOTOR_GEOMETRY_CIRC_LO                             0x45

#define REG_MOTOR_TORQUE_GAUGE_POLARITY                        0x46 //-

#define REG_MOTOR_STATUS_MAIN                                  0x47 //Indicates the current main status of the motor. 0-Running, 1-NoCommand, 2-Startup, 3-I2CShutOff, 4-AntiBackwardShort, 5-AlarmRegen, 6-AlarmShort, 7-OverSpeedI, 8-OverSpeedV, 9-V12UVP, 10-V12OVP, 11-VPwrUVP, 12-VPwrOVP, 13-OCProtect, 14-BadStatorPN, 15-HallError

#define REG_MOTOR_SN_ITEM_HI                                   0x60 //serial number
#define REG_MOTOR_SN_ITEM_LO                                   0x61
#define REG_MOTOR_SN_PN_HI                                     0x62 //partnumber
#define REG_MOTOR_SN_PN_LO                                     0x63
#define REG_MOTOR_SN_YEAR                                      0x64 //mfd. year
#define REG_MOTOR_SN_MONTH                                     0x65 //mfd. month
#define REG_MOTOR_SN_DAY                                       0x66 //mfd. day

#define REG_MOTOR_SN_OEM_HI                                    0x67 //OEM
#define REG_MOTOR_SN_OEM_LO                                    0x68
#define REG_MOTOR_SN_PRODUCT_HI                                0x69 //product
#define REG_MOTOR_SN_PRODUCT_LO                                0x6A
#define REG_MOTOR_SN_LOCATION                                  0x6B //Location

#define REG_MOTOR_TORQUE_GAUGE_TYPE                            0x6C //-

#define REG_MOTOR_ASSIST_STATOR_PN_HI                          0x6D //-
#define REG_MOTOR_ASSIST_STATOR_PN_LO                          0x6E //

#define REG_MOTOR_STATUS_POWER_VOLTAGE_HI                      0x70 //- [unit:V, factor:0.001]
#define REG_MOTOR_STATUS_POWER_VOLTAGE_LO                      0x71

#define REG_MOTOR_STATUS_12V_VOLTAGE_HI                        0x72 //- [unit:V, factor:0.001]
#define REG_MOTOR_STATUS_12V_VOLTAGE_LO                        0x73

#define REG_MOTOR_STATUS_5V_VOLTAGE_HI                         0x74 //- [unit:V, factor:0.001]
#define REG_MOTOR_STATUS_5V_VOLTAGE_LO                         0x75

#define REG_MOTOR_CONFIG_MAX_DISCHARGE_HI                      0x7A // Maximum drawn current on vPower [unit:A, factor:0.001]
#define REG_MOTOR_CONFIG_MAX_DISCHARGE_LO                      0x7B //

#define REG_MOTOR_CONFIG_MAX_CHARGE_HI                         0x7C //Maximum regen. current on vPower [unit:A, factor:0.001]
#define REG_MOTOR_CONFIG_MAX_CHARGE_LO                         0x7D //

#define REG_MOTOR_STATISTIC_MAX_POWER_VOLTAGE_HI               0x80 //- [unit:V, factor:0.001]
#define REG_MOTOR_STATISTIC_MAX_POWER_VOLTAGE_LO               0x81

#define REG_MOTOR_STATISTIC_MAX_TEMPERATURE_HI                 0x82 //-
#define REG_MOTOR_STATISTIC_MAX_TEMPERATURE_LO                 0x83

#define REG_MOTOR_STATISTIC_ODOMETER_HI                        0x84 //- [unit:km]
#define REG_MOTOR_STATISTIC_ODOMETER_LO                        0x85

#define REG_MOTOR_STATISTIC_CHRONO_HOURS_HI                    0x86 //- [unit:h]
#define REG_MOTOR_STATISTIC_CHRONO_HOURS_LO                    0x87

#define REG_MOTOR_STATISTIC_CHRONO_SECONDS_HI                  0x88 //- [unit:s]
#define REG_MOTOR_STATISTIC_CHRONO_SECONDS_LO                  0x89

#define REG_MOTOR_PREFERENCE_REGION                            0x8A //-

#define REG_MOTOR_ASSIST_MAXSPEED                              0x8B //- [unit:km/h]
#define REG_MOTOR_ASSIST_DYNAMIC_FLAG                          0x8C //- [range:0..1]

#define REG_MOTOR_CONFIG_PWM_LIMIT_ENABLE                      0x8D //-

#define REG_MOTOR_STATUS_CODES                                 0x92 //Indicates conditions currently detected by motor. Bit 0-Sensor saturation
#define REG_MOTOR_STATUS_CODES_LATCH                           0x93 //Indicates conditions detected by motor since its last power up. See bit description of status.codes


#define REG_MOTOR_PROTECT_UNLOCK                               0xA5 //unlock register write UNLOCK_KEY here before setting protected registers
#define MOTOR_PROTECT_UNLOCK_KEY                               0xAA
#define MOTOR_PROTECT_LOCK_KEY                                 0x00

#define REG_MOTOR_STATISTIC_HALL_DCHS_HI                       0xB0 //-
#define REG_MOTOR_STATISTIC_HALL_DCHS_LO                       0xB1

#define REG_MOTOR_STATISTIC_HALL_TRANS_HI                      0xB2 //-
#define REG_MOTOR_STATISTIC_HALL_TRANS_LO                      0xB3

#define REG_MOTOR_STATISTIC_HALL_RING_HI                       0xB4 //-
#define REG_MOTOR_STATISTIC_HALL_RING_LO                       0xB5

#define REG_MOTOR_STATISTIC_HALL_LOST_HI                       0xB6 //-
#define REG_MOTOR_STATISTIC_HALL_LOST_LO                       0xB7

#define REG_MOTOR_TORQUE_GAUGE_NOISE_HI                        0xC4 //- [unit:%, range:0..100, factor:0.0015259]
#define REG_MOTOR_TORQUE_GAUGE_NOISE_LO                        0xC5

#define REG_MOTOR_TORQUE_GAUGE_DELAY_HI                        0xC6 //- [unit:s, range:0..?, factor:0.001]
#define REG_MOTOR_TORQUE_GAUGE_DELAY_LO                        0xC7

#define REG_MOTOR_TORQUE_GAUGE_SPEED                           0xC8 //- [unit:rpm, range:0..?, factor:9.091]

#define REG_MOTOR_TORQUE_GAUGE_VOLTAGE_HI                      0xC9 //- [unit:V, range:0..5, factor:0.000076295, offset:5]
#define REG_MOTOR_TORQUE_GAUGE_VOLTAGE_LO                      0xCA

#define REG_MOTOR_TORQUE_GAUGE_REFERENCE_HI                    0xCB //- [unit:V, range:0..5, factor:0.000076295, offset:5]
#define REG_MOTOR_TORQUE_GAUGE_REFERENCE_LO                    0xCC

#define REG_MOTOR_CONFIG_COMMUNICATION_MODE_HI                 0xCD //Sets the communication mode. 0 for CAN and 0xca01 for I2C

#define REG_MOTOR_TORQUE_GAUGE_GAIN                            0xCE //- [unit:%, range:0..398, factor:1.5625]

#define REG_MOTOR_TORQUE_GAUGE_MAX_VOLTAGE                     0xE0 //Maximum voltage allowed for the sensor. When the sensor detect a voltage over this value for motor.torque.gaugeMaxVoltageDelay, it assumes an electrical failure and cuts assistance [unit:V, range:0..5, factor:0.019608]
#define REG_MOTOR_TORQUE_GAUGE_MAX_VOLTAGE_DELAY               0xE1 //Time after which a voltage over motor.torque.gaugeMaxVoltage is assumed to be an electrical failure, cutting assistance [unit:s, range:0..25.5, factor:0.1]

#define REG_MOTOR_ASSIST_LEVEL_OFFSLOPE_HI                     0xD0 //Speed at which the assist level set in the motor decreases when the console stops sending requests (when it is removed for example) [unit:%/s, factor:3.05]
#define REG_MOTOR_ASSIST_LEVEL_OFFSLOPE_LO                     0xD1

#define REG_MOTOR_ASSIST_REGEN_INFLEX                          0xD2 //Speed from which regen is not attenuated [unit:rpm, range:5..?, factor:9.091]

#define REG_MOTOR_ASSIST_MAXSPEED_DERATE_DELTA                 0xD3 //Speed before maxSpeed to start derating [unit:rpm, factor:9.091]
//----  /Motor


#define BXID_MOTOR            0x20
#define BXR_MOTOR_LEVEL       0x09
#define BXR_MOTOR_SWVERS      0x20  
// battery 


#define ID_BATTERY                  0x10

#define ID_BATTERY_RESPONSE                                    0x08
#define REG_BATTERY_CONFIG_ALLOW_BUCKCHARGING_ON_BIKE          0x12 //Specifies if the battery can recharge in buck mode even on a bike. Make sure it is IMPOSSIBLE to have an accessory output before setting this to 1. 0: Disallow, 1: Allow
#define REG_BATTERY_STATUS_CHARGER_MANAGER_STATUS              0x13 //Gives state of charging Mef: 0-Off, 1-Stand-by, 2-Charger, 3-Accessory, 4-Vdcin sense, 5-Overtemp, 6-Charge done, 7-Buck failed
#define REG_BATTERY_CONFIG_WAKE_ON_POWERVOLTAGE                0x14 //Specifies if the battery should wake up automatically when a voltage is present on the vPower. A value of 0 disables the feature

#define REG_BATTERY_STATUS_FLAGS_HI                            0x1D //Alert status bits: 0-Vctrl (code 20), 1-Precharge (code 21 and 67), 2-Relay (code 22), 3-BMS (code 23), 4-DCDC (code 28), 6-GG out of range temperature, 7-battery pack out of range temperature, 8-balancer overvolt (code 62), 9-Balancer undervolt (code 61), 10-Pack problem (code 63), 11-Accessory overcurrent (code 60), 12-Electronic fuse (code 66), 13-Balancer plug not connected, 14- +5v short(lached)

#define REG_BATTERY_STATUS_CELLPACK_CURRENT_HI                 0x1E //Reading battery current by a shunt resistor. No delay, no calibration compared to battery.gg.ai [unit:A, factor:0.001]
#define REG_BATTERY_STATUS_CELLPACK_CURRENT_LO                 0x1F //!!! signed !!!
#define REG_BATTERY_CONFIG_POWER_VOLTAGE_ENABLE                0x21 //- ??? Enable/Disable vPower ???
#define REG_BATTERY_CONFIG_ACCESSORY_ENABLED                   0x22 //-
#define REG_BATTERY_CONFIG_SHUTDOWN                            0x25 //write 1 to shutdwon system
#define REG_BATTERY_CONFIG_CONTROL_VOLTAGE_ENABLE              0x26 //Enable/Disable vControl
#define REG_BATTERY_STATUS_ESTIMATED_SOC                       0x30 //Return an estimated value of SOC based on battery voltage. Only works with LiIon battery [unit:%]
#define REG_BATTERY_STATUS_BATTERY_VOLTAGE_NORMALIZED          0x32 //Battery voltage normalized with 3.7V/cell. status.vBattInternal it used in Rev 104 and less otherwise status.vBatt [unit:V, factor:0.416667, offset:20.8333]
#define REG_BATTERY_STATISTIC_BATTERY_AVGVOLTAGE_NORMALIZED    0x33 //Average battery voltage read during 50s based on battery.status.vBatt, in percentage of its nominal voltage [unit:V, factor:0.416667, offset:20.8333]
#define REG_BATTERY_STATUS_INTERNAL_BATTERY_VOLTAGE_HI         0x4A //Reading of vBattInternal [unit:V, factor:0.001]
#define REG_BATTERY_STATUS_INTERNAL_BATTERY_VOLTAGE_LO         0x4B
#define REG_BATTERY_STATUS_CONSOLE_VOLTAGE_HI                  0x4C //Reading of vConsole (voltage applied to console) [unit:V, factor:0.001]
#define REG_BATTERY_STATUS_CONSOLE_VOLTAGE_LO                  0x4D

#define REG_BATTERY_STATUS_12V_VOLTAGE_HI                      0x4E //Reading of internal 12V [unit:V, factor:0.001]
#define REG_BATTERY_STATUS_12V_VOLTAGE_LO                      0x4F

#define REG_BATTERY_STATUS_BATTERY_VOLTAGE_HI                  0xA6 //Reading of vBatt. Return same value as vCell13 [unit:V, factor:0.001]
#define REG_BATTERY_STATUS_BATTERY_VOLTAGE_LO                  0xA7

#define REG_BATTERY_STATUS_POWER_VOLTAGE_HI                    0xAA //Reading of vPower ("High" voltage applied to motor) [unit:V, factor:0.001]
#define REG_BATTERY_STATUS_POWER_VOLTAGE_LO                    0xAB

#define REG_CELLMON_CELL_VOLTAGE_1                             0x81 //[unit:V, factor:0.001]
  //...                                                      //!!! signed !!!
#define REG_CELLMON_CELL_VOLTAGE_13                            0x8D //[unit:V, factor:0.001]