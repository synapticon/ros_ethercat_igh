//
// Created by alexey on 5-11-18.
//

#ifndef CIA402_CIA402FSMDATA_H
#define CIA402_CIA402FSMDATA_H

#endif //CIA402_CIA402FSMDATA_H

#include "DriveDef.h"
#include "Cia402FsmTransition.h"

/*
 * Command        Bit 7         Bit 3         Bit 2         Bit 1         Bit 0         Transition
 *                Fault Reset   Enable Oper.  Quick Stop    Disable Volt. Switch on
 *
 * Shutdown       X             X             1             1             0             2,6,8
 * Switch on      X             X             1             1             1             3
 * Disable Volt.  X             X             X             0             X             7,9,10,12
 * Quick Stop     X             X             0             1             X             7,10,11
 * Disable Oper.  X             0             1             1             1             5
 * Enable Oper.   X             1             1             1             1             4,16
 * Fault Reset    1             X             X             X             X             15
 */

enum Cia402Command {
  CIA402_CMD_SWITCH_ON              = 0,
  CIA402_CMD_DISABLE_VOLTAGE        = 1,
  CIA402_CMD_QUICK_STOP             = 2,
  CIA402_CMD_ENABLE_OPERATION       = 3,
  CIA402_CMD_FAULT_REST             = 7
};

enum Cia402Status {
  Cia402_STATUS_READY_TO_SWITCH_ON      = 0,
  Cia402_STATUS_SWITCHED_ON             = 1,
  Cia402_STATUS_OPERATION_ENABLED       = 2,
  Cia402_STATUS_FAULT                   = 3,
  Cia402_STATUS_VOLTAGE_DISABLED        = 4,
  Cia402_STATUS_QUICK_STOP_NOT_ACTIVE   = 5,
  Cia402_STATUS_SWITCH_ON_DISABLED      = 6,
  Cia402_STATUS_WARNING                 = 7,
};

struct Cia402FsmData {
  Transition* transition;
  DriveCommand drive_cmd;
  DriveState drive_state;
  uint32_t cia402_cmd;
  uint32_t cia402_opmode;
  uint32_t cia402_status;
  struct {
    double fault_ack{1};
    double shutdown{1};
    double switch_on{1};
    double disable_voltage{1};
    double quick_stop{1};
    double disable_operation{1};
    double enable_operation{1};
  } timeout_sec;
};