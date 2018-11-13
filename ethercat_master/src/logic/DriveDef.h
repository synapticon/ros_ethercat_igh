//
// Created by alexey on 5-11-18.
//

#ifndef CIA402_DRIVEDEF_H
#define CIA402_DRIVEDEF_H

// Abstracted drive commands

enum DriveCommand {
  DRIVE_CMD_OFF         = 0x0,
  DRIVE_CMD_DISENGAGE   = 0x1,
  DRIVE_CMD_ENGAGE      = 0x2,
  DRIVE_CMD_FAULT_ACK   = 0x3,
  DRIVE_CMD_QUICK_STOP  = 0x4,
};

// Abstracted drive states

enum DriveState {
  DRIVE_STATUS_OFF                  = 0x0,
  DRIVE_STATUS_DISENGAGED           = 0x1,
  DRIVE_STATUS_ENGAGED              = 0x2,
  DRIVE_STATUS_FAULT                = 0x3,
  DRIVE_STATUS_QUICK_STOP_ACTIVE    = 0x4,
};

template <typename T, typename... Args>
T bitIsSet(T value, Args... bits) {
  for (auto&& bit : { bits... }) {
    if (!((value >> bit) & 1U)) {
      return false;
    }
  }
  return true;
}

template <typename T, typename... Args>
T bitIsCleared(T value, Args... bits) {
  return !bitIsSet(value, bits...);
}

template <typename T, typename... Args>
T setBit(T value, Args... bits) {
  for (auto&& bit : { bits... }) {
    value |= 1UL << bit;
  }
  return value;
}

template <typename T, typename... Args>
T clearBit(T value, Args... bits) {
  for (auto&& bit : { bits... }) {
    value &= ~(1UL << bit);
  }
  return value;
}


#endif //CIA402_DRIVEDEF_H
