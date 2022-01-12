package net.thefletcher.revrobotics.enums;

public enum FaultID {
  kBrownout(0),
  kOvercurrent(1),
  kIWDTReset(2),
  kMotorFault(3),
  kSensorFault(4),
  kStall(5),
  kEEPROMCRC(6),
  kCANTX(7),
  kCANRX(8),
  kHasReset(9),
  kDRVFault(10),
  kOtherFault(11),
  kSoftLimitFwd(12),
  kSoftLimitRev(13),
  kHardLimitFwd(14),
  kHardLimitRev(15);

  @SuppressWarnings("MemberName")
  public final int value;

  FaultID(int value) {
    this.value = value;
  }

  public static FaultID fromId(int id) {
    for (FaultID type : values()) {
      if (type.value == id) {
        return type;
      }
    }
    return null;
  }
}