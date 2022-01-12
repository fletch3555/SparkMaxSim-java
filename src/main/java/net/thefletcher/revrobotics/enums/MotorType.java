package net.thefletcher.revrobotics.enums;

public enum MotorType {
  kBrushed(0), kBrushless(1);

  @SuppressWarnings("MemberName")
  public final int value;

  MotorType(int value) {
    this.value = value;
  }

  public static MotorType fromId(int id) {
    for (MotorType type : values()) {
      if (type.value == id) {
        return type;
      }
    }
    return null;
  }
}