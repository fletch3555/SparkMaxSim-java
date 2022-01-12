package net.thefletcher.revrobotics.enums;

public enum PeriodicFrame {
  kStatus0(0), kStatus1(1), kStatus2(2);

  @SuppressWarnings("MemberName")
  public final int value;

  PeriodicFrame(int value) {
    this.value = value;
  }

  public static PeriodicFrame fromId(int id) {
    for (PeriodicFrame type : values()) {
      if (type.value == id) {
        return type;
      }
    }
    return null;
  }
}