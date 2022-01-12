package net.thefletcher.revrobotics.enums;

public enum SoftLimitDirection {
  kForward(0), kReverse(1);

  @SuppressWarnings("MemberName")
  public final int value;

  SoftLimitDirection(int value) {
    this.value = value;
  }

  public static SoftLimitDirection fromID(int id) {
    if (id == 1) {
      return kReverse;
    }
    return kForward;
  }
}