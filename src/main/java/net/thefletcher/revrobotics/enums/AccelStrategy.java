package net.thefletcher.revrobotics.enums;

public enum AccelStrategy {
  kTrapezoidal(0), kSCurve(1);

  @SuppressWarnings("MemberName")
  public final int value;

  AccelStrategy(int value) {
    this.value = value;
  }

  public static AccelStrategy fromInt(int value) {
    switch (value) {
    case 0:
      return kTrapezoidal;
    case 1:
      return kSCurve;
    default:
      return kTrapezoidal;
    }
  }
}