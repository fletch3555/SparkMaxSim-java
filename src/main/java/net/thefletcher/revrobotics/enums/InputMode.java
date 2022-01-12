package net.thefletcher.revrobotics.enums;

public enum InputMode {
  kPWM(0), kCAN(1);

  @SuppressWarnings("MemberName")
  public final int value;

  InputMode(int value) {
    this.value = value;
  }

  public static InputMode fromId(int id) {
    if (id == 1) {
      return kCAN;
    }
    return kPWM;
  }
}