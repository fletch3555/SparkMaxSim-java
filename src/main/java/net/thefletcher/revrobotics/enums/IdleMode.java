package net.thefletcher.revrobotics.enums;

public enum IdleMode {
  kCoast(0), kBrake(1);

  @SuppressWarnings("MemberName")
  public final int value;

  IdleMode(int value) {
    this.value = value;
  }

  public static IdleMode fromId(int id) {
    if (id == 1) {
      return kBrake;
    }
    return kCoast;
  }
}