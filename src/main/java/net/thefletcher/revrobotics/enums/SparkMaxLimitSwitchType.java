package net.thefletcher.revrobotics.enums;

public enum SparkMaxLimitSwitchType {
  kNormallyOpen(0),
  kNormallyClosed(1);

  @SuppressWarnings("MemberName")
  public final int value;

  SparkMaxLimitSwitchType(int value) {
    this.value = value;
  }
}