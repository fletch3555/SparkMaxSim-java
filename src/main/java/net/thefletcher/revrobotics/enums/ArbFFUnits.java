package net.thefletcher.revrobotics.enums;

public enum ArbFFUnits {
  kVoltage(0), kPercentOut(1);

  @SuppressWarnings("MemberName")
  public final int value;

  ArbFFUnits(int value) {
    this.value = value;
  }
}