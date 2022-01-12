package net.thefletcher.revrobotics.enums;

public enum SparkMaxLimitSwitchDirection {
    kForward(0), kReverse(1);

    @SuppressWarnings("MemberName")
    public final int value;

    SparkMaxLimitSwitchDirection(int value) {
      this.value = value;
    }
  }