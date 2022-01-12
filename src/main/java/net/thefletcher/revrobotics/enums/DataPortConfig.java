package net.thefletcher.revrobotics.enums;

public enum DataPortConfig {
  kNone(-1, "none"),
  kLimitSwitches(0, "limit switches"),
  kAltEncoder(1, "alternate encoder");

  public final int m_value;
  public final String m_name;

  DataPortConfig(int value, String name) {
    m_value = value;
    m_name = name;
  }

  public static DataPortConfig fromInt(int id) {
    for (DataPortConfig type : values()) {
      if (type.m_value == id) {
        return type;
      }
    }
    return kNone;
  }
}