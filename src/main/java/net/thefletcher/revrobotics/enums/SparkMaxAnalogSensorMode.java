package net.thefletcher.revrobotics.enums;

public enum SparkMaxAnalogSensorMode {
    kAbsolute(0),
    kRelative(1);

    @SuppressWarnings("MemberName")
    public final int value;

    SparkMaxAnalogSensorMode(int value) {
        this.value = value;
    }

    public static SparkMaxAnalogSensorMode fromId(int id) {
        if (id == 1) {
            return kRelative;
        }
        return kAbsolute;
    }
}