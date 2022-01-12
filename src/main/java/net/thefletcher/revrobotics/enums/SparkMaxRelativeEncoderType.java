package net.thefletcher.revrobotics.enums;

public enum SparkMaxRelativeEncoderType {
    kNoSensor(0), kHallSensor(1), kQuadrature(2);

    @SuppressWarnings("MemberName")
    public final int value;

    SparkMaxRelativeEncoderType(int value) {
        this.value = value;
    }

    public static SparkMaxRelativeEncoderType fromId(int id) {
        switch (id) {
        case 1:
            return kHallSensor;
        case 2:
            return kQuadrature;
        default:
            return kNoSensor;
        }
    }
}