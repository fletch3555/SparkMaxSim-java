package net.thefletcher.revrobotics.enums;

public enum FeedbackSensorType {
    kNoSensor(0), kHallSensor(1), kQuadrature(2), kSensorless(3), kAnalog(4), kAltQuadrature(5);

    @SuppressWarnings("MemberName")
    public final int value;

    FeedbackSensorType(int value) {
        this.value = value;
    }

    public static FeedbackSensorType fromId(int id) {
        switch (id) {
        case 1:
            return kHallSensor;
        case 2:
            return kQuadrature;
        case 3: 
            return kSensorless;
        case 4: 
            return kAnalog;
        case 5:
            return kAltQuadrature;
        default:
            return kNoSensor;
        }
    }
}