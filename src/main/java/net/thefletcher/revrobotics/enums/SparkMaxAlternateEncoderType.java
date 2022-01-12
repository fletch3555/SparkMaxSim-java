package net.thefletcher.revrobotics.enums;

public enum SparkMaxAlternateEncoderType {
    kQuadrature(0);

    @SuppressWarnings("MemberName")
    public final int value;

    SparkMaxAlternateEncoderType(int value) {
        this.value = value;
    }

    public static SparkMaxAlternateEncoderType fromId(int id) {
        return kQuadrature;
    }
}