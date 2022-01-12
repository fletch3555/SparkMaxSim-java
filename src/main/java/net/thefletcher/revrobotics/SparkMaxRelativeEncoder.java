package net.thefletcher.revrobotics;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.SimDevice.Direction;
import net.thefletcher.revrobotics.enums.FeedbackSensorType;
import net.thefletcher.revrobotics.enums.MotorType;
import net.thefletcher.revrobotics.enums.SparkMaxRelativeEncoderType;

/**
 * Get an instance of this class by using {@link CANSparkMax#getEncoder()} or {@link
 * CANSparkMax#getEncoder(Type, int)}.
 */
public class SparkMaxRelativeEncoder implements RelativeEncoder {

  // package-private to the revrobotics package
  final SparkMaxRelativeEncoderType type;
  final int countsPerRev;
  final CANSparkMax sparkMax;

  private final SimDevice m_simDevice;
  private SimDouble m_simPosition;
  private SimDouble m_simVelocity;
  private SimDouble m_simPositionConversionFactor;
  private SimDouble m_simVelocityConversionFactor;
  private SimInt m_simAverageDepth;
  private SimInt m_simMeasurementPeriod;
  private SimBoolean m_simInverted;

  // package-private to the revrobotics package
  SparkMaxRelativeEncoder(CANSparkMax sparkMax, SparkMaxRelativeEncoderType type, int countsPerRev) {

    m_simDevice = SimDevice.create("CANSparkMax[" + sparkMax.getDeviceId() + "] - RelativeEncoder");

    if (m_simDevice != null) {
      m_simPosition = m_simDevice.createDouble("Position", Direction.kOutput, 0.0);
      m_simVelocity = m_simDevice.createDouble("Velocity", Direction.kOutput, 0.0);
      m_simPositionConversionFactor = m_simDevice.createDouble("Position Conversion Factor", Direction.kOutput, 1.0);
      m_simVelocityConversionFactor = m_simDevice.createDouble("Idle Mode", Direction.kOutput, 1.0);
      m_simAverageDepth = m_simDevice.createInt("Average Depth", Direction.kOutput, 1);
      m_simMeasurementPeriod = m_simDevice.createInt("Measurement Period", Direction.kOutput, 0);
      m_simInverted = m_simDevice.createBoolean("Inverted", Direction.kOutput, sparkMax.getInverted());
    }

    this.sparkMax = sparkMax;
    this.type = type;
    this.countsPerRev = countsPerRev;

    if (type == null) {
      throw new IllegalArgumentException("encoderType must not be null");
    }

    boolean setCountsPerRevParameter = true;

    if (type == SparkMaxRelativeEncoderType.kHallSensor) {
      // The CPR for a hall sensor is hardcoded in the SPARK MAX firmware as 42
      setCountsPerRevParameter = false;

      if (sparkMax.motorType == MotorType.kBrushed) {
        throw new IllegalStateException("A hall sensor cannot be used with a brushed motor");
      }

      // Make sure that the correct CPR value was provided
      if (countsPerRev != 42 && countsPerRev != 0) {
        throw new IllegalArgumentException("countsPerRev must be 42 when using the hall sensor");
      }
    } else { // non-hall sensor
      if (sparkMax.motorType == MotorType.kBrushless) {
        throw new IllegalStateException(
            "The encoder type must be kHallSensor when the SPARK MAX is configured in brushless mode.\n"
                + "To use an external quadrature encoder with a brushless motor, you must wire it as an Alternate Encoder, and then call getAlternateEncoder() on the CANSparkMax object.");
      }

      if (countsPerRev < 1) {
        throw new IllegalArgumentException("countsPerRev must be a positive number");
      }
    }

    if (m_simDevice == null) {
      CANSparkMaxJNI.c_SparkMax_SetSensorType(sparkMax.m_sparkMax, type.value);
      if (setCountsPerRevParameter) {
        CANSparkMaxJNI.c_SparkMax_SetCountsPerRevolution(sparkMax.m_sparkMax, countsPerRev);
      }
    }
  }

  @Override
  public double getPosition() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simPosition.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetEncoderPosition(sparkMax.m_sparkMax);
    }
  }

  @Override
  public double getVelocity() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simVelocity.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetEncoderVelocity(sparkMax.m_sparkMax);
    }
  }

  @Override
  public REVLibError setPosition(double position) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simPosition.set(position);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetEncoderPosition(sparkMax.m_sparkMax, (float) position));
    }
  }

  @Override
  public REVLibError setPositionConversionFactor(double factor) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simPositionConversionFactor.set(factor);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetPositionConversionFactor(sparkMax.m_sparkMax, (float) factor));
    }
  }

  @Override
  public REVLibError setVelocityConversionFactor(double factor) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simVelocityConversionFactor.set(factor);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetVelocityConversionFactor(sparkMax.m_sparkMax, (float) factor));
    }
  }

  @Override
  public double getPositionConversionFactor() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simPositionConversionFactor.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetPositionConversionFactor(sparkMax.m_sparkMax);
    }
  }

  @Override
  public double getVelocityConversionFactor() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simVelocityConversionFactor.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetVelocityConversionFactor(sparkMax.m_sparkMax);
    }
  }

  @Override
  public REVLibError setAverageDepth(int depth) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simAverageDepth.set(depth);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(
          CANSparkMaxJNI.c_SparkMax_SetAverageDepth(sparkMax.m_sparkMax, depth));
    }
  }

  @Override
  public int getAverageDepth() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simAverageDepth.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAverageDepth(sparkMax.m_sparkMax);
    }
  }

  @Override
  public REVLibError setMeasurementPeriod(int period_us) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simMeasurementPeriod.set(period_us);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetMeasurementPeriod(sparkMax.m_sparkMax, period_us));
    }
  }

  @Override
  public int getMeasurementPeriod() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simMeasurementPeriod.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetMeasurementPeriod(sparkMax.m_sparkMax);
    }
  }

  @Override
  public int getCountsPerRevolution() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return countsPerRev;
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetCountsPerRevolution(sparkMax.m_sparkMax);
    }
  }

  @Override
  public REVLibError setInverted(boolean inverted) {
    sparkMax.throwIfClosed();
    if (sparkMax.getMotorType() == MotorType.kBrushless) {
      throw new IllegalArgumentException(
          "The encoder cannot be inverted separately from the motor in Brushless Mode");
    }
    if (m_simDevice != null) {
      m_simInverted.set(inverted);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetEncoderInverted(sparkMax.m_sparkMax, inverted));
    }
  }

  @Override
  public boolean getInverted() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simInverted.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetEncoderInverted(sparkMax.m_sparkMax);
    }
  }

  // package-private to the revrobotics package
  int getSparkMaxFeedbackDeviceId() {
    return (sparkMax.getMotorType() == MotorType.kBrushless)
        ? FeedbackSensorType.kHallSensor.value
        : FeedbackSensorType.kQuadrature.value;
  }
}
