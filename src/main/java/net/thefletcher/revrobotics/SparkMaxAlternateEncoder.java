package net.thefletcher.revrobotics;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.SimDevice.Direction;
import net.thefletcher.revrobotics.enums.*;

import java.util.Locale;

// package-private to the revrobotics package
public class SparkMaxAlternateEncoder implements RelativeEncoder {
  // package-private to the revrobotics package
  final CANSparkMax sparkMax;
  final SparkMaxAlternateEncoderType type;
  final int countsPerRev;
  
  private SimDevice m_simDevice;
  private SimDouble m_simPosition;
  private SimDouble m_simVelocity;
  private SimDouble m_simPositionConversionFactor;
  private SimDouble m_simVelocityConversionFactor;
  private SimInt m_simAverageDepth;
  private SimInt m_simMeasurementPeriod;
  private SimBoolean m_simInverted;

  // package-private to the revrobotics package
  SparkMaxAlternateEncoder(CANSparkMax sparkMax, SparkMaxAlternateEncoderType type, int countsPerRev) {

    m_simDevice = SimDevice.create("CANSparkMax[" + sparkMax.getDeviceId() + "] - AlternateEncoder");

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

    if (countsPerRev == 0) {
      throw new IllegalArgumentException("countsPerRev must not be zero");
    }

    if (m_simDevice == null) {
      REVLibError error = REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_AttemptToSetDataPortConfig(sparkMax.m_sparkMax, DataPortConfig.kAltEncoder.m_value));
      if (error == REVLibError.kSparkMaxDataPortAlreadyConfiguredDifferently) {
        DataPortConfig currentConfig = DataPortConfig.fromInt(CANSparkMaxJNI.c_SparkMax_GetDataPortConfig(sparkMax.m_sparkMax));
        throw new IllegalStateException(String.format(Locale.ENGLISH, "An alternate encoder cannot be used while %s is active", currentConfig.m_name));
      }

      CANSparkMaxJNI.c_SparkMax_SetAltEncoderCountsPerRevolution(sparkMax.m_sparkMax, countsPerRev);
    }
    // If we ever add additional entries to the AlternateEncoderType enum, we need to use the
    // encoderType param.
  }

  public double getPosition() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simPosition.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAltEncoderPosition(sparkMax.m_sparkMax);
    }
  }

  public double getVelocity() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simVelocity.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAltEncoderVelocity(sparkMax.m_sparkMax);
    }
  }

  public REVLibError setPosition(double position) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simPosition.set(position);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderPosition(sparkMax.m_sparkMax, (float) position));
    }
  }

  public REVLibError setPositionConversionFactor(double factor) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simPositionConversionFactor.set(factor);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderPositionFactor(sparkMax.m_sparkMax, (float) factor));
    }
  }

  public REVLibError setVelocityConversionFactor(double factor) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simVelocityConversionFactor.set(factor);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderVelocityFactor(sparkMax.m_sparkMax, (float) factor));
    }
  }

  public double getPositionConversionFactor() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simPositionConversionFactor.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAltEncoderPositionFactor(sparkMax.m_sparkMax);
    }
  }

  public double getVelocityConversionFactor() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simVelocityConversionFactor.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAltEncoderVelocityFactor(sparkMax.m_sparkMax);
    }
  }

  public REVLibError setAverageDepth(int depth) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simAverageDepth.set(depth);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderAverageDepth(sparkMax.m_sparkMax, depth));
    }
  }

  public int getAverageDepth() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simAverageDepth.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAltEncoderAverageDepth(sparkMax.m_sparkMax);
    }
  }

  public REVLibError setMeasurementPeriod(int period_us) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simMeasurementPeriod.set(period_us);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderMeasurementPeriod(sparkMax.m_sparkMax, period_us));
    }
  }

  public int getMeasurementPeriod() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return countsPerRev;
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAltEncoderMeasurementPeriod(sparkMax.m_sparkMax);
    }
  }

  public int getCountsPerRevolution() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return countsPerRev;
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAltEncoderCountsPerRevolution(sparkMax.m_sparkMax);
    }
  }

  @Override
  public REVLibError setInverted(boolean inverted) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simInverted.set(inverted);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderInverted(sparkMax.m_sparkMax, inverted));
    }
  }

  @Override
  public boolean getInverted() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simInverted.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAltEncoderInverted(sparkMax.m_sparkMax);
    }
  }

  // package-private to the revrobotics package
  int getSparkMaxFeedbackDeviceId() {
    return FeedbackSensorType.kQuadrature.value;
  }
}
