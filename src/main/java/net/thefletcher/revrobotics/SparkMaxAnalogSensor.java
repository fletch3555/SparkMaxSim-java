package net.thefletcher.revrobotics;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANAnalog;
import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import net.thefletcher.revrobotics.enums.FeedbackSensorType;
import net.thefletcher.revrobotics.enums.SparkMaxAnalogSensorMode;

/** Get an instance of this class by using {@link CANSparkMax#getAnalog(Mode)}. */
public class SparkMaxAnalogSensor implements AnalogInput, CANAnalog {

  // package-private to the revrobotics package
  final SparkMaxAnalogSensorMode mode;
  final CANSparkMax sparkMax;

  private SimDevice m_simDevice;
  private SimDouble m_simVoltage;
  private SimDouble m_simPosition;
  private SimDouble m_simVelocity;
  private SimDouble m_simPositionConversionFactor;
  private SimDouble m_simVelocityConversionFactor;
  private SimBoolean m_simInverted;

  // package-private (can only be used by other classes in this package)
  SparkMaxAnalogSensor(CANSparkMax sparkMax, SparkMaxAnalogSensorMode mode) {
    m_simDevice = SimDevice.create("CANSparkMax[" + sparkMax.getDeviceId() + "] - SparkMaxAnalogSensor");
    if (m_simDevice != null) {
      m_simVoltage = m_simDevice.createDouble("Voltage", Direction.kOutput, 0.0);
      m_simPosition = m_simDevice.createDouble("Position", Direction.kOutput, 0.0);
      m_simVelocity = m_simDevice.createDouble("Velocity", Direction.kOutput, 0.0);
      m_simPositionConversionFactor = m_simDevice.createDouble("Position Conversion Factor", Direction.kOutput, 1.0);
      m_simVelocityConversionFactor = m_simDevice.createDouble("Idle Mode", Direction.kOutput, 1.0);
      m_simInverted = m_simDevice.createBoolean("Inverted", Direction.kOutput, sparkMax.getInverted());
    }

    this.sparkMax = sparkMax;
    this.mode = mode;

    if (mode == null) {
      throw new IllegalArgumentException("mode must not be null");
    }

    if (m_simDevice == null) {
      CANSparkMaxJNI.c_SparkMax_SetAnalogMode(sparkMax.m_sparkMax, mode.value);
    }
  }

  /**
   * Get the voltage of the analog sensor.
   *
   * @return Voltage of the sensor.
   */
  @Override
  public double getVoltage() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simVoltage.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAnalogVoltage(sparkMax.m_sparkMax);
    }
  }

  /**
   * Get the position of the sensor. Returns value in the native unit of 'volt' by default, and can
   * be changed by a scale factor using setPositionConversionFactor().
   *
   * @return Position of the sensor
   */
  @Override
  public double getPosition() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simPosition.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAnalogPosition(sparkMax.m_sparkMax);
    }
  }

  /**
   * Get the velocity of the sensor. Returns value in the native units of 'volts per second' by
   * default, and can be changed by a scale factor using setVelocityConversionFactor().
   *
   * @return Velocity of the sensor in volts per second
   */
  @Override
  public double getVelocity() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simVelocity.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAnalogVelocity(sparkMax.m_sparkMax);
    }
  }

  /**
   * Set the conversion factor for the position of the analog sensor. By default, revolutions per
   * volt is 1. Changing the position conversion factor will also change the position units.
   *
   * @param factor The conversion factor which will be multiplied by volts
   * @return {@link REVLibError#kOk} if successful
   */
  @Override
  public REVLibError setPositionConversionFactor(double factor) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simPositionConversionFactor.set(factor);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAnalogPositionConversionFactor(sparkMax.m_sparkMax, (float) factor));
    }
  }

  /**
   * Set the conversion factor for the velocity of the analog sensor. By default, revolutions per
   * volt second is 1. Changing the velocity conversion factor will also change the velocity units.
   *
   * @param factor The conversion factor which will be multiplied by volts per second
   * @return {@link REVLibError#kOk} if successful
   */
  @Override
  public REVLibError setVelocityConversionFactor(double factor) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simVelocityConversionFactor.set(factor);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAnalogVelocityConversionFactor(sparkMax.m_sparkMax, (float) factor));
    }
  }

  /**
   * Get the current conversion factor for the position of the analog sensor.
   *
   * @return Analog position conversion factor
   */
  @Override
  public double getPositionConversionFactor() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simPositionConversionFactor.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAnalogPositionConversionFactor(sparkMax.m_sparkMax);
    }
  }

  /**
   * Get the current conversion factor for the velocity of the analog sensor.
   *
   * @return Analog velocity conversion factor
   */
  @Override
  public double getVelocityConversionFactor() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simVelocityConversionFactor.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAnalogVelocityConversionFactor(sparkMax.m_sparkMax);
    }
  }

  /**
   * Set the phase of the anlog sensor so that it is set to be in phase with the motor itself
   *
   * @param inverted The phase of the sensor
   * @return {@link REVLibError#kOk} if successful
   */
  @Override
  public REVLibError setInverted(boolean inverted) {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      m_simInverted.set(inverted);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAnalogInverted(sparkMax.m_sparkMax, inverted));
    }
  }

  /**
   * Get the phase of the analog sensor
   *
   * @return The phase of the sensor
   */
  @Override
  public boolean getInverted() {
    sparkMax.throwIfClosed();
    if (m_simDevice != null) {
      return m_simInverted.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetAnalogInverted(sparkMax.m_sparkMax);
    }
  }

  // package-private to the revrobotics package
  int getSparkMaxFeedbackDeviceId() {
    return FeedbackSensorType.kAnalog.value;
  }
}
