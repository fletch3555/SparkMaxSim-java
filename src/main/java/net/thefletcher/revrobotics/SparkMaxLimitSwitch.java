package net.thefletcher.revrobotics;

import java.util.Locale;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.hal.*;
import edu.wpi.first.hal.SimDevice.Direction;
import net.thefletcher.revrobotics.enums.*;

public class SparkMaxLimitSwitch implements CANDigitalInput {

  private final CANSparkMax m_device;
  private final SparkMaxLimitSwitchDirection m_limitSwitch;
  final SparkMaxLimitSwitchType m_switchType;

  private SimDevice m_simDevice;
  private SimBoolean m_simEnabled;
  private SimBoolean m_simValue;

  /**
   * Constructs a CANDigitalInput.
   *
   * @param device      The Spark Max to which the limit switch is attached.
   * @param limitSwitch Whether this is forward or reverse limit switch.
   * @param polarity    Whether the limit switch is normally open or normally
   *                    closed.
   */
  public SparkMaxLimitSwitch(CANSparkMax device, SparkMaxLimitSwitchDirection direction, SparkMaxLimitSwitchType switchType) {
    m_device = device;
    m_limitSwitch = direction;
    m_switchType = switchType;

    if (direction == null) {
      throw new IllegalArgumentException("limitSwitch must not be null");
    }

    if (switchType == null) {
      throw new IllegalArgumentException("polarity must not be null");
    }

    m_simDevice = SimDevice.create("CANSparkMax[" + device.getDeviceId() + "] - " + (SparkMaxLimitSwitchDirection.kForward.equals(direction) ? "Forward" : "Reverse") + "LimitSwitch");
    if (m_simDevice != null) {
      m_simEnabled = m_simDevice.createBoolean("Enabled", Direction.kOutput, false);
      m_simValue = m_simDevice.createBoolean("Value", Direction.kBidir, false);
    }
    else {
      REVLibError error = REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_AttemptToSetDataPortConfig(
          device.m_sparkMax, DataPortConfig.kLimitSwitches.m_value
        )
      );
      
      if (error == REVLibError.kSparkMaxDataPortAlreadyConfiguredDifferently) {
        DataPortConfig currentConfig = DataPortConfig.fromInt(
            CANSparkMaxJNI.c_SparkMax_GetDataPortConfig(device.m_sparkMax));
        throw new IllegalStateException(
          String.format(
            Locale.ENGLISH,
            "Limit switches cannot be used while %s is active",
            currentConfig.m_name
          )
        );
      }
      
      CANSparkMaxJNI.c_SparkMax_SetLimitPolarity(
        m_device.m_sparkMax, direction.value, switchType.value);
    }
  }
  /**
   * Returns {@code true} if the limit switch is pressed, based on the selected polarity.
   *
   * <p>This method works even if the limit switch is not enabled for controller shutdown.
   *
   * @return {@code true} if the limit switch is pressed
   */
  public boolean isPressed() {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return m_simValue.get();
    }
    else {
      if (m_limitSwitch == SparkMaxLimitSwitchDirection.kForward) {
        return m_device.getFault(FaultID.kHardLimitFwd);
      } else {
        return m_device.getFault(FaultID.kHardLimitRev);
      }
    }
  }

  /**
   * Returns {@code true} if the limit switch is pressed, based on the selected polarity.
   *
   * <p>This method works even if the limit switch is not enabled.
   *
   * @return {@code true} if the limit switch is pressed
   * @deprecated Use {@link #isPressed()} instead
   */
  @Override
  @Deprecated(forRemoval = true)
  public boolean get() {
    return isPressed();
  }

  /**
   * Enables or disables controller shutdown based on the limit switch.
   *
   * @param enable Enable/disable motor shutdown based on the limit switch state. This does not
   *     affect the result of the get() command.
   * @return {@link REVLibError#kOk} if successful
   */
  @Override
  public REVLibError enableLimitSwitch(boolean enable) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      m_simEnabled.set(enable);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_EnableLimitSwitch(
          m_device.m_sparkMax, m_limitSwitch.value, enable));
    }
  }

  /**
   * @return True if limit switch is enabled
   */
  public boolean isLimitSwitchEnabled() {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return m_simEnabled.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_IsLimitEnabled(m_device.m_sparkMax, m_limitSwitch.value);
    }
  }
}
