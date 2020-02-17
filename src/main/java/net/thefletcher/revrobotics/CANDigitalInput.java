/*
 * Copyright (c) 2018-2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package net.thefletcher.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.hal.*;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import net.thefletcher.revrobotics.enums.*;

public class CANDigitalInput implements Sendable {

  private final CANSparkMax m_device;
  private LimitSwitch m_limitSwitch = LimitSwitch.kForward;

  private SimDevice m_simDevice;
  private SimEnum m_simDirection;
  private SimEnum m_simPolarity;
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
  public CANDigitalInput(CANSparkMax device, LimitSwitch limitSwitch, LimitSwitchPolarity polarity) {
    m_device = device;
    m_limitSwitch = limitSwitch;

    if (m_device.m_altEncInitialized) {
      throw new IllegalArgumentException("Cannot instantiate a limit switch while using an alternative encoder");
    }

    m_simDevice = SimDevice.create((LimitSwitch.kForward.equals(limitSwitch) ? "Forward" : "Reverse") + "LimitSwitch", device.getDeviceId());
    if (m_simDevice != null) {
      m_simDirection = m_simDevice.createEnum("Direction", true, new String[] {"Forward", "Reverse"}, limitSwitch.value);
      m_simPolarity = m_simDevice.createEnum("Polarity", true, new String[] {"Normally Open", "Normally Closed"}, polarity.value);
      m_simEnabled = m_simDevice.createBoolean("Enabled", false, false);
      m_simValue = m_simDevice.createBoolean("Value", false, false);
    }
    else {
      CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetDataPortConfig(m_device.m_sparkMax, 0));
      CANSparkMaxJNI.c_SparkMax_SetLimitPolarity(m_device.m_sparkMax, limitSwitch.value, polarity.value);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(BuiltInWidgets.kBooleanBox.getWidgetName());
    builder.addBooleanProperty("Value", this::get, null);
  }

  /**
   * Get the value from a digital input channel.
   *
   * Retrieve the value of a single digital input channel from a motor controller.
   * This method will return the state of the limit input based on the selected
   * polarity, whether or not it is enabled.
   *
   * @return The state of the limit switch based on the configured polarity
   */
  public boolean get() {
    if (m_simDevice != null) {
      return m_simValue.get();
    }
    else {
      if (m_limitSwitch == LimitSwitch.kForward) {
        return m_device.getFault(FaultType.kHardLimitFwd);
      } else {
        return m_device.getFault(FaultType.kHardLimitRev);
      }
    }
  }

  /**
   * Enables or disables controller shutdown based on limit switch.
   *
   * @param enable Enable/disable motor shutdown based on limit switch state. This
   *               does not effect the result of the get() command.
   *
   * @return CANError Set to CANError::kOk if successful
   *
   */
  public CANError enableLimitSwitch(boolean enable) {
    if (m_simDevice != null) {
      m_simEnabled.set(enable);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_EnableLimitSwitch(m_device.m_sparkMax, m_limitSwitch.value, enable));
    }
  }

  /**
   * @return True if limit switch is enabled
   */
  public boolean isLimitSwitchEnabled() {
    if (m_simDevice != null) {
      return m_simEnabled.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_IsLimitEnabled(m_device.m_sparkMax, m_limitSwitch.value);
    }
  }
}
