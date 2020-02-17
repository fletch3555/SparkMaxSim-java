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
import java.util.HashMap;
import java.util.Map;
import net.thefletcher.revrobotics.enums.*;

public class CANPIDController {
  class SimPIDSlot {
    public SimDouble pGain;
    public SimDouble iGain;
    public SimDouble iZone;
    public SimDouble dGain;
    public SimDouble dFilter;
    public SimDouble ffGain;
    public SimDouble outputMin;
    public SimDouble outputMax;
    public SimDouble smartMotionMaxVelocity;
    public SimDouble smartMotionMaxAccel;
    public SimDouble smartMotionMinOutputVelocity;
    public SimDouble smartMotionAllowedClosedLoopError;
    public SimEnum smartMotionAccelStrategy;
    public SimDouble iMaxAccum;

    public SimPIDSlot(int slot, SimDevice simDevice) {
      if (simDevice != null) {
        pGain = simDevice.createDouble("[" + slot + "] P Gain", false, 0.0);
        iGain = simDevice.createDouble("[" + slot + "] I Gain", false, 0.0);
        iZone = simDevice.createDouble("[" + slot + "] I Zone", false, 0.0);
        dGain = simDevice.createDouble("[" + slot + "] D Gain", false, 0.0);
        dFilter = simDevice.createDouble("[" + slot + "] D Filter", false, 0.0);
        ffGain = simDevice.createDouble("[" + slot + "] FF Gain", false, 0.0);
        outputMin = simDevice.createDouble("[" + slot + "] Min Output", false, -1.0);
        outputMax = simDevice.createDouble("[" + slot + "] Max Output", false, 1.0);
        smartMotionMaxVelocity = simDevice.createDouble("[" + slot + "] SmartMotion Max Velocity", false, 0.0);
        smartMotionMaxAccel = simDevice.createDouble("[" + slot + "] SmartMotion Max Acceleration", false, 0.0);
        smartMotionMinOutputVelocity = simDevice.createDouble("[" + slot + "] SmartMotion Min Output Velocity", false, 0.0);
        smartMotionAllowedClosedLoopError = simDevice.createDouble("[" + slot + "] SmartMotion Allowed Closed-Loop Error", false, 0.0);
      }
      else {
        throw new RuntimeException("Unable to use Sim slot when not in simulation");
      }
    }
  }

  private final CANSparkMax m_device;

  private SimDevice m_simDevice;
  private final Map<Integer, SimPIDSlot> m_simSlots = new HashMap<>();
  private SimDouble m_simIAccum;

  /**
   * Constructs a CANPIDController.
   *
   * @param device The Spark Max this object configures.
   */
  public CANPIDController(CANSparkMax device) {
    m_device = device;

    m_simDevice = SimDevice.create("CANPIDController", device.getDeviceId());
    if (m_simDevice != null) {
      m_simSlots.put(0, new SimPIDSlot(0, m_simDevice));
      m_simSlots.put(1, new SimPIDSlot(1, m_simDevice));
      m_simSlots.put(2, new SimPIDSlot(2, m_simDevice));
      m_simSlots.put(3, new SimPIDSlot(3, m_simDevice));

      m_simIAccum = m_simDevice.createDouble("I Accum", true, 0.0);
    }
  }

  /**
   * Set the controller reference value based on the selected control mode.
   *
   * @param value The value to set depending on the control mode. For basic
   * duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). Native units can be changed using the setPositionConversionFactor()
   * or setVelocityConversionFactor() methods of the CANEncoder class
   *
   * @param ctrl Is the control type
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setReference(double value, ControlType ctrl) {
    return setReference(value, ctrl, 0);
  }

  /**
   * Set the controller reference value based on the selected control mode.
   * This will override the pre-programmed control mode but not change what
   * is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic
   * duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). Native units can be changed using the setPositionConversionFactor()
   * or setVelocityConversionFactor() methods of the CANEncoder class
   *
   * @param ctrl Is the control type to override with
   *
   * @param pidSlot for this command
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setReference(double value, ControlType ctrl, int pidSlot) {
    return setReference(value, ctrl, pidSlot, 0);
  }

  /**
   * Set the controller reference value based on the selected control mode.
   * This will override the pre-programmed control mode but not change what
   * is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic
   * duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). Native units can be changed using the setPositionConversionFactor()
   * or setVelocityConversionFactor() methods of the CANEncoder class
   *
   * @param ctrl Is the control type to override with
   *
   * @param pidSlot for this command
   *
   * @param arbFeedforward A value from which is represented in voltage
   * applied to the motor after the result of the specified control mode. The
   * units for the parameter is Volts. This value is set after the control mode,
   * but before any current limits or ramp rates.
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setReference(double value, ControlType ctrl, int pidSlot,
                               double arbFeedforward) {
    return m_device.setpointCommand(value, ctrl, pidSlot, arbFeedforward);
  }

  /**
   * Set the controller reference value based on the selected control mode.
   * This will override the pre-programmed control mode but not change what
   * is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic
   * duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). Native units can be changed using the setPositionConversionFactor()
   * or setVelocityConversionFactor() methods of the CANEncoder class
   *
   * @param ctrl Is the control type to override with
   *
   * @param pidSlot for this command
   *
   * @param arbFeedforward A value from which is represented in voltage
   * applied to the motor after the result of the specified control mode. The
   * units for the parameter is Volts. This value is set after the control mode,
   * but before any current limits or ramp rates.
   * 
   * @param arbFFUnits The units the arbitrary feed forward term is in
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setReference(double value, ControlType ctrl, int pidSlot,
                               double arbFeedforward, ArbFFUnits arbFFUnits) {
    return m_device.setpointCommand(value, ctrl, pidSlot, arbFeedforward, arbFFUnits.value);
  }

  /**
   * Set the Proportional Gain constant of the PIDF controller on the SPARK
   * MAX. This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The proportional gain value, must be positive
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setP(double gain) {
    return setP(gain, 0);
  }

  /**
   * Set the Proportional Gain constant of the PIDF controller on the SPARK
   * MAX. This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The proportional gain value, must be positive
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setP(double gain, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).pGain.set(gain);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetP(m_device.m_sparkMax, slotID, (float)gain));
    }
  }

  /**
   * Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The integral gain value, must be positive
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setI(double gain) {
    return setI(gain, 0);
  }

  /**
   * Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The integral gain value, must be positive
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setI(double gain, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).iGain.set(gain);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetI(m_device.m_sparkMax, slotID, (float)gain));
    }
  }

  /**
   * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The derivative gain value, must be positive
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setD(double gain) {
    return setD(gain, 0);
  }

    /**
   * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The derivative gain value, must be positive
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setD(double gain, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).dGain.set(gain);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetD(m_device.m_sparkMax, slotID, (float)gain));
    }
  }

  /**
   * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.
   *
   * @param gain The derivative filter value, must be a positive number between 0 and 1
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setDFilter(double gain) {
    return setDFilter(gain,0);
  }

  /**
   * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.
   *
   * @param gain The derivative filter value, must be a positive number between 0 and 1
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setDFilter(double gain, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).dFilter.set(gain);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetDFilter(m_device.m_sparkMax, slotID, (float)gain));
    }
  }

  /**
   * Set the Feed-froward Gain constant of the PIDF controller on the SPARK
   * MAX. This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The feed-forward gain value
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setFF(double gain) {
    return setFF(gain, 0);
  }

  /**
   * Set the Feed-froward Gain constant of the PIDF controller on the SPARK
   * MAX. This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The feed-forward gain value
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setFF(double gain, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).ffGain.set(gain);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetFF(m_device.m_sparkMax, slotID, (float)gain));
    }
  }

  /**
   * Set the IZone range of the PIDF controller on the SPARK MAX. This value
   * specifies the range the |error| must be within for the integral constant
   * to take effect.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   * The recommended method to configure this parameter is to use the
   * SPARK MAX GUI to tune and save parameters.
   *
   * @param IZone The IZone value, must be positive. Set to 0 to disable
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setIZone(double IZone) {
    return setIZone(IZone, 0);
  }

  /**
   * Set the IZone range of the PIDF controller on the SPARK MAX. This value
   * specifies the range the |error| must be within for the integral constant
   * to take effect.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   * The recommended method to configure this parameter is to use the
   * SPARK MAX GUI to tune and save parameters.
   *
   * @param IZone The IZone value, must be positive. Set to 0 to disable
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setIZone(double IZone, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).iZone.set(IZone);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetIZone(m_device.m_sparkMax, slotID, (float)IZone));
    }
  }

  /**
   * Set the min amd max output for the closed loop mode.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   * The recommended method to configure this parameter is to use the
   * SPARK MAX GUI to tune and save parameters.
   *
   * @param min Reverse power minimum to allow the controller to output
   *
   * @param max Forward power maximum to allow the controller to output
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setOutputRange(double min, double max) {
    return setOutputRange(min, max, 0);
  }

  /**
   * Set the min amd max output for the closed loop mode.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   * The recommended method to configure this parameter is to use the
   * SPARK MAX GUI to tune and save parameters.
   *
   * @param min Reverse power minimum to allow the controller to output
   *
   * @param max Forward power maximum to allow the controller to output
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setOutputRange(double min, double max, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).outputMin.set(min);
      m_simSlots.get(slotID).outputMax.set(max);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetOutputRange(m_device.m_sparkMax, slotID, (float)min, (float)max));
    }
  }

  /**
   * Get the Proportional Gain constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double P Gain value
   *
   */
  public double getP() {
    return getP(0);
  }

  /**
   * Get the Proportional Gain constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double P Gain value
   *
   */
  public double getP(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).pGain.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetP(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double I Gain value
   *
   */
  public double getI() {
    return getI(0);
  }

  /**
   * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double I Gain value
   *
   */
  public double getI(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).iGain.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetI(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double D Gain value
   *
   */
  public double getD() {
    return getD(0);
  }

  /**
   * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double D Gain value
   *
   */
  public double getD(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).dGain.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetD(m_device.m_sparkMax, slotID);
    }
  }

    /**
   * Get the Derivative Filter constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double D Filter value
   *
   */
  public double getDFilter(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).dFilter.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetDFilter(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the Feed-forward Gain constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double F Gain value
   *
   */
  public double getFF() {
    return getFF(0);
  }

  /**
   * Get the Feed-forward Gain constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double F Gain value
   *
   */
  public double getFF(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).ffGain.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetFF(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the IZone constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double IZone value
   *
   */
  public double getIZone() {
    return getIZone(0);
  }

  /**
   * Get the IZone constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double IZone value
   *
   */
  public double getIZone(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).iZone.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetIZone(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the derivative filter constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double D Filter
   *
   */
  // public double getDFilter(int slotID = 0);

  /**
   * Get the min output of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double min value
   *
   */
  public double getOutputMin() {
    return getOutputMin(0);
  }

  /**
   * Get the min output of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double min value
   *
   */
  public double getOutputMin(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).outputMin.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetOutputMin(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the max output of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double max value
   *
   */
  public double getOutputMax() {
    return getOutputMax(0);
  }

  /**
   * Get the max output of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double max value
   *
   */
  public double getOutputMax(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).outputMax.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetOutputMax(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Configure the maximum velocity of the SmartMotion mode. This is the
   * velocity that is reached in the middle of the profile and is what
   * the motor should spend most of its time at
   *
   * @param maxVel The maxmimum cruise velocity for the motion profile 
   * in RPM
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionMaxVelocity(double maxVel, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).smartMotionMaxVelocity.set(maxVel);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionMaxVelocity(m_device.m_sparkMax, slotID, (float)maxVel));
    }
  }

  /**
   * Configure the maximum acceleration of the SmartMotion mode. This is
   * the accleration that the motor velocity will increase at until the
   * max velocity is reached
   *
   * @param maxAccel The maxmimum acceleration for the motion profile 
   * in RPM per second
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionMaxAccel(double maxAccel, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).smartMotionMaxAccel.set(maxAccel);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionMaxAccel(m_device.m_sparkMax, slotID, (float)maxAccel));
    }
  }

  /**
   * Configure the mimimum velocity of the SmartMotion mode. Any 
   * requested velocities below this value will be set to 0.
   *
   * @param minVel The minimum velocity for the motion profile in RPM
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionMinOutputVelocity(double minVel, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).smartMotionMinOutputVelocity.set(minVel);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionMinOutputVelocity(m_device.m_sparkMax, slotID, (float)minVel));
    }
  }

  /**
   * Configure the allowed closed loop error of SmartMotion mode.
   * This value is how much deviation from your setpoint is 
   * tolerated and is useful in preventing oscillation around your 
   * setpoint.
   *
   * @param allowedErr The allowed deviation for your setpoint vs 
   * actual position in rotations
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionAllowedClosedLoopError(double allowedErr, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).smartMotionAllowedClosedLoopError.set(allowedErr);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionAllowedClosedLoopError(m_device.m_sparkMax, slotID, (float)allowedErr));
    }
  }

  /**
   * Coming soon. Configure the acceleration strategy used to control 
   * acceleration on the motor. The current strategy is trapezoidal 
   * motion profiling.
   *
   * @param accelStrategy The acceleration strategy to use for the 
   * automatically generated motion profile
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionAccelStrategy(AccelStrategy accelStrategy, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).smartMotionAccelStrategy.set(accelStrategy.value);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionAccelStrategy(m_device.m_sparkMax, slotID, accelStrategy.value));
    }
  }

  /**
   * Get the maximum velocity of the SmartMotion mode. This is the 
   * velocity that is reached in the middle of the profile and is 
   * what the motor should spend most of its time at
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The maxmimum cruise velocity for the motion profile in
   * RPM
   */
  public double getSmartMotionMaxVelocity(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).smartMotionMaxVelocity.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetSmartMotionMaxVelocity(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the maximum acceleration of the SmartMotion mode. This is 
   * the accleration that the motor velocity will increase at until
   * the max velocity is reached
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The maxmimum acceleration for the motion profile in 
   * RPM per second
   */
  public double getSmartMotionMaxAccel(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).smartMotionMaxAccel.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetSmartMotionMaxAccel(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the mimimum velocity of the SmartMotion mode. Any requested
   * velocities below this value will be set to 0.
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The minimum velocity for the motion profile in RPM
   */
  public double getSmartMotionMinOutputVelocity(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).smartMotionMinOutputVelocity.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetSmartMotionMinOutputVelocity(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the allowed closed loop error of SmartMotion mode. This 
   * value is how much deviation from your setpoint is tolerated
   * and is useful in preventing oscillation around your setpoint.
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The allowed deviation for your setpoint vs actual 
   * position in rotations
   * 
   */
  public double getSmartMotionAllowedClosedLoopError(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).smartMotionAllowedClosedLoopError.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetSmartMotionAllowedClosedLoopError(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Get the acceleration strategy used to control acceleration on 
   * the motor.The current strategy is trapezoidal motion profiling.
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The acceleration strategy to use for the automatically 
   * generated motion profile.
   * 
   */
  public AccelStrategy getSmartMotionAccelStrategy(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return AccelStrategy.fromInt(m_simSlots.get(slotID).smartMotionAccelStrategy.get());
    }
    else {
      return AccelStrategy.fromInt(CANSparkMaxJNI.c_SparkMax_GetSmartMotionAccelStrategy(m_device.m_sparkMax, slotID));
    }
  }

  /**
   * Configure the maximum I accumulator of the PID controller.
   * This value is used to constrain the I accumulator to help 
   * manage integral wind-up
   *
   * @param iMaxAccum The max value to contrain the I accumulator to
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setIMaxAccum(double iMaxAccum, int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        return CANError.kParamInvalidID;
      }
      
      m_simSlots.get(slotID).iMaxAccum.set(iMaxAccum);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetIMaxAccum(m_device.m_sparkMax, slotID, (float)iMaxAccum));
    }
  }

  /**
   * Get the maximum I accumulator of the PID controller. 
   * This value is used to constrain the I accumulator to help manage
   * integral wind-up
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The max value to contrain the I accumulator to
   */
  public double getIMaxAccum(int slotID) {
    if (m_simDevice != null) {
      if (!m_simSlots.containsKey(slotID)) {
        throw new RuntimeException("Invalid SlotID");
      }
      
      return m_simSlots.get(slotID).iMaxAccum.get();
    }
    else {
      return (double)CANSparkMaxJNI.c_SparkMax_GetIMaxAccum(m_device.m_sparkMax, slotID);
    }
  }

  /**
   * Set the I accumulator of the PID controller. This is useful
   * when wishing to force a reset on the I accumulator of the 
   * PID controller. You can also preset values to see how it 
   * will respond to certain I characteristics
   *
   * To use this function, the controller must be in a closed
   * loop control mode by calling setReference()
   *
   * @param iAccum The value to set the I accumulator to
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setIAccum(double iAccum) {
    if (m_simDevice != null) {
      m_simIAccum.set(iAccum);
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetIAccum(m_device.m_sparkMax, (float)iAccum));
    }
  }

  /**
   * Get the I accumulator of the PID controller. This is useful
   * when wishing to see what the I accumulator value is to help 
   * with PID tuning
   *
   * @return The value of the I accumulator
   */
  public double getIAccum() {
    if (m_simDevice != null) {
      return m_simIAccum.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetIAccum(m_device.m_sparkMax);
    }
  }

  /**
   * Set the controller's feedback device. 
   * 
   * The default feedback device in brushless mode is assumed to be the
   * integrated encoder and the default feedback device in brushed mode 
   * is assumed to be a quadrature encoder. This is used to changed to 
   * another feedback device for the controller, such as an analog sensor.
   * 
   * If there is a limited range on the feedback sensor that should be 
   * observed by the PIDController, it can be set by calling 
   * SetFeedbackSensorRange() on the sensor object.
   * 
   * @param sensor The sensor to use as a feedback device
   * 
   * @return CANError set to kOK if successful 
   */
  public CANError setFeedbackDevice(final CANSensor sensor) {
    if (m_simDevice != null) {
      return CANError.kOk;
    }
    else {
      return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetFeedbackDevice(m_device.m_sparkMax, sensor.getID()));
    }
  }
}
