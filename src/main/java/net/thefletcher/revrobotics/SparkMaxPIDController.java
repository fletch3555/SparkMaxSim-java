package net.thefletcher.revrobotics;


import com.revrobotics.CANSensor;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.hal.*;
import edu.wpi.first.hal.SimDevice.Direction;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import net.thefletcher.revrobotics.enums.*;

import static java.util.Optional.ofNullable;

public class SparkMaxPIDController implements CANPIDController {
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
        pGain = simDevice.createDouble("[" + slot + "] P Gain", Direction.kBidir, 0.0);
        iGain = simDevice.createDouble("[" + slot + "] I Gain", Direction.kBidir, 0.0);
        iZone = simDevice.createDouble("[" + slot + "] I Zone", Direction.kBidir, 0.0);
        dGain = simDevice.createDouble("[" + slot + "] D Gain", Direction.kBidir, 0.0);
        dFilter = simDevice.createDouble("[" + slot + "] D Filter", Direction.kBidir, 0.0);
        ffGain = simDevice.createDouble("[" + slot + "] FF Gain", Direction.kBidir, 0.0);
        outputMin = simDevice.createDouble("[" + slot + "] Min Output", Direction.kBidir, -1.0);
        outputMax = simDevice.createDouble("[" + slot + "] Max Output", Direction.kBidir, 1.0);
        smartMotionMaxVelocity = simDevice.createDouble("[" + slot + "] SmartMotion Max Velocity", Direction.kBidir, 0.0);
        smartMotionMaxAccel = simDevice.createDouble("[" + slot + "] SmartMotion Max Acceleration", Direction.kBidir, 0.0);
        smartMotionMinOutputVelocity = simDevice.createDouble("[" + slot + "] SmartMotion Min Output Velocity", Direction.kBidir, 0.0);
        smartMotionAllowedClosedLoopError = simDevice.createDouble("[" + slot + "] SmartMotion Allowed Closed-Loop Error", Direction.kBidir, 0.0);
        smartMotionAccelStrategy = simDevice.createEnum("[" + slot + "] SmartMotion Accel Strategy", Direction.kBidir, Arrays.stream(AccelStrategy.values()).map(AccelStrategy::toString).toArray(String[]::new), AccelStrategy.kSCurve.value);
        iMaxAccum = simDevice.createDouble("[" + slot + "] I Accumulator Max", Direction.kBidir, 0.0);
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
  public SparkMaxPIDController(CANSparkMax device) {
    m_device = device;

    m_simDevice = SimDevice.create("CANSparkMax[" + device.getDeviceId() + "] - PIDController");
    if (m_simDevice != null) {
      m_simIAccum = m_simDevice.createDouble("I Accumulator", Direction.kOutput, 0.0);
    }
  }

  private SimPIDSlot getPIDSlot(int slotId) {
    if (slotId < 0 || slotId >= 4) {
      return null;
    }

    return m_simSlots.computeIfAbsent(slotId, k -> new SimPIDSlot(slotId, m_simDevice));
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setReference(double value, ControlType ctrl) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setReference(double value, ControlType ctrl, int pidSlot) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setReference(double value, ControlType ctrl, int pidSlot, double arbFeedforward) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setReference(double value, ControlType ctrl, int pidSlot, double arbFeedforward, ArbFFUnits arbFFUnits) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setP(double gain) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setP(double gain, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.pGain.set(gain);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetP(m_device.m_sparkMax, slotID, (float)gain));
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setI(double gain) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setI(double gain, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.iGain.set(gain);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetI(m_device.m_sparkMax, slotID, (float)gain));
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setD(double gain) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setD(double gain, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.dGain.set(gain);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetD(m_device.m_sparkMax, slotID, (float)gain));
    }
  }

  /**
   * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.
   *
   * @param gain The derivative filter value, must be a positive number between 0 and 1
   *
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setDFilter(double gain) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setDFilter(double gain, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.dFilter.set(gain);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetDFilter(m_device.m_sparkMax, slotID, (float)gain));
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setFF(double gain) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setFF(double gain, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.ffGain.set(gain);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetFF(m_device.m_sparkMax, slotID, (float)gain));
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setIZone(double IZone) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setIZone(double IZone, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.iZone.set(IZone);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetIZone(m_device.m_sparkMax, slotID, (float)IZone));
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setOutputRange(double min, double max) {
    m_device.throwIfClosed();
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
   * @return REVLibError Set to REV_OK if successful
   *
   */
  public REVLibError setOutputRange(double min, double max, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.outputMin.set(min);
          slot.outputMax.set(max);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetOutputRange(m_device.m_sparkMax, slotID, (float)min, (float)max));
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
    m_device.throwIfClosed();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .pGain.get();
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
    m_device.throwIfClosed();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .iGain.get();
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
    m_device.throwIfClosed();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .dGain.get();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .dFilter.get();
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
    m_device.throwIfClosed();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .ffGain.get();
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
    m_device.throwIfClosed();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .iZone.get();
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
    m_device.throwIfClosed();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .outputMin.get();
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
    m_device.throwIfClosed();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .outputMax.get();
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
   * @return REVLibError Set to kOK if successful
   */
  public REVLibError setSmartMotionMaxVelocity(double maxVel, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.smartMotionMaxVelocity.set(maxVel);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionMaxVelocity(m_device.m_sparkMax, slotID, (float)maxVel));
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
   * @return REVLibError Set to kOK if successful
   */
  public REVLibError setSmartMotionMaxAccel(double maxAccel, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.smartMotionMaxAccel.set(maxAccel);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionMaxAccel(m_device.m_sparkMax, slotID, (float)maxAccel));
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
   * @return REVLibError Set to kOK if successful
   */
  public REVLibError setSmartMotionMinOutputVelocity(double minVel, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.smartMotionMinOutputVelocity.set(minVel);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionMinOutputVelocity(m_device.m_sparkMax, slotID, (float)minVel));
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
   * @return REVLibError Set to kOK if successful
   */
  public REVLibError setSmartMotionAllowedClosedLoopError(double allowedErr, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.smartMotionAllowedClosedLoopError.set(allowedErr);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionAllowedClosedLoopError(m_device.m_sparkMax, slotID, (float)allowedErr));
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
   * @return REVLibError Set to kOK if successful
   */
  public REVLibError setSmartMotionAccelStrategy(AccelStrategy accelStrategy, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.smartMotionAccelStrategy.set(accelStrategy.value);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartMotionAccelStrategy(m_device.m_sparkMax, slotID, accelStrategy.value));
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .smartMotionMaxVelocity.get();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .smartMotionMaxAccel.get();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .smartMotionMinOutputVelocity.get();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
        .smartMotionAllowedClosedLoopError.get();
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return AccelStrategy.fromInt(
        ofNullable(getPIDSlot(slotID))
          .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
          .smartMotionAccelStrategy.get()
      );
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
   * @return REVLibError Set to kOK if successful
   */
  public REVLibError setIMaxAccum(double iMaxAccum, int slotID) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
        .map(slot -> {
          slot.iMaxAccum.set(iMaxAccum);
          return REVLibError.kOk;
        })
        .orElse(REVLibError.kParamInvalidID);
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetIMaxAccum(m_device.m_sparkMax, slotID, (float)iMaxAccum));
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return ofNullable(getPIDSlot(slotID))
      .orElseThrow(() -> new RuntimeException("Invalid SlotID"))
      .iMaxAccum.get();
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
   * @return REVLibError Set to kOK if successful
   */
  public REVLibError setIAccum(double iAccum) {
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      m_simIAccum.set(iAccum);
      return REVLibError.kOk;
    }
    else {
      return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetIAccum(m_device.m_sparkMax, (float)iAccum));
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
    m_device.throwIfClosed();
    if (m_simDevice != null) {
      return m_simIAccum.get();
    }
    else {
      return CANSparkMaxJNI.c_SparkMax_GetIAccum(m_device.m_sparkMax);
    }
  }


  @Override
  public REVLibError setFeedbackDevice(MotorFeedbackSensor sensor) {
    return setFeedbackDevice((CANSensor) sensor);
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
   * @return REVLibError set to kOK if successful 
   */
  public REVLibError setFeedbackDevice(final CANSensor sensor) {
    m_device.throwIfClosed();

    if (sensor instanceof SparkMaxRelativeEncoder
        || sensor instanceof SparkMaxAlternateEncoder
        || sensor instanceof SparkMaxAnalogSensor) {
      if (m_simDevice != null) {
        return REVLibError.kOk;
      }
      else {
        // Handle sensors that are directly connected to the SPARK Max
        CANSparkMax sparkMaxSensorIsConnectedTo;
        int feedbackDeviceId;

        if (sensor instanceof SparkMaxRelativeEncoder) {
          sparkMaxSensorIsConnectedTo = ((SparkMaxRelativeEncoder) sensor).sparkMax;
          feedbackDeviceId = ((SparkMaxRelativeEncoder) sensor).getSparkMaxFeedbackDeviceId();
        } else if (sensor instanceof SparkMaxAlternateEncoder) {
          sparkMaxSensorIsConnectedTo = ((SparkMaxAlternateEncoder) sensor).sparkMax;
          feedbackDeviceId = ((SparkMaxAlternateEncoder) sensor).getSparkMaxFeedbackDeviceId();
        } else {
          sparkMaxSensorIsConnectedTo = ((SparkMaxAnalogSensor) sensor).sparkMax;
          feedbackDeviceId = ((SparkMaxAnalogSensor) sensor).getSparkMaxFeedbackDeviceId();
        }

        if (sparkMaxSensorIsConnectedTo != this.m_device) {
          throw new IllegalArgumentException(
              "A sensor attached to one SPARK MAX cannot be used as a feedback device for a different SPARK MAX");
        }

        return REVLibError.fromInt(
            CANSparkMaxJNI.c_SparkMax_SetFeedbackDevice(m_device.m_sparkMax, feedbackDeviceId));
      }
    } else {
      // Right now, the SPARK MAX does not support sensors that are not directly connected to itself
      throw new IllegalArgumentException(
          sensor.getClass().getSimpleName()
              + " cannot be used as a feedback device for a SPARK MAX at this time");
    }
  }
}
