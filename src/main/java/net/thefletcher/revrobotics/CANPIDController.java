package net.thefletcher.revrobotics;

import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;

import net.thefletcher.revrobotics.enums.*;

/** @deprecated Use {@link SparkMaxPIDController} instead. */
@Deprecated(forRemoval = true)
public interface CANPIDController {
  /**
   * Set the controller reference value based on the selected control mode.
   *
   * @param value The value to set depending on the control mode. For basic duty cycle control this
   *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
   *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
   *     (Amps). Native units can be changed using the setPositionConversionFactor() or
   *     setVelocityConversionFactor() methods of the CANEncoder class
   * @param ctrl the control type
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setReference(double value, ControlType ctrl);

  /**
   * Set the controller reference value based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic duty cycle control this
   *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
   *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
   *     (Amps). Native units can be changed using the setPositionConversionFactor() or
   *     setVelocityConversionFactor() methods of the CANEncoder class
   * @param ctrl Is the control type to override with
   * @param pidSlot for this command
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setReference(double value, ControlType ctrl, int pidSlot);

  /**
   * Set the controller reference value based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic duty cycle control this
   *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
   *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
   *     (Amps). Native units can be changed using the setPositionConversionFactor() or
   *     setVelocityConversionFactor() methods of the CANEncoder class
   * @param ctrl Is the control type to override with
   * @param pidSlot for this command
   * @param arbFeedforward A value from which is represented in voltage applied to the motor after
   *     the result of the specified control mode. The units for the parameter is Volts. This value
   *     is set after the control mode, but before any current limits or ramp rates.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setReference(
      double value, ControlType ctrl, int pidSlot, double arbFeedforward);

  /**
   * Set the controller reference value based on the selected control mode. This will override the
   * pre-programmed control mode but not change what is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic duty cycle control this
   *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
   *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
   *     (Amps). Native units can be changed using the setPositionConversionFactor() or
   *     setVelocityConversionFactor() methods of the CANEncoder class
   * @param ctrl Is the control type to override with
   * @param pidSlot for this command
   * @param arbFeedforward A value from which is represented in voltage applied to the motor after
   *     the result of the specified control mode. The units for the parameter is Volts. This value
   *     is set after the control mode, but before any current limits or ramp rates.
   * @param arbFFUnits The units the arbitrary feed forward term is in
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setReference(
      double value,
      ControlType ctrl,
      int pidSlot,
      double arbFeedforward,
      ArbFFUnits arbFFUnits);

  /**
   * Set the Proportional Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
   * GUI to tune and save parameters.
   *
   * @param gain The proportional gain value, must be positive
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setP(double gain);

  /**
   * Set the Proportional Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
   * GUI to tune and save parameters.
   *
   * @param gain The proportional gain value, must be positive
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setP(double gain, int slotID);

  /**
   * Set the Integral Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
   * GUI to tune and save parameters.
   *
   * @param gain The integral gain value, must be positive
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setI(double gain);

  /**
   * Set the Integral Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
   * GUI to tune and save parameters.
   *
   * @param gain The integral gain value, must be positive
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setI(double gain, int slotID);

  /**
   * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
   * GUI to tune and save parameters.
   *
   * @param gain The derivative gain value, must be positive
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setD(double gain);

  /**
   * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
   * GUI to tune and save parameters.
   *
   * @param gain The derivative gain value, must be positive
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setD(double gain, int slotID);

  /**
   * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called.
   *
   * @param gain The derivative filter value, must be a positive number between 0 and 1
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setDFilter(double gain);

  /**
   * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called.
   *
   * @param gain The derivative filter value, must be a positive number between 0 and 1
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setDFilter(double gain, int slotID);

  /**
   * Set the Feed-froward Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
   * GUI to tune and save parameters.
   *
   * @param gain The feed-forward gain value
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setFF(double gain);

  /**
   * Set the Feed-froward Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
   * Parameter API and should be used infrequently. The parameter does not presist unless
   * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
   * GUI to tune and save parameters.
   *
   * @param gain The feed-forward gain value
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setFF(double gain, int slotID);

  /**
   * Set the IZone range of the PIDF controller on the SPARK MAX. This value specifies the range the
   * |error| must be within for the integral constant to take effect.
   *
   * <p>This uses the Set Parameter API and should be used infrequently. The parameter does not
   * presist unless burnFlash() is called. The recommended method to configure this parameter is to
   * use the SPARK MAX GUI to tune and save parameters.
   *
   * @param IZone The IZone value, must be positive. Set to 0 to disable
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIZone(double IZone);

  /**
   * Set the IZone range of the PIDF controller on the SPARK MAX. This value specifies the range the
   * |error| must be within for the integral constant to take effect.
   *
   * <p>This uses the Set Parameter API and should be used infrequently. The parameter does not
   * presist unless burnFlash() is called. The recommended method to configure this parameter is to
   * use the SPARK MAX GUI to tune and save parameters.
   *
   * @param IZone The IZone value, must be positive. Set to 0 to disable
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIZone(double IZone, int slotID);

  /**
   * Set the min amd max output for the closed loop mode.
   *
   * <p>This uses the Set Parameter API and should be used infrequently. The parameter does not
   * presist unless burnFlash() is called. The recommended method to configure this parameter is to
   * use the SPARK MAX GUI to tune and save parameters.
   *
   * @param min Reverse power minimum to allow the controller to output
   * @param max Forward power maximum to allow the controller to output
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setOutputRange(double min, double max);

  /**
   * Set the min amd max output for the closed loop mode.
   *
   * <p>This uses the Set Parameter API and should be used infrequently. The parameter does not
   * presist unless burnFlash() is called. The recommended method to configure this parameter is to
   * use the SPARK MAX GUI to tune and save parameters.
   *
   * @param min Reverse power minimum to allow the controller to output
   * @param max Forward power maximum to allow the controller to output
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setOutputRange(double min, double max, int slotID);

  /**
   * Get the Proportional Gain constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @return double P Gain value
   */
  public double getP();

  /**
   * Get the Proportional Gain constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double P Gain value
   */
  public double getP(int slotID);

  /**
   * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @return double I Gain value
   */
  public double getI();

  /**
   * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double I Gain value
   */
  public double getI(int slotID);

  /**
   * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @return double D Gain value
   */
  public double getD();

  /**
   * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double D Gain value
   */
  public double getD(int slotID);

  /**
   * Get the Derivative Filter constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double D Filter value
   */
  public double getDFilter(int slotID);

  /**
   * Get the Feed-forward Gain constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @return double F Gain value
   */
  public double getFF();

  /**
   * Get the Feed-forward Gain constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double F Gain value
   */
  public double getFF(int slotID);

  /**
   * Get the IZone constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @return double IZone value
   */
  public double getIZone();

  /**
   * Get the IZone constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double IZone value
   */
  public double getIZone(int slotID);

  /**
   * Get the derivative filter constant of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double D Filter
   */
  // public double getDFilter(int slotID = 0);

  /**
   * Get the min output of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @return double min value
   */
  public double getOutputMin();

  /**
   * Get the min output of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double min value
   */
  public double getOutputMin(int slotID);

  /**
   * Get the max output of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @return double max value
   */
  public double getOutputMax();

  /**
   * Get the max output of the PIDF controller on the SPARK MAX.
   *
   * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
   * non-blocking call and will return a cached value if the parameter is not returned by the
   * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return double max value
   */
  public double getOutputMax(int slotID);

  /**
   * Configure the maximum velocity of the SmartMotion mode. This is the velocity that is reached in
   * the middle of the profile and is what the motor should spend most of its time at
   *
   * @param maxVel The maxmimum cruise velocity for the motion profile in RPM
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSmartMotionMaxVelocity(double maxVel, int slotID);

  /**
   * Configure the maximum acceleration of the SmartMotion mode. This is the accleration that the
   * motor velocity will increase at until the max velocity is reached
   *
   * @param maxAccel The maxmimum acceleration for the motion profile in RPM per second
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSmartMotionMaxAccel(double maxAccel, int slotID);

  /**
   * Configure the mimimum velocity of the SmartMotion mode. Any requested velocities below this
   * value will be set to 0.
   *
   * @param minVel The minimum velocity for the motion profile in RPM
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSmartMotionMinOutputVelocity(double minVel, int slotID);

  /**
   * Configure the allowed closed loop error of SmartMotion mode. This value is how much deviation
   * from your setpoint is tolerated and is useful in preventing oscillation around your setpoint.
   *
   * @param allowedErr The allowed deviation for your setpoint vs actual position in rotations
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSmartMotionAllowedClosedLoopError(double allowedErr, int slotID);

  /**
   * NOTE: As of the 2022 FRC season, the firmware only supports the trapezoidal motion profiling
   * acceleration strategy.
   *
   * <p>Configure the acceleration strategy used to control acceleration on the motor.
   *
   * @param accelStrategy The acceleration strategy to use for the automatically generated motion
   *     profile
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setSmartMotionAccelStrategy(AccelStrategy accelStrategy, int slotID);

  /**
   * Get the maximum velocity of the SmartMotion mode. This is the velocity that is reached in the
   * middle of the profile and is what the motor should spend most of its time at
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return The maxmimum cruise velocity for the motion profile in RPM
   */
  public double getSmartMotionMaxVelocity(int slotID);

  /**
   * Get the maximum acceleration of the SmartMotion mode. This is the accleration that the motor
   * velocity will increase at until the max velocity is reached
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return The maxmimum acceleration for the motion profile in RPM per second
   */
  public double getSmartMotionMaxAccel(int slotID);

  /**
   * Get the mimimum velocity of the SmartMotion mode. Any requested velocities below this value
   * will be set to 0.
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return The minimum velocity for the motion profile in RPM
   */
  public double getSmartMotionMinOutputVelocity(int slotID);

  /**
   * Get the allowed closed loop error of SmartMotion mode. This value is how much deviation from
   * your setpoint is tolerated and is useful in preventing oscillation around your setpoint.
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return The allowed deviation for your setpoint vs actual position in rotations
   */
  public double getSmartMotionAllowedClosedLoopError(int slotID);

  /**
   * Get the acceleration strategy used to control acceleration on the motor. As of the 2022 FRC
   * season, the strategy is always trapezoidal motion profiling, regardless of what the device may
   * report.
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return The acceleration strategy to use for the automatically generated motion profile.
   */
  public AccelStrategy getSmartMotionAccelStrategy(int slotID);

  /**
   * Configure the maximum I accumulator of the PID controller. This value is used to constrain the
   * I accumulator to help manage integral wind-up
   *
   * @param iMaxAccum The max value to contrain the I accumulator to
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIMaxAccum(double iMaxAccum, int slotID);

  /**
   * Get the maximum I accumulator of the PID controller. This value is used to constrain the I
   * accumulator to help manage integral wind-up
   *
   * @param slotID Is the gain schedule slot, the value is a number between 0 and 3. Each slot has
   *     its own set of gain values and can be changed in each control frame using SetReference().
   * @return The max value to contrain the I accumulator to
   */
  public double getIMaxAccum(int slotID);

  /**
   * Set the I accumulator of the PID controller. This is useful when wishing to force a reset on
   * the I accumulator of the PID controller. You can also preset values to see how it will respond
   * to certain I characteristics
   *
   * <p>To use this function, the controller must be in a closed loop control mode by calling
   * setReference()
   *
   * @param iAccum The value to set the I accumulator to
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setIAccum(double iAccum);

  /**
   * Get the I accumulator of the PID controller. This is useful when wishing to see what the I
   * accumulator value is to help with PID tuning
   *
   * @return The value of the I accumulator
   */
  public double getIAccum();

  /**
   * Set the controller's feedback device
   *
   * <p>The default feedback device in brushless mode is assumed to be the integrated encoder and
   * the default feedback device in brushed mode is assumed to be a quadrature encoder. This is used
   * to changed to another feedback device for the controller, such as an analog sensor.
   *
   * <p>If there is a limited range on the feedback sensor that should be observed by the
   * PIDController, it can be set by calling SetFeedbackSensorRange() on the sensor object.
   *
   * @param sensor The sensor to use as a feedback device
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setFeedbackDevice(final MotorFeedbackSensor sensor);
}
