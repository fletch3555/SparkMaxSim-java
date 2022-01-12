package net.thefletcher.revrobotics;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.hal.*;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import net.thefletcher.revrobotics.enums.*;

public class CANSparkMax extends CANSparkMaxLowLevel implements Sendable {

	private SparkMaxRelativeEncoder encoder;
	private final Object encoderLock = new Object();
  
	private SparkMaxAlternateEncoder altEncoder;
	private final Object altEncoderLock = new Object();
  
	private SparkMaxAnalogSensor analogSensor;
	private final Object analogSensorLock = new Object();
  
	private SparkMaxPIDController pidController;
	private final Object pidControllerLock = new Object();
  
	private SparkMaxLimitSwitch forwardLimitSwitch;
	private final Object forwardLimitSwitchLock = new Object();
  
	private SparkMaxLimitSwitch reverseLimitSwitch;
	private final Object reverseLimitSwitchLock = new Object();

	protected SimBoolean m_simInverted;
	protected SimEnum m_simIdleMode;
	protected SimBoolean m_simVoltageCompentsationEnabled;
	protected SimDouble m_simOpenLoopRampRate;
	protected SimDouble m_simClosedLoopRampRate;
    protected SimDouble m_simSoftLimitFwd;
    protected SimBoolean m_simSoftLimitFwdEnabled;
    protected SimDouble m_simSoftLimitRev;
    protected SimBoolean m_simSoftLimitRevEnabled;

	boolean m_altEncInitialized = false;
	boolean m_limitSwitchInitialized = false;

	// Only used for get() and set() API
	private double m_setpoint = 0.0;
	
	/**
	 * Create a new SPARK MAX Controller
	 *
	 * @param deviceID The device ID.
	 * @param type     The motor type connected to the controller. Brushless motors
	 *                 must be connected to their matching color and the hall sensor
	 *                 plugged in. Brushed motors must be connected to the Red and
	 *                 Black terminals only.
	 */
	public CANSparkMax(int deviceID, MotorType type) {
		super(deviceID, type);

        if (m_simDevice != null) {
            m_simInverted = m_simDevice.createBoolean("Inverted", Direction.kOutput, false);
            m_simIdleMode = m_simDevice.createEnum("Idle Mode", Direction.kOutput, new String[] {"Coast", "Brake"}, 0);
			m_simVoltageCompentsationEnabled = m_simDevice.createBoolean("Voltage Compensation Enabled", Direction.kOutput, false);
			m_simOpenLoopRampRate = m_simDevice.createDouble("Open Loop Ramp Rate", Direction.kOutput, 1.0);
			m_simClosedLoopRampRate = m_simDevice.createDouble("Closed Loop Ramp Rate", Direction.kOutput, 1.0);
			m_simSoftLimitFwd = m_simDevice.createDouble("Fwd Soft Limit", Direction.kOutput, 0.0);
            m_simSoftLimitFwdEnabled = m_simDevice.createBoolean("Fwd Soft Limit Enabled", Direction.kOutput, false);
            m_simSoftLimitRev = m_simDevice.createDouble("Rev Soft Limit", Direction.kOutput, 0.0);
            m_simSoftLimitRevEnabled = m_simDevice.createBoolean("Rev Soft Limit Enabled", Direction.kOutput, false);
        }
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType(BuiltInWidgets.kMotorController.getWidgetName());
		builder.setActuator(true);
		builder.setSafeState(this::stopMotor);
		builder.addDoubleProperty("Value", this::get, this::set);
	}

	/**
	 * Closes the SPARK MAX Controller
	 */
	@Override
	public void close() {
		super.close();
		SendableRegistry.remove(this);
	}

	/**** Speed Controller Interface ****/
	/**
	 * Common interface for setting the speed of a speed controller.
	 *
	 * @param speed The speed to set. Value should be between -1.0 and 1.0.
	 */
	@Override
	public void set(double speed) {
		throwIfClosed();
		// Only for 'get' API
		m_setpoint = speed;
		setpointCommand(speed, ControlType.kDutyCycle);
	}

    /**
     * Sets the voltage output of the SpeedController.  This is equivillant to 
     * a call to SetReference(output, rev::ControlType::kVoltage). The behavior
     * of this call differs slightly from the WPILib documetation for this call
     * since the device internally sets the desired voltage (not a compensation value).
     * That means that this *can* be a 'set-and-forget' call.
     *
     * @param outputVolts The voltage to output.
     */
	@Override
	public void setVoltage(double outputVolts) {
		throwIfClosed();
		setpointCommand(outputVolts, ControlType.kVoltage);
	}

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed. Value is between -1.0 and 1.0.
	 */
	@Override
	public double get() {
		throwIfClosed();
		return m_setpoint;
	}

	/**
	 * Common interface for inverting direction of a speed controller.
	 *
	 * This call has no effect if the controller is a follower.
	 *
	 * @param isInverted The state of inversion, true is inverted.
	 */
	@Override
	public void setInverted(boolean isInverted) {
		throwIfClosed();
		if (m_simDevice != null) {
            m_simInverted.set(isInverted);
        }
    	else {
			CANSparkMaxJNI.c_SparkMax_SetInverted(m_sparkMax, isInverted);
		}
	}

	/**
	 * Common interface for returning the inversion state of a speed controller.
	 * 
     * This call has no effect if the controller is a follower.
	 *
	 * @return isInverted The state of inversion, true is inverted.
	 */
	@Override
	public boolean getInverted() {
		throwIfClosed();
        if (m_simDevice != null) {
            return m_simInverted.get();
        }
    	else {
			return CANSparkMaxJNI.c_SparkMax_GetInverted(m_sparkMax);
        }
	}
    

	/**
	 * Common interface for disabling a motor.
	 */
	@Override
	public void disable() {
		throwIfClosed();
		set(0);
	}

	@Override
	public void stopMotor() {
		throwIfClosed();
		set(0);
	}

	/******* Extended Functions *******/
	/**
	 * Returns and object for interfacing with the encoder connected to the 
	 * encoder pins or front port of the SPARK MAX.
	 * 
	 * The default encoder type is assumed to be the hall effect for brushless.
	 * This can be modified for brushed DC to use a quadrature encoder.
	 * 
	 * Assumes that the encoder the is integrated encoder, configured as:
	 * EncoderType.kHallEffect, 0 counts per revolution.
	 * 
	 * @return An object for interfacing with the integrated encoder.
	 */
	public RelativeEncoder getEncoder() {
		throwIfClosed();
		return getEncoder(SparkMaxRelativeEncoderType.kHallSensor, 0);
	}

	/**
	 * Returns and object for interfacing with the encoder connected to the 
	 * encoder pins or front port of the SPARK MAX.
	 * 
	 * The default encoder type is assumed to be the hall effect for brushless.
	 * This can be modified for brushed DC to use a quadrature encoder.
	 * 
	 * @param sensorType The encoder type for the motor: kHallEffect or kQuadrature
	 * @param counts_per_rev The counts per revolution of the encoder
	 * @return An object for interfacing with an encoder
	 */
	public RelativeEncoder getEncoder(SparkMaxRelativeEncoderType encoderType, int countsPerRev) {
		throwIfClosed();
		synchronized (encoderLock) {
		  if (encoder == null) {
			encoder = new SparkMaxRelativeEncoder(this, encoderType, countsPerRev);
		  } else {
			if (encoder.type != encoderType) {
			  throw new IllegalStateException(
				  "The main encoder on this SPARK MAX has already been initialized as a "
					  + encoder.type);
			}
			// Counts per revolution is ignored for the hall sensor encoder type
			if (encoder.type != SparkMaxRelativeEncoderType.kHallSensor
				&& encoder.getCountsPerRevolution() != countsPerRev) {
			  throw new IllegalStateException(
				  "The main encoder on this SPARK MAX has already been initialized with countsPerRev set to "
					  + encoder.countsPerRev);
			}
			// The user isn't trying to change their encoder settings mid-program, so we're all set
		  }
		  return encoder;
		}
	}

	/**
	 * Returns an object for interfacing with an encoder connected to the alternate 
	 * data port configured pins. This is defined as :
	 * 
	 * Mutli-function Pin: Encoder A
	 * Limit Switch Reverse: Encoder B
	 * 
	 * This call will disable the limit switch inputs
	 * @return Returns an object for interfacing with an encoder connected to the alternate 
	 * data port configured pins
	 */
	public RelativeEncoder getAlternateEncoder(int countsPerRev) {
		throwIfClosed();
		return getAlternateEncoder(SparkMaxAlternateEncoderType.kQuadrature, countsPerRev);
	}

	/**
	 * Returns an object for interfacing with an encoder connected to the alternate 
	 * data port configured pins. This is defined as :
	 * 
	 * Mutli-function Pin: Encoder A
	 * Limit Switch Reverse: Encoder B
	 * 
	 * This call will disable the limit switch inputs.
	 * 
	 * @param counts_per_rev the Counts per revolution of the encoder
	 * @param sensorType The encoder type for the motor: currently only kQuadrature
	 * @return Returns an object for interfacing with an encoder connected to the alternate 
	 * data port configured pins
	 */
	public RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoderType encoderType, int countsPerRev) {
		throwIfClosed();
		synchronized (altEncoderLock) {
		  if (altEncoder == null) {
			altEncoder = new SparkMaxAlternateEncoder(this, encoderType, countsPerRev);
		  } else {
			if (altEncoder.type != encoderType) {
			  throw new IllegalStateException(
				  "The alternate encoder on this SPARK MAX has already been initialized as a "
					  + altEncoder.type);
			}
			if (altEncoder.countsPerRev != countsPerRev) {
			  throw new IllegalStateException(
				  "The alternate encoder on this SPARK MAX has already been initialized with countsPerRev set to "
					  + altEncoder.countsPerRev);
			}
			// The user isn't trying to change their alternate encoder settings mid-program,
			// so we're all set
		  }
		  return altEncoder;
		}
	}

	/**
	 * @param mode The mode of the analog sensor, either absolute or relative
	 * @return An object for interfacing with a connected analog sensor.
	 */
	public SparkMaxAnalogSensor getAnalog(SparkMaxAnalogSensorMode mode) {
		throwIfClosed();
		synchronized (analogSensorLock) {
			if (analogSensor == null) {
				analogSensor = new SparkMaxAnalogSensor(this, mode);
			} else {
				if (analogSensor.mode != mode) {
					throw new IllegalStateException(
						"The analog sensor connected to this SPARK MAX has already been configured with mode "
							+ analogSensor.mode);
				}
				// The user isn't trying to change their analog sensor settings mid-program,
				// so we're all set
			}
			return analogSensor;
		}
	}

  /** @return An object for interfacing with the integrated PID controller. */
  public SparkMaxPIDController getPIDController() {
    throwIfClosed();
    synchronized (pidControllerLock) {
      if (pidController == null) {
        pidController = new SparkMaxPIDController(this);
      }
      return pidController;
    }
  }

  /**
   * Returns an object for interfacing with the forward limit switch connected to the appropriate
   * pins on the data port.
   *
   * <p>This call will disable support for the alternate encoder.
   *
   * @param switchType Whether the limit switch is normally open or normally closed.
   * @return An object for interfacing with the forward limit switch.
   */
  public SparkMaxLimitSwitch getForwardLimitSwitch(SparkMaxLimitSwitchType switchType) {
    throwIfClosed();
    synchronized (forwardLimitSwitchLock) {
      if (forwardLimitSwitch == null) {
        forwardLimitSwitch =
            new SparkMaxLimitSwitch(this, SparkMaxLimitSwitchDirection.kForward, switchType);
      } else {
        if (forwardLimitSwitch.m_switchType != switchType) {
          throw new IllegalStateException(
              "The forward limit switch on this SPARK MAX has already been configured with the"
                  + forwardLimitSwitch.m_switchType
                  + " switch type");
        }
      }
      return forwardLimitSwitch;
    }
  }

  /**
   * Returns an object for interfacing with the reverse limit switch connected to the appropriate
   * pins on the data port.
   *
   * <p>This call will disable support for the alternate encoder.
   *
   * @param switchType Whether the limit switch is normally open or normally closed.
   * @return An object for interfacing with the reverse limit switch.
   */
  public SparkMaxLimitSwitch getReverseLimitSwitch(SparkMaxLimitSwitchType switchType) {
    throwIfClosed();
    synchronized (reverseLimitSwitchLock) {
      if (reverseLimitSwitch == null) {
        reverseLimitSwitch =
            new SparkMaxLimitSwitch(this, SparkMaxLimitSwitchDirection.kReverse, switchType);
      } else {
        if (reverseLimitSwitch.m_switchType != switchType) {
          throw new IllegalStateException(
              "The reverse limit switch on this SPARK MAX has already been configured with the "
                  + reverseLimitSwitch.m_switchType
                  + " switch type");
        }
      }
      return reverseLimitSwitch;
    }
  }

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * @param limit The current limit in Amps.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 *
	 */
	public REVLibError setSmartCurrentLimit(int limit) {
		return setSmartCurrentLimit(limit, 0);
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * The controller can also limit the current based on the RPM of the motor in a
	 * linear fashion to help with controllability in closed loop control. For a
	 * response that is linear the entire RPM range leave limit RPM at 0.
	 *
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param freeLimit  The current limit at free speed (5700RPM for NEO).
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit) {
		return setSmartCurrentLimit(stallLimit, freeLimit, 20000);
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * The controller can also limit the current based on the RPM of the motor in a
	 * linear fashion to help with controllability in closed loop control. For a
	 * response that is linear the entire RPM range leave limit RPM at 0.
	 *
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param freeLimit  The current limit at free speed (5700RPM for NEO).
	 * @param limitRPM   RPM less than this value will be set to the stallLimit, RPM
	 *                   values greater than limitRPM will scale linearly to
	 *                   freeLimit
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
		if (m_simDevice != null) {
			// TODO: fix no-op?
            return REVLibError.kOk;
        }
        else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartCurrentLimit(m_sparkMax, stallLimit, freeLimit, limitRPM));
		}
	}

	/**
	 * Sets the secondary current limit in Amps.
	 *
	 * The motor controller will disable the output of the controller briefly if the
	 * current limit is exceeded to reduce the current. This limit is a simplified
	 * 'on/off' controller. This limit is enabled by default but is set higher than
	 * the default Smart Current Limit.
	 *
	 * The time the controller is off after the current limit is reached is
	 * determined by the parameter limitCycles, which is the number of PWM cycles
	 * (20kHz). The recommended value is the default of 0 which is the minimum time
	 * and is part of a PWM cycle from when the over current is detected. This
	 * allows the controller to regulate the current close to the limit value.
	 *
	 * The total time is set by the equation
	 *
	 * <code>
	 * t = (50us - t0) + 50us * limitCycles
	 * t = total off time after over current
	 * t0 = time from the start of the PWM cycle until over current is detected
	 * </code>
	 *
	 *
	 * @param limit The current limit in Amps.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError setSecondaryCurrentLimit(double limit) {
		return setSecondaryCurrentLimit(limit, 0);
	}

	/**
	 * Sets the secondary current limit in Amps.
	 *
	 * The motor controller will disable the output of the controller briefly if the
	 * current limit is exceeded to reduce the current. This limit is a simplified
	 * 'on/off' controller. This limit is enabled by default but is set higher than
	 * the default Smart Current Limit.
	 *
	 * The time the controller is off after the current limit is reached is
	 * determined by the parameter limitCycles, which is the number of PWM cycles
	 * (20kHz). The recommended value is the default of 0 which is the minimum time
	 * and is part of a PWM cycle from when the over current is detected. This
	 * allows the controller to regulate the current close to the limit value.
	 *
	 * The total time is set by the equation
	 *
	 * <code>
	 * t = (50us - t0) + 50us * limitCycles
	 * t = total off time after over current
	 * t0 = time from the start of the PWM cycle until over current is detected
	 * </code>
	 *
	 *
	 * @param limit      The current limit in Amps.
	 * @param chopCycles The number of additional PWM cycles to turn the driver off
	 *                   after overcurrent is detected.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles) {
		if (m_simDevice != null) {
			// TODO: fix no-op?
            return REVLibError.kOk;
        }
        else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSecondaryCurrentLimit(m_sparkMax, (float)limit, chopCycles));
		}
	}

	/**
	 * Sets the idle mode setting for the SPARK MAX.
	 *
	 * @param mode Idle mode (coast or brake).
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError setIdleMode(IdleMode mode) {	
        if (m_simDevice != null) {
            m_simIdleMode.set(mode.value);
            return REVLibError.kOk;
        }
        else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetIdleMode(m_sparkMax, mode.value));
		}
	}

	/**
	 * Gets the idle mode setting for the SPARK MAX.
	 *
	 * This uses the Get Parameter API and should be used infrequently. This
	 * function uses a non-blocking call and will return a cached value if the
	 * parameter is not returned by the timeout. The timeout can be changed by
	 * calling SetCANTimeout(int milliseconds)
	 *
	 * @return IdleMode Idle mode setting
	 */
	public IdleMode getIdleMode() {
		if (m_simDevice != null) {
            return IdleMode.fromId(m_simIdleMode.get());
        }
        else {
			return IdleMode.fromId(CANSparkMaxJNI.c_SparkMax_GetIdleMode(m_sparkMax));
		}
	}

	/**
	 * Sets the voltage compensation setting for all modes on the SPARK MAX and
	 * enables voltage compensation.
	 *
	 * @param nominalVoltage Nominal voltage to compensate output to
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError enableVoltageCompensation(double nominalVoltage) {
		if (m_simDevice != null) {
			m_simVoltageCompentsationEnabled.set(true);
			return REVLibError.kOk;
		}
		else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_EnableVoltageCompensation(m_sparkMax, (float)nominalVoltage));
		}
	}

	/**
	 * Disables the voltage compensation setting for all modes on the SPARK MAX.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError disableVoltageCompensation() {
		if (m_simDevice != null) {
			m_simVoltageCompentsationEnabled.set(false);
			return REVLibError.kOk;
		}
		else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_DisableVoltageCompensation(m_sparkMax));
		}
	}

	/**
	 * Get the configured voltage compensation nominal voltage value
	 *
	 * @return The nominal voltage for voltage compensation mode.
	 */
	public double getVoltageCompensationNominalVoltage() {
		if (m_simDevice != null) {
			// TODO: fix no-op?
			return 12.0;
		}
		else {
			return (double)CANSparkMaxJNI.c_SparkMax_GetVoltageCompensationNominalVoltage(m_sparkMax);
		}
	}

	/**
	 * Sets the ramp rate for open loop control modes.
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @param rate Time in seconds to go from 0 to full throttle.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError setOpenLoopRampRate(double rate) {
		if (m_simDevice != null) {
			m_simOpenLoopRampRate.set(rate);
			return REVLibError.kOk;
		}
		else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetOpenLoopRampRate(m_sparkMax, (float)rate));
		}
	}

	/**
	 * Sets the ramp rate for closed loop control modes.
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @param rate Time in seconds to go from 0 to full throttle.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError setClosedLoopRampRate(double rate) {
		if (m_simDevice != null) {
			m_simClosedLoopRampRate.set(rate);
			return REVLibError.kOk;
		}
		else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetClosedLoopRampRate(m_sparkMax, (float)rate));
		}
	}

	/**
	 * Get the configured open loop ramp rate
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @return ramp rate time in seconds to go from 0 to full throttle.
	 */
	public double getOpenLoopRampRate() {
		if (m_simDevice != null) {
			return m_simOpenLoopRampRate.get();
		}
		else {
			return (double)CANSparkMaxJNI.c_SparkMax_GetOpenLoopRampRate(m_sparkMax);
		}
	}

	/**
	 * Get the configured closed loop ramp rate
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @return ramp rate time in seconds to go from 0 to full throttle.
	 */
	public double getClosedLoopRampRate() {
		if (m_simDevice != null) {
			return m_simClosedLoopRampRate.get();
		}
		else {
			return (double)CANSparkMaxJNI.c_SparkMax_GetClosedLoopRampRate(m_sparkMax);
		}
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
     * 
     * The motor will spin in the same direction as the leader. This can be changed
     * by passing a true constant after the leader parameter.
	 * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
	 *
	 * @param leader The motor controller to follow.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError follow(final CANSparkMax leader) {
		return follow(leader, false);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
	 *
	 * @param leader The motor controller to follow.
	 * @param invert Set the follower to output opposite of the leader
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError follow(final CANSparkMax leader, boolean invert) {
		return follow(ExternalFollower.kFollowerSparkMax, leader.getDeviceId(), invert);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 * 
	 * The motor will spin in the same direction as the leader. This can be changed
	 * by passing a true constant after the deviceID parameter.
	 * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
	 *
	 * @param leader   The type of motor controller to follow (Talon SRX, Spark Max,
	 *                 etc.).
	 * @param deviceID The CAN ID of the device to follow.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError follow(ExternalFollower leader, int deviceID) {
		boolean inverted = getInverted();
		return follow(leader, deviceID, inverted);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
	 *
	 * @param leader   The type of motor controller to follow (Talon SRX, Spark Max,
	 *                 etc.).
	 * @param deviceID The CAN ID of the device to follow.
	 *
	 * @param invert   Set the follower to output opposite of the leader
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError follow(ExternalFollower leader, int deviceID, boolean invert) {
		FollowConfig maxFollower = new FollowConfig();
		maxFollower.leaderArbId = (leader.arbId == 0 ? 0 : leader.arbId | deviceID);
		maxFollower.config.predefined = leader.configId;
		maxFollower.config.invert = invert ? 1 : 0;
		return setFollow(maxFollower);
	}

	/**
	 * Returns whether the controller is following another controller
	 *
	 * @return True if this device is following another controller false otherwise
	 */
	public boolean isFollower() {
		if (m_simDevice != null) {
			return m_simIsFollower.get();
		}
		else {
			return CANSparkMaxJNI.c_SparkMax_IsFollower(m_sparkMax);
		}
	}

	/**
	 * @return All fault bits as a short
	 */
	public short getFaults() {
		if (m_simDevice != null) {
			return 0;
		}
		else {
			return (short)CANSparkMaxJNI.c_SparkMax_GetFaults(m_sparkMax);
		}
	}

	/**
	 * @return All sticky fault bits as a short
	 */
	public short getStickyFaults() {
		if (m_simDevice != null) {
			return 0;
		}
		else {
			return (short)CANSparkMaxJNI.c_SparkMax_GetStickyFaults(m_sparkMax);
		}
	}

	/**
	 * Get the value of a specific fault
	 *
	 * @param faultID The type of the fault to retrive
	 *
	 * @return True if the fault with the given type occurred.
	 */
	public boolean getFault(FaultID faultID) {
		if (m_simDevice != null) {
			return false;
		}
		else {
			return CANSparkMaxJNI.c_SparkMax_GetFault(m_sparkMax, faultID.value);
		}
	}

	/**
	 * Get the value of a specific sticky fault
	 *
	 * @param faultID The type of the sticky fault to retrive
	 *
	 * @return True if the sticky fault with the given type occurred.
	 */
	public boolean getStickyFault(FaultID faultID) {
		if (m_simDevice != null) {
			return false;
		}
		else {
			return CANSparkMaxJNI.c_SparkMax_GetStickyFault(m_sparkMax, faultID.value);
		}
	}

	/**
	 * @return The voltage fed into the motor controller.
	 */
	public double getBusVoltage() {
		if (m_simDevice != null) {
			return 12.0;
		}
		else {
			return (double)CANSparkMaxJNI.c_SparkMax_GetBusVoltage(m_sparkMax);
		}
	}

	/**
	 * @return The motor controller's applied output duty cycle.
	 */
	public double getAppliedOutput() {
		if (m_simDevice != null) {
			return m_simOutputValue.get();
		}
		else {
			return (double)CANSparkMaxJNI.c_SparkMax_GetAppliedOutput(m_sparkMax);
		}
	}

	/**
	 * @return The motor controller's output current in Amps.
	 */
	public double getOutputCurrent() {
		if (m_simDevice != null) {
			return 0;
		}
		else {
			return (double)CANSparkMaxJNI.c_SparkMax_GetOutputCurrent(m_sparkMax);
		}
	}

	/**
	 * @return The motor temperature in Celsius.
	 */
	public double getMotorTemperature() {
		if (m_simDevice != null) {
			return 0;
		}
		else {
			return (double)CANSparkMaxJNI.c_SparkMax_GetMotorTemperature(m_sparkMax);
		}
	}

	/**
	 * Clears all sticky faults.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError clearFaults() {
		if (m_simDevice != null) {
			return REVLibError.kOk;
		}
		else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_ClearFaults(m_sparkMax));
		}
	}

	/**
	 * Writes all settings to flash.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError burnFlash() {
		if (m_simDevice != null) {
			return REVLibError.kOk;
		}
		else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_BurnFlash(m_sparkMax));
		}
	}

	/**
	 * Sets timeout for sending CAN messages with SetParameter* and GetParameter* calls.
	 * These calls will block for up to this amoutn of time before returning a timeout
	 * erro. A timeout of 0 will make the SetParameter* calls non-blocking, and instead
	 * will check the response in a separate thread. With this configuration, any
	 * error messages will appear on the drivestration but will not be returned by the
	 * GetLastError() call.
	 *
	 * @param milliseconds The timeout in milliseconds.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 */
	public REVLibError setCANTimeout(int milliseconds) {
		if (m_simDevice != null) {
			return REVLibError.kOk;
		}
		else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetCANTimeout(m_sparkMax, milliseconds));
		}
	}

	/**
     * Enable soft limits
     *
     * @param direction the direction of motion to restrict
     * 
     * @param enable set true to enable soft limits
	 * 
	 * @return REVLibError Set to REVLibError.kOK if successful
     */
	public REVLibError enableSoftLimit(SoftLimitDirection direction, boolean enable) {
		if (m_simDevice != null) {
            switch (direction) {
                case kForward:
                    m_simSoftLimitFwdEnabled.set(enable);
                    break;
                case kReverse:
                    m_simSoftLimitRevEnabled.set(enable);
                    break;
                default:
                    return REVLibError.kInvalid;
            }
            return REVLibError.kOk;
        }
        else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_EnableSoftLimit(m_sparkMax, direction.value, enable));
		}
	}

	 /**
     * Set the soft limit based on position. The default unit is
     * rotations, but will match the unit scaling set by the user.
     * 
     * Note that this value is not scaled internally so care must
     * be taken to make sure these units match the desired conversion
     *
     * @param direction the direction of motion to restrict
     * 
     * @param limit position soft limit of the controller
	 * 
	 * @return REVLibError Set to REVLibError.kOK if successful
     */
	public REVLibError setSoftLimit(SoftLimitDirection direction, float limit) {
		if (m_simDevice != null) {
            switch (direction) {
                case kForward:
                    m_simSoftLimitFwd.set(limit);
                    break;
                case kReverse:
                    m_simSoftLimitRev.set(limit);
                    break;
                default:
                    return REVLibError.kInvalid;
            }
            return REVLibError.kOk;
        }
        else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSoftLimit(m_sparkMax, direction.value, limit));
		}
	}

	/**
     * Get the soft limit setting in the controller
     *
     * @param direction the direction of motion to restrict
     * 
     * @return position soft limit setting of the controller
     */
	public double getSoftLimit(SoftLimitDirection direction) {
		if (m_simDevice != null) {
            switch (direction) {
                case kForward:
                    return m_simSoftLimitFwd.get();
                case kReverse:
                    return m_simSoftLimitRev.get();
                default:
                    return 0.0;
            }
        }
        else {
			return (double)CANSparkMaxJNI.c_SparkMax_GetSoftLimit(m_sparkMax, direction.value);
		}
	}

	/**
	 * @param direction The direction of the motion to restrict
	 * 
     * @return true if the soft limit is enabled.
     */
	public boolean isSoftLimitEnabled(SoftLimitDirection direction) {
		if (m_simDevice != null) {
            switch (direction) {
                case kForward:
                    return m_simSoftLimitFwdEnabled.get();
                case kReverse:
					return m_simSoftLimitRevEnabled.get();
				default:
					return false;
			}
        }
        else {
			return (boolean)CANSparkMaxJNI.c_SparkMax_IsSoftLimitEnabled(m_sparkMax, direction.value);
		}
	}

	/**
	 * Gets the feedback device ID that was set on SparkMax itself.
	 * 
	 * @return Feedback device ID on the SparkMax
	 */
	protected int getFeedbackDeviceID() {
		if (m_simDevice != null) {
            return -1;
        }
        else {
			return (int)CANSparkMaxJNI.c_SparkMax_GetFeedbackDeviceID(m_sparkMax);
		}
	}

	/**
     * All device errors are tracked on a per thread basis for all
     * devices in that thread. This is meant to be called 
     * immediately following another call that has the possibility 
     * of returning an error to validate if an  error has occurred. 
     * 
     * @return the last error that was generated.
     */
	public REVLibError getLastError() {
		if (m_simDevice != null) {
            return REVLibError.kOk;
        }
        else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_GetLastError(m_sparkMax));
		}
	}
}
