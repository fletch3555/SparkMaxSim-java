package net.thefletcher.revrobotics;

import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.hal.*;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;

import net.thefletcher.revrobotics.enums.*;

public abstract class CANSparkMaxLowLevel implements SpeedController, MotorController, AutoCloseable {
	public static final int kAPIMajorVersion = CANSparkMaxJNI.c_SparkMax_GetAPIMajorRevision();
	public static final int kAPIMinorVersion = CANSparkMaxJNI.c_SparkMax_GetAPIMinorRevision();
	public static final int kAPIBuildVersion = CANSparkMaxJNI.c_SparkMax_GetAPIBuildRevision();
	public static final int kAPIVersion = CANSparkMaxJNI.c_SparkMax_GetAPIVersion();

	public class PeriodicStatus0 {
		public double appliedOutput;
		public short faults;
		public short stickyFaults;
		public byte lock;
		public MotorType motorType;
		public boolean isFollower;
		public boolean isInverted;
		public boolean roboRIO;
	}

	public class PeriodicStatus1 {
		public double sensorVelocity;
		public byte motorTemperature;
		public double busVoltage;
		public double outputCurrent;
	}

	public class PeriodicStatus2 {
		public double sensorPosition;
		public double iAccum;
	}
	
	protected final AtomicBoolean isClosed = new AtomicBoolean(false);

	protected SimDevice m_simDevice;
	protected SimEnum m_simMotorType;
	protected MotorType motorType;
	protected SimBoolean m_simIsFollower;
	protected SimDouble m_simOutputValue;
	protected SimEnum m_simControlType;

	/**
	 * Create a new SPARK MAX Controller
	 *
	 * @param deviceID The device ID.
	 * @param type     The motor type connected to the controller. Brushless motors
	 *                 must be connected to their matching color and the hall sensor
	 *                 plugged in. Brushed motors must be connected to the Red and
	 *                 Black terminals only.
	 */
	public CANSparkMaxLowLevel(int deviceID, MotorType type) {
		if (type == null) {
			throw new IllegalArgumentException("type must not be null");
		}

		m_simDevice = SimDevice.create("CANSparkMax", deviceID);
		if (m_simDevice != null) {
			m_simMotorType = m_simDevice.createEnum("Motor Type", Direction.kOutput, new String[] { "Brushed", "Brushless" }, type.value);
			m_simIsFollower = m_simDevice.createBoolean("Follower", Direction.kOutput, false);
			m_simOutputValue = m_simDevice.createDouble("Output", Direction.kOutput, 0.0);
			m_simControlType = m_simDevice.createEnum("Control Type", Direction.kOutput, new String[] { "DutyCycle", "Velocity", "Voltage", "Position", "SmartMotion", "Current", "SmartVelocity" }, 0);
		}

		m_deviceID = deviceID;
		m_firmwareString = "";
		m_motorType = type;

		if (m_simDevice != null) {
			m_sparkMax = -1;
		} else {
			if (CANSparkMaxJNI.c_SparkMax_RegisterId(deviceID) == REVLibError.kDuplicateCANId.value) {
				throw new IllegalStateException("A CANSparkMax instance has already been created with this device ID: " + deviceID);
			}
			m_sparkMax = CANSparkMaxJNI.c_SparkMax_Create(deviceID, type.value);
		}

	}

	/** Closes the SPARK MAX Controller */
	@Override
	public void close() {
		boolean alreadyClosed = isClosed.getAndSet(true);
		if (alreadyClosed) {
			return;
		}
		
		if (m_simDevice != null) {
			m_sparkMax = -1;
		} else {
			CANSparkMaxJNI.c_SparkMax_Destroy(m_sparkMax);
		}
	}

	/**
	 * Get the firmware version of the SPARK MAX.
	 *
	 * @return uint32_t Firmware version integer. Value is represented as 4 bytes,
	 *         Major.Minor.Build H.Build L
	 *
	 */
	public int getFirmwareVersion() {
		throwIfClosed();
		if (m_simDevice != null) {
			return -1;
		} else {
			return CANSparkMaxJNI.c_SparkMax_GetFirmwareVersion(m_sparkMax);
		}
	}

	/**
	 * Set the control frame send period for the native CAN Send thread.
	 * 
	 * @param periodMs The send period in milliseconds between 1ms and 100ms or set
	 *                 to 0 to disable periodic sends. Note this is not updated
	 *                 until the next call to Set() or SetReference().
	 * 
	 */
	public void setControlFramePeriodMs(int periodMs) {
		throwIfClosed();
		if (m_simDevice != null) {
			// TODO: fix no-op?
		} else {
			CANSparkMaxJNI.c_SparkMax_SetControlFramePeriod(m_sparkMax, periodMs);
		}
	}

	/**
	 * Get the firmware version of the SPARK MAX as a string.
	 *
	 * @return std::string Human readable firmware version string
	 *
	 */
	public String getFirmwareString() {
		throwIfClosed();
		if (m_firmwareString == "") {
			int version = getFirmwareVersion();
			ByteBuffer b = ByteBuffer.allocate(4);
			b.putInt(version);
	  
			byte[] verBytes = b.array();
	  
			StringBuilder firmwareString = new StringBuilder();
			firmwareString
				.append("v")
				.append((int) verBytes[0])
				.append(".")
				.append((int) verBytes[1])
				.append(".")
				.append((int) verBytes[2] << 8 | (int) verBytes[3]);
	  
			this.m_firmwareString = firmwareString.toString();
		}
		return m_firmwareString;
	}

	/**
	 * Get the unique serial number of the SPARK MAX. Not currently available.
	 *
	 * @return byte[] Vector of bytes representig the unique serial number
	 *
	 */
	public byte[] getSerialNumber() {
		throwIfClosed();
		return new byte[0];
	}

	/**
	 * Get the configured Device ID of the SPARK MAX.
	 *
	 * @return int device ID
	 *
	 */
	public int getDeviceId() {
		throwIfClosed();
		return m_deviceID;
	}

	/**
	 * Get the motor type setting from when the SparkMax was created.
	 * 
	 * This does not use the Get Parameter API which means it does not read what
	 * motor type is stored on the SparkMax itself. Instead, it reads the stored
	 * motor type from when the SparkMax object was first created.
	 * 
	 * @return MotorType Motor type setting
	 */
	public MotorType getInitialMotorType() {
		throwIfClosed();
		return m_motorType;
	}

	/**
	 * Get the motor type setting for the SPARK MAX.
	 *
	 * This uses the Get Parameter API and should be used infrequently. This
	 * function uses a non-blocking call and will return a cached value if the
	 * parameter is not returned by the timeout. The timeout can be changed by
	 * calling SetCANTimeout(int milliseconds)
	 *
	 * @return MotorType Motor type setting
	 *
	 */
	public MotorType getMotorType() {
		throwIfClosed();
		if (m_simDevice != null) {
			return MotorType.fromId(m_simMotorType.get());
		} else {
			return MotorType.fromId(CANSparkMaxJNI.c_SparkMax_GetMotorType(m_sparkMax));
		}
	}

	/**
	 * Set the rate of transmission for periodic frames from the SPARK MAX
	 *
	 * Each motor controller sends back three status frames with different data at
	 * set rates. Use this function to change the default rates.
	 *
	 * Defaults: Status0 - 10ms Status1 - 20ms Status2 - 50ms
	 *
	 * This value is not stored in the FLASH after calling burnFlash() and is reset
	 * on powerup.
	 *
	 * Refer to the SPARK MAX reference manual on details for how and when to
	 * configure this parameter.
	 *
	 * @param frameID  The frame ID can be one of PeriodicFrame type
	 * @param periodMs The rate the controller sends the frame to the controller.
	 *
	 * @return REVLibError Set to REVLibError.kOK if successful
	 *
	 */
	public REVLibError setPeriodicFramePeriod(PeriodicFrame frameID, int periodMs) {
		throwIfClosed();
		if (m_simDevice != null) {
			// TODO: fix no-op?
			return REVLibError.kOk;
		} else {
			return REVLibError.fromInt(
				CANSparkMaxJNI.c_SparkMax_SetPeriodicFramePeriod(m_sparkMax, frameID.value, periodMs));
		}
	}

	/**
	 * Allow external controllers to recieve control commands over USB. For example,
	 * a configuration where the heartbeat (and enable/disable) is sent by the main
	 * controller, but control frames are sent by other CAN devices over USB.
	 * 
	 * This is global for all controllers on the same bus.
	 * 
	 * This does not disable sending control frames from this device. To prevent
	 * conflicts, do not enable this feature and also send Set() for SetReference()
	 * from the controllers you wish to control.
	 *
	 * @param enable Enable or disable external control
	 *
	 */
	public static void enableExternalUSBControl(boolean enable) { // TODO: stupid statics...
		CANSparkMaxJNI.c_SparkMax_EnableExternalControl(enable);
	}

	/**
	 * Send enabled or disabled command to controllers. This is global for all
	 * controllers on the same bus, and will only work for non-roboRIO targets in
	 * non-competiton use. This function will also not work if a roboRIO is present
	 * on the CAN bus.
	 * 
	 * This does not disable sending control frames from this device. To prevent
	 * conflicts, do not enable this feature and also send Set() for SetReference()
	 * from the controllers you wish to control.
	 *
	 * @param enable Enable or disable external control
	 *
	 */
	static void setEnable(boolean enable) { // TODO: stupid statics...
		CANSparkMaxJNI.c_SparkMax_SetEnable(enable);
	}

	REVLibError setFollow(FollowConfig follower) {
		throwIfClosed();
		if (m_simDevice != null) {
			m_simIsFollower.set(true);
			return REVLibError.kOk;
		} else {
			return REVLibError.fromInt(
				CANSparkMaxJNI.c_SparkMax_SetFollow(m_sparkMax, follower.leaderArbId, follower.config.getRaw()));
		}
	}

	REVLibError setpointCommand(double value) {
		throwIfClosed();
		return setpointCommand(value, ControlType.kDutyCycle);
	}

	REVLibError setpointCommand(double value, ControlType ctrl) {
		throwIfClosed();
		return setpointCommand(value, ctrl, 0);
	}

	REVLibError setpointCommand(double value, ControlType ctrl, int pidSlot) {
		throwIfClosed();
		return setpointCommand(value, ctrl, pidSlot, 0);
	}

	REVLibError setpointCommand(double value, ControlType ctrl, int pidSlot, double arbFeedforward) {
		throwIfClosed();
		return setpointCommand(value, ctrl, pidSlot, arbFeedforward, 0);
	}

	REVLibError setpointCommand(double value, ControlType ctrl, int pidSlot, double arbFeedforward, int arbFFUnits) {
		throwIfClosed();
		if (m_simDevice != null) {
			m_simOutputValue.set(value);
			m_simControlType.set(ctrl.value);
			return REVLibError.kOk;
		} else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetpointCommand(m_sparkMax, (float) value, ctrl.value,
					pidSlot, (float) arbFeedforward, arbFFUnits));
		}
	}

	public float getSafeFloat(float f) {
		throwIfClosed();
		if (Float.isNaN(f) || Float.isInfinite(f))
			return 0;

		return f;
	}

	/**
	 * Restore motor controller parameters to factory default until the next
	 * controller reboot
	 *
	 * @return REVLibError Set to REVLibError::kOk if successful
	 */
	public REVLibError restoreFactoryDefaults() {
		throwIfClosed();
		return restoreFactoryDefaults(false);
	}

	/**
	 * Restore motor controller parameters to factory default
	 *
	 * @param persist If true, burn the flash with the factory default parameters
	 *
	 * @return REVLibError Set to REVLibError::kOk if successful
	 */
	public REVLibError restoreFactoryDefaults(boolean persist) {
		throwIfClosed();
		if (m_simDevice != null) {
			// TODO: fix no-op?
			return REVLibError.kOk;
		} else {
			return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_RestoreFactoryDefaults(m_sparkMax, persist));
		}
	}

	protected void throwIfClosed() {
	  if (isClosed.get()) {
		throw new IllegalStateException("This SPARK MAX object has previously been closed.");
	  }
	}

	protected static class FollowConfig {
		int leaderArbId;
		Config config;

		public static class Config {
			public int rsvd1;
			public int invert;
			public int rsvd2;
			public int predefined;

			public int getRaw() {
				return (rsvd1 & 0x3FFFF) | (invert & 0x1) << 18 | (rsvd2 & 0x1F) << 19 | (predefined & 0xFF) << 24;
			}
		}

		FollowConfig() {
			config = new Config();
		}
	}

	protected long m_sparkMax;
	private final int m_deviceID;
	private String m_firmwareString;
	protected final MotorType m_motorType;
}
