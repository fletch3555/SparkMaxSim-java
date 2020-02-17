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
import net.thefletcher.revrobotics.enums.*;

public class CANAnalog implements CANSensor {
    public CANSparkMax m_device;

    private SimDevice m_simDevice;
    private SimEnum m_simAnalogMode;
    private SimDouble m_simVoltage;
    private SimDouble m_simPosition;
    private SimDouble m_simVelocity;
    private SimDouble m_simPositionConversionFactor;
    private SimDouble m_simVelocityConversionFactor;
    private SimBoolean m_simInverted;

    /**
     * Constructs a CANAnalog.
     *
     * @param device The Spark Max to which the analog sensor is attached.
     * @param mode   The mode of the analog sensor, either absolute or relative
     */
    public CANAnalog(CANSparkMax device, AnalogMode mode) {
        m_device = device;

        m_simDevice = SimDevice.create("CANAnalog", device.getDeviceId());
        if (m_simDevice != null) {
            m_simAnalogMode = m_simDevice.createEnum("Analog Mode", true, new String[] {"Absolute", "Relative"}, mode.value);
            m_simVoltage = m_simDevice.createDouble("Voltage", false, 0.0);
            m_simPosition = m_simDevice.createDouble("Position", false, 0.0);
            m_simVelocity = m_simDevice.createDouble("Velocity", false, 0.0);
            m_simPositionConversionFactor = m_simDevice.createDouble("Position Conversion Factor", false, 0.0);
            m_simVelocityConversionFactor = m_simDevice.createDouble("Velocity Conversion Factor", false, 0.0);
            m_simInverted = m_simDevice.createBoolean("Inverted", false, false);
        }
        else {
            CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAnalogMode(m_device.m_sparkMax, mode.value));
        }
    }

    /**
     * Get the voltage of the analog sensor.
     * 
     * @return Voltage of the sensor.
     */
    public double getVoltage() {
        if (m_simDevice != null) {
            return m_simVoltage.get();
        }
        else {
            return (double) CANSparkMaxJNI.c_SparkMax_GetAnalogVoltage(m_device.m_sparkMax);
        }
    }

    /**
     * Get the position of the motor. Returns value in the native unit of 'volt' by
     * default, and can be changed by a scale factor using
     * setPositionConversionFactor().
     * 
     * @return Position of the sensor in volts
     */
    public double getPosition() {
        if (m_simDevice != null) {
            return m_simPosition.get();
        }
        else {
            return (double) CANSparkMaxJNI.c_SparkMax_GetAnalogPosition(m_device.m_sparkMax);
        }
    }

    /**
     * Get the velocity of the motor. Returns value in the native units of 'volts
     * per second' by default, and can be changed by a scale factor using
     * setVelocityConversionFactor().
     * 
     * @return Velocity of the sensor in volts per second
     */
    public double getVelocity() {
        if (m_simDevice != null) {
            return m_simVelocity.get();
        }
        else {
            return (double) CANSparkMaxJNI.c_SparkMax_GetAnalogVelocity(m_device.m_sparkMax);
        }
    }

    /**
     * Set the conversion factor for the position of the analog sensor. By default,
     * revolutions per volt is 1. Changing the position conversion factor will also
     * change the position units.
     * 
     * @param factor The conversion factor which will be multiplied by volts
     * 
     * @return CANError Set to CANError.kOK if successful
     */
    public CANError setPositionConversionFactor(double factor) {
        if (m_simDevice != null) {
            m_simPositionConversionFactor.get();
            return CANError.kOk;
        }
        else {
            return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAnalogPositionConversionFactor(m_device.m_sparkMax, (float) factor));
        }
    }

    /**
     * Set the conversion factor for the veolocity of the analog sensor. By default,
     * revolutions per volt second is 1. Changing the velocity conversion factor
     * will also change the velocity units.
     * 
     * @param factor The conversion factor which will be multipled by volts per
     *               second
     * 
     * @return CANError Set to CANError.kOK is successful
     */
    public CANError setVelocityConversionFactor(double factor) {
        if (m_simDevice != null) {
            m_simVelocityConversionFactor.set(factor);
            return CANError.kOk;
        }
        else {
            return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAnalogVelocityConversionFactor(m_device.m_sparkMax, (float) factor));
        }
    }

    /**
     * Get the current conversion factor for the position of the analog sensor.
     * 
     * @return Analog position conversion factor
     */
    public double getPositionConversionFactor() {
        if (m_simDevice != null) {
            return m_simPositionConversionFactor.get();
        }
        else {
            return CANSparkMaxJNI.c_SparkMax_GetAnalogPositionConversionFactor(m_device.m_sparkMax);
        }
    }

    /**
     * Get the current conversion factor for the velocity of the analog sensor.
     * 
     * @return Analog velocity conversion factor
     */
    public double getVelocityConversionFactor() {
        if (m_simDevice != null) {
            return m_simVelocityConversionFactor.get();
        }
        else {
            return CANSparkMaxJNI.c_SparkMax_GetAnalogVelocityConversionFactor(m_device.m_sparkMax);
        }
    }

    @Override
    public int getID() {
        return FeedbackSensorType.kAnalog.value;
    }

    @Override
    public CANError setInverted(boolean inverted) {
        if (m_simDevice != null) {
            m_simInverted.set(inverted);
            return CANError.kOk;
        }
        else {
            return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAnalogInverted(m_device.m_sparkMax, inverted));
        }
    }

    @Override
    public boolean getInverted() {
        if (m_simDevice != null) {
            return m_simInverted.get();
        }
        else {
            return (boolean) CANSparkMaxJNI.c_SparkMax_GetAnalogInverted(m_device.m_sparkMax);
        }
    }
}