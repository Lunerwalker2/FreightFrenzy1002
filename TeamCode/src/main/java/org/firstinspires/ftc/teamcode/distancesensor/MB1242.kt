package org.firstinspires.ftc.teamcode.distancesensor

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadWindow
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import com.qualcomm.robotcore.util.TypeConversion
import java.util.*

@I2cDeviceType
@DeviceProperties(name = "MB1242 I2C Ultrasonic Distance Sensor",
description = "Ultrasonic Distance Sensor from Maxbotics",
xmlTag = "MB1242")
class MB1242(deviceClient: I2cDeviceSynch): I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, true) {



    init {
        this.deviceClient.i2cAddress = I2cAddr.create8bit(0xE1)

        super.registerArmingStateCallback(false)
        this.deviceClient.engage()
    }


    /**
     * Performs a range command on the sensor. Allow at least 80-100ms before reading.
     */
    protected fun initiateRangeCommand() {
        /*
        MB1242 Datasheet:
        Write to address 1110 0000 - the last 0 indicates a write
        The byte 0101 0001 (decimal 81) - Commands the sensor to take a range measurement


        "It is best to allow 100ms between readings to allow for proper acoustic dissipation."
         */
        deviceClient.write8(0xE0, 0x51)//default address with 0 at the end for write
    }

    /**
     * Reads the range from the last command in cm. Allow at least 80-100ms after the range command
     * in order to read.
     */
    protected fun readRangeValueCm(): Short {
        /*
        MB1242 Datasheet:
        Read from address 1110 0001 -  the last 1 indicates a read
        Read two bytes of data - First is the high range and the second is the low range

        Value is the range in cm from last range reading. Allow at least 80m between last range
        command.
         */
        return TypeConversion.byteArrayToShort(deviceClient.read(0xE1, 2))
    }



    @Synchronized
    override fun doInitialize(): Boolean {
        return true
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown

    override fun getDeviceName(): String = "MB1242 I2C Ultrasonic Distance Sensor"


}