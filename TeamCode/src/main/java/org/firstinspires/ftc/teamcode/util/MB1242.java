package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;


@I2cDeviceType
@DeviceProperties(
        name = "MB1242",
        description = "ultrasonic distance sensor",
        xmlTag = "MB1242"
)
public class MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> {


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "MB1242 I2C Ultrasonic Distance Sensor";
    }

    public MB1242(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0xe1));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public void ping() {
        deviceClient.write(0xE0, TypeConversion.intToByteArray(0x51));
    }

    /**
     * Allow 100ms between pinging.
     */
    public short readRange(){
        return TypeConversion.byteArrayToShort(deviceClient.read(0xE1, 2));
    }


}
