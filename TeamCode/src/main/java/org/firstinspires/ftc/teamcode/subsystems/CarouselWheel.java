package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CarouselWheel extends SubsystemBase {


    private final DcMotor wheelMotor;
    private double power = 0.0; //the power for the wheel
    private final Telemetry telemetry;

    public CarouselWheel(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public CarouselWheel(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;
        wheelMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
        wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void periodic(){

        if(telemetry != null){
            telemetry.addData("Carousel Wheel", (power != 0.0) ? "ON" : "OFF");
        }
    }


    /**
     * Forward is clockwise
     * @param power The power to set the motor to
     */
    public void setWheelPower(double power){
        wheelMotor.setPower(power);
    }



}
