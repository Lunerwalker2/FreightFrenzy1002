package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselWheel extends SubsystemBase {


    private final DcMotor leftWheelMotor;
    private final DcMotor rightWheelMotor;

    private double leftPower = 0.0; //the power for the wheel
    private double rightPower = 0.0;

    public CarouselWheel(HardwareMap hardwareMap){
        super(); //Call the super class constructor to register with the scheduler

        leftWheelMotor = hardwareMap.get(DcMotor.class, "leftCarouselMotor");
        rightWheelMotor = hardwareMap.get(DcMotor.class, "rightCarouselMotor");

        leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //Not really necessary
        rightWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    @Override
    public void periodic(){
        leftWheelMotor.setPower(leftPower);
        rightWheelMotor.setPower(rightPower);
    }


    public void leftForward(){
        leftPower = 0.8;
    }

    public void rightForward(){
        rightPower = 0.8;
    }

    public void leftStop(){
        leftPower = 0.0;
    }

    public void rightStop(){
        rightPower = 0.0;
    }

    public void setPower(double power){
        this.leftPower = power;
        this.rightPower = power;
    }

}
