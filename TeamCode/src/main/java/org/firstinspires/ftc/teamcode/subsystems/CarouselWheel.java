package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CarouselWheel extends SubsystemBase {


    private final DcMotor leftWheelMotor;
    private final DcMotor rightWheelMotor;

    private double leftPower = 0.0; //the power for the wheel
    private double rightPower = 0.0;

    private Telemetry telemetry;

    public CarouselWheel(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public CarouselWheel(HardwareMap hardwareMap, Telemetry telemetry){
        super(); //Call the super class constructor to register with the scheduler

        this.telemetry = telemetry;

        leftWheelMotor = hardwareMap.get(DcMotor.class, "leftCarouselMotor");
        rightWheelMotor = hardwareMap.get(DcMotor.class, "rightCarouselMotor");

        rightWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Not really necessary
        rightWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void periodic(){
        leftWheelMotor.setPower(leftPower);
        rightWheelMotor.setPower(rightPower);

        if(telemetry != null){
            telemetry.addData("Left wheel activated", leftPower != 0.0);
            telemetry.addData("Right wheel activated", rightPower != 0.0);
        }
    }


    public void leftForward(){
        leftPower = 0.55;
    }

    public void rightForward(){
        rightPower = 0.55;
    }

    public void slowLeftForward(){
        leftPower = 0.2;
    }

    public void slowRightForward(){
        rightPower = 0.2;
    }

    public void fastLeftForward(){
        leftPower = 0.7;
    }

    public void fastRightForward(){
        rightPower = 0.7;
    }

    public void leftStop(){
        leftPower = 0.0;
    }

    public void rightStop(){
        rightPower = 0.0;
    }



}
