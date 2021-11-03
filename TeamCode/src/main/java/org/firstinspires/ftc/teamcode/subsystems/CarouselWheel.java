package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselWheel extends SubsystemBase {


    private final DcMotor wheelMotor;

    private double power = 0.0; //the power for the wheel

    public CarouselWheel(HardwareMap hardwareMap){
        super(); //Call the super class constructor to register with the scheduler

        wheelMotor = hardwareMap.get(DcMotor.class, "carouselMotor");

        wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //Not really necessary
    }


    @Override
    public void periodic(){
        wheelMotor.setPower(power);
    }


    public void forward(){
        power = 0.8;
    }

    public void back(){
        power = -0.8;
    }

    public void setPower(double power){
        this.power = power;
    }

}
