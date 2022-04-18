package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {

    public enum LiftPosition {
        BOTTOM(300),
        MIDDLE(400),
        TOP(575);

        public int position;

        LiftPosition(int position){
            this.position = position;
        }
    }

    private final DcMotorEx liftMotor;

    public Lift(HardwareMap hardwareMap) {

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic(){
        //happens every loop
    }

    public void setLiftPower(double power){
        liftMotor.setPower(power);
    }

    public void stop(){
        liftMotor.setPower(0);
    }

    public double getLiftPosition(){
        return liftMotor.getCurrentPosition();
    }

    public void resetLiftPosition(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //SDK will automatically switch back to regular mode
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean atUpperLimit(){
        return getLiftPosition() > 620;
    }

    public boolean atLowerLimit(){
        return getLiftPosition() < 5;
    }

}
