package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {


    private final DcMotorEx frontIntake;
    private final DcMotorEx backIntake;
    private final Servo frontFlap;
    private final Servo backFlap;
    private Telemetry telemetry;
    private State state = State.STOP;
    private boolean front = true;


    public enum State {
        INTAKE(0.7, 0.7),//0.57, 0.38
        OUTTAKE(-0.8, -0.8),
        STOP(0.0, 0.0);

        public double powerFront;
        public double powerBack;
        State(double powerFront, double powerBack){
            this.powerFront = powerFront;
            this.powerBack = powerBack;
        }
    }


    public Intake(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }


    public Intake(HardwareMap hardwareMap, Telemetry telemetry){

        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");

        frontFlap = hardwareMap.get(Servo.class, "frontFlap");
        backFlap = hardwareMap.get(Servo.class, "backFlap");

        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType type = frontIntake.getMotorType();
        type.setAchieveableMaxRPMFraction(0.9);
        frontIntake.setMotorType(type);
        backIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType type2 = backIntake.getMotorType();
        type.setAchieveableMaxRPMFraction(0.9);
        backIntake.setMotorType(type2);

//        backIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (telemetry != null) telemetry.addData("Intake State", state);
    }

    public void setState(State state){
        if(state != State.STOP){
            if (front) frontIntake.setPower(state.powerFront);
            else backIntake.setPower(state.powerBack);
        } else {
            frontIntake.setPower(0);
            backIntake.setPower(0);
        }
        this.state = state;
    }

    public void outtakeBoth(){
        backIntake.setPower(State.OUTTAKE.powerBack);
        frontIntake.setPower(State.OUTTAKE.powerFront);
    }

    public void setSide(boolean front){
        this.front = front;
        if(front){
            frontFlap.setPosition(1.0);
            backFlap.setPosition(1.0);
        } else {
            frontFlap.setPosition(0.0);
            backFlap.setPosition(0.0);
        }
    }

    /**
     * Starts the intake on whatever the current side is.
     */
    public void intake(){
        setState(State.INTAKE);
    }

    /**
     * Starts the outtake on whatever the current side is.
     */
    public void outtake(){
        setState(State.OUTTAKE);
    }

    public void stop(){
        setState(State.STOP);
    }

}
