package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {


    private DcMotorSimple frontIntake;
    private DcMotorSimple backIntake;
    private Telemetry telemetry;
    private State state = State.STOP;
    private boolean front = true;


    public enum State {
        INTAKE(0.65),
        OUTTAKE(-0.5),
        STOP(0.0);

        public double power;
        State(double power){
            this.power = power;
        }
    }


    public Intake(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }


    public Intake(HardwareMap hardwareMap, Telemetry telemetry){

        frontIntake = hardwareMap.get(DcMotorSimple.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorSimple.class, "backIntake");

        backIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (telemetry != null) telemetry.addData("Intake State", state);
    }

    public void setState(State state){
        if(front) frontIntake.setPower(state.power);
        else backIntake.setPower(state.power);
        this.state = state;
    }


    public void setSide(boolean front){
        this.front = front;
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
