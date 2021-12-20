package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {


    private DcMotorSimple intakeMotor;
    private Telemetry telemetry;
    private State state = State.STOP;

    public enum State {
        INTAKE(1),
        OUTTAKE(-1),
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

        intakeMotor = hardwareMap.get(DcMotorSimple.class, "intakeMotor");
        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        if(telemetry != null) telemetry.addData("Intake State", state);
    }

    public void setState(State state){
        intakeMotor.setPower(state.power);
        this.state = state;
    }

    public void intake(){
        setState(State.INTAKE);
    }

    public void outtake(){
        setState(State.OUTTAKE);
    }

    public void stop(){
        setState(State.STOP);
    }

}
