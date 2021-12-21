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
    private DistanceSensor freightSensor;
    private static final double freightDetectedThreshold = 3.0;
    private int numLoops = 0;
    public boolean freightDetected = false;


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

        frontIntake = hardwareMap.get(DcMotorSimple.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorSimple.class, "backIntake");

//        freightSensor = (DistanceSensor) hardwareMap.get(RevColorSensorV3.class, "freightSensor");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        if(telemetry != null) telemetry.addData("Intake State", state);
        //reading i2c sensors is expensive, so only read every so often.
//        numLoops++;
//        if(numLoops >= 4){
//            freightDetected = checkFreightDetected();
//            numLoops = 0;
//        }
    }

    public void setState(State state){
        if(front) frontIntake.setPower(state.power);
        else backIntake.setPower(state.power);
        this.state = state;
    }

    public boolean checkFreightDetected(){
        return freightSensor.getDistance(DistanceUnit.INCH) < freightDetectedThreshold;
    }

    public void setSide(boolean front){
        this.front = front;
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
