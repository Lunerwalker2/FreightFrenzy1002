package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CappingArm extends SubsystemBase {

    Servo cappingServo;

    private Telemetry telemetry;

    public enum Positions {
        IN_ROBOT(0.0),
        ABOVE_POLE(0.5),
        ON_POLE(0.62),
        PICK_UP(0.85);

        public double position;
        Positions(double position){
            this.position = position;
        }
    }

    private Positions[] positionsList = Positions.values();
    private Positions currentPosition = Positions.IN_ROBOT;

    public CappingArm(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public CappingArm(HardwareMap hardwareMap, Telemetry telemetry){
        cappingServo = hardwareMap.get(Servo.class, "cappingServo");
        this.telemetry = telemetry;
        cappingServo.setPosition(currentPosition.position);
    }


    @Override
    public void periodic() {
        if(telemetry != null){
            telemetry.addData("Capping Arm Position", currentPosition);
        }
    }

    public void setArmPosition(Positions position){
        currentPosition = position;
        cappingServo.setPosition(position.position);
    }

    public void setArmPositionRaw(double position){
        cappingServo.setPosition(position);
    }

    public void incrementPosition(){
        int nextPos = currentPosition.ordinal() + 1;
        if(nextPos < positionsList.length) setArmPosition(positionsList[nextPos]);
    }

    public void decrementPosition(){
        int nextPos = currentPosition.ordinal() - 1;
        if(nextPos >= 0) setArmPosition(positionsList[nextPos]);
    }

}
