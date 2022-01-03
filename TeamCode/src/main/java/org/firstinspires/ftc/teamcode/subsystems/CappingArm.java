package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CappingArm extends SubsystemBase {

    Servo cappingServo;

    private Telemetry telemetry;

    private final double inRobotPosition = 0.0;
    private double currentPosition = 0.0;

    public CappingArm(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public CappingArm(HardwareMap hardwareMap, Telemetry telemetry){
        cappingServo = hardwareMap.get(Servo.class, "cappingServo");
        this.telemetry = telemetry;
        cappingServo.setPosition(inRobotPosition);
    }


    @Override
    public void periodic() {
        if(telemetry != null){
            telemetry.addData("Capping Arm Position", currentPosition);
        }
    }


    public void setArmPositionRaw(double position){
        currentPosition = position;
        cappingServo.setPosition(position);
    }

    public void raise(){
        currentPosition += 0.001;
        if(currentPosition > 1.0) currentPosition = 1.0;
        cappingServo.setPosition(currentPosition);
    }

    public void lower(){
        currentPosition -= 0.001;
        if(currentPosition < 0.0) currentPosition = 0.0;
        cappingServo.setPosition(currentPosition);
    }

}
