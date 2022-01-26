package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CappingArm extends SubsystemBase {

    private Servo cappingServo;
    private Servo swivelServo;

    private Telemetry telemetry;

    private final double inRobotArmPosition = 0.0;
    private final double inRobotSwivelPosition = 0.0;
    private double currentArmPosition = 0.0;
    private double currentSwivelPosition = 0.0;

    public CappingArm(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public CappingArm(HardwareMap hardwareMap, Telemetry telemetry){
        cappingServo = hardwareMap.get(Servo.class, "cappingServo");
//        swivelServo = hardwareMap.get(Servo.class, "swivelServo");
        this.telemetry = telemetry;
        cappingServo.setPosition(inRobotArmPosition);
//        swivelServo.setPosition(inRobotSwivelPosition);
    }


    @Override
    public void periodic() {
        if(telemetry != null){
            telemetry.addData("Capping Arm Position", currentArmPosition);
        }
    }


    public void setArmPositionRaw(double position){
        currentArmPosition = position;
        cappingServo.setPosition(position);
    }
//
//    public void swivelLeft(){
//        currentSwivelPosition -= 0.001;
//        if(currentSwivelPosition < 0.0) currentSwivelPosition = 0.0;
//        swivelServo.setPosition(currentSwivelPosition);
//    }
//
//    public void swivelRight(){
//        currentSwivelPosition += 0.001;
//        if(currentSwivelPosition > 1.0) currentSwivelPosition = 1.0;
//        swivelServo.setPosition(currentSwivelPosition);
//    }

    public void raise(){
        currentArmPosition += 0.001;
        if(currentArmPosition > 1.0) currentArmPosition = 1.0;
        cappingServo.setPosition(currentArmPosition);
    }

    public void lower(){
        currentArmPosition -= 0.001;
        if(currentArmPosition < 0.0) currentArmPosition = 0.0;
        cappingServo.setPosition(currentArmPosition);
    }

}
