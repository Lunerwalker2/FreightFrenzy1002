package org.firstinspires.ftc.teamcode.subsystems.old;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw extends SubsystemBase {


    private final Servo clawServo;
    private Telemetry telemetry;
    private boolean isClosed = false;

    public Claw(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public Claw(HardwareMap hardwareMap, Telemetry telemetry){
        super(); //registers this subsystem
        clawServo = hardwareMap.get(Servo.class, "claw");
        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        if(telemetry != null){
            telemetry.addData("Servo closed", isClosed);
        }
    }

    public void openClaw(){
        clawServo.setPosition(0.2);
        isClosed = false;
    }

    public void closeClaw(){
        clawServo.setPosition(0.65);
        isClosed = true;
    }


}
