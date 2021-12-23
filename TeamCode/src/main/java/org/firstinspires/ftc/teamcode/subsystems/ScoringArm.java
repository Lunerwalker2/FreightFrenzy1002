package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ScoringArm extends SubsystemBase {

    private final Telemetry telemetry;
    private final Servo armServo;

    //Position for in the robot and loading
    private final double loadingPosition = 0.0;

    //Maximum position for outside the robot and for scoring (by default) in teleop
    private final double maxOuterPosition = 0.7;

    //Position for normal scoring
    private final double scoringPosition = 0.4;


    public ScoringArm(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public ScoringArm(HardwareMap hardwareMap, Telemetry telemetry){
        armServo = hardwareMap.get(Servo.class, "scoringArmServo");
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if(telemetry != null){
            telemetry.addData("Scoring Arm Position", () -> {
                if(armServo.getPosition() != loadingPosition ||
                armServo.getPosition() != maxOuterPosition ||
                armServo.getPosition() != scoringPosition){
                    return String.format("%.4f", armServo.getPosition());
                } else if(armServo.getPosition() == loadingPosition){
                    return loadingPosition + "(Loading Position)";
                } else if(armServo.getPosition() == scoringPosition){
                    return scoringPosition + "(Scoring Position)";
                } else {
                    return maxOuterPosition + "(Outer Position)";
                }
            });
        }
    }

    /**
     * Raises the scoring arm by a small increment if the arm is within the limits.
     */
    public void raise(){
        double nextPosition = armServo.getPosition() + 0.005;
        if(nextPosition <= maxOuterPosition){
            armServo.setPosition(nextPosition);
        }
    }

    /**
     * Lowers the scoring arm by a small increment if the arm is within the limits.
     */
    public void lower(){
        double nextPosition = armServo.getPosition() - 0.005;
        if(nextPosition >= maxOuterPosition){
            armServo.setPosition(nextPosition);
        }
    }

    /**
     * Moves the scoring arm to the lowered (loading) position.
     */
    public void loadingPosition(){
        armServo.setPosition(loadingPosition);
    }

    /**
     * Moves the scoring arm to the raised (scoring) position.
     */
    public void scoringPosition(){
        armServo.setPosition(scoringPosition);
    }

    /**
     * Moves the scoring arm to the maximum raised position.
     */
    public void outerPosition(){
        armServo.setPosition(maxOuterPosition);
    }
}
