package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ScoringArm extends SubsystemBase {

    private final Telemetry telemetry;
    private final Servo armServo;

    //Position for in the robot and loading
    public static final double loadingPosition = 0.1;

    //Position for normal scoring
    public static final double scoringPosition = 1.0;


    public ScoringArm(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public ScoringArm(HardwareMap hardwareMap, Telemetry telemetry) {
        armServo = hardwareMap.get(Servo.class, "scoringArmServo");
        loadingPosition();
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (telemetry != null) {
            if (armServo.getPosition() != loadingPosition ||
                    armServo.getPosition() != scoringPosition) {
                telemetry.addLine("Scoring Arm Position" + String.format("%.4f", armServo.getPosition()));
            } else if (armServo.getPosition() == loadingPosition) {
                telemetry.addLine(loadingPosition + "(Loading Position)");
            } else {
                telemetry.addLine(scoringPosition + "(Scoring Position)");
            }
        }
    }

    /**
     * Raises the scoring arm by a small increment if the arm is within the limits.
     */
    public void raise() {
        double nextPosition = armServo.getPosition() + 0.005;
        if (nextPosition <= scoringPosition) {
            armServo.setPosition(nextPosition);
        }
    }

    /**
     * Lowers the scoring arm by a small increment if the arm is within the limits.
     */
    public void lower() {
        double nextPosition = armServo.getPosition() - 0.005;
        if (nextPosition >= loadingPosition) {
            armServo.setPosition(nextPosition);
        }
    }

    public double getPosition(){
        return armServo.getPosition();
    }

    public void setPosition(double position){
        armServo.setPosition(position);
    }

    /**
     * Moves the scoring arm to the lowered (loading) position.
     */
    public void loadingPosition() {
        armServo.setPosition(loadingPosition);
    }

    /**
     * Moves the scoring arm to the raised (scoring) position.
     */
    public void scoringPosition() {
        armServo.setPosition(scoringPosition);
    }
}
