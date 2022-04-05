package org.firstinspires.ftc.teamcode.teleOp.oldTests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.OrientIntakeFlapCommand;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;

@Autonomous
public class OrientFlapTest extends CommandOpMode {


    private LeftIntake leftIntake;
    private RightIntake rightIntake;

    private final double kP = 0.03;
    private final double tolerance = 10;
    private final double ticksPerHalfRot = 28*5.2/2;
    private final double target = 0;
    private double error = 0;


    @Override
    public void initialize(){

//        leftIntake = new LeftIntake(hardwareMap);
        rightIntake = new RightIntake(hardwareMap);
    }

    @Override
    public void run(){
        super.run();

        error = target - (rightIntake.currentPosition() % ticksPerHalfRot);

        double output = kP * error;

        if(Math.abs(error) > tolerance) rightIntake.intakePower(output);
        else rightIntake.stop();

//        telemetry.addData("Left Intake Pos", leftIntake.currentPosition());
        telemetry.addData("ticks per half rot", ticksPerHalfRot);
        telemetry.addData("error", error);
        telemetry.addData("output", output);
        telemetry.addData("Right Intake Pos", rightIntake.currentPosition());

        telemetry.update();
    }
}
