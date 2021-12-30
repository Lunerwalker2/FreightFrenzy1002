package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;

@TeleOp
public class ClawTester extends CommandOpMode {

    private Bucket bucket;

    private boolean prevPressed = false;
    private boolean prevDown = false;

    @Override
    public void initialize() {

        bucket = new Bucket(hardwareMap);

        new GamepadEx(gamepad1).getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(bucket::dump, bucket::load);

    }
}
