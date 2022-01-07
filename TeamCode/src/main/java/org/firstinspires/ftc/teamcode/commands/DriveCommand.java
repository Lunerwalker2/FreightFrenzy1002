package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveCommand extends CommandBase {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DriveCommand(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public DriveCommand(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
}
