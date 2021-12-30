package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.ColorInt;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class SetHubLEDCommand extends InstantCommand {



    private final List<LynxModule> hubList;
    private final int colorInt;


    /**
     * Color integer is in form of AARRGGBB. Both hubs' LEDs will be changed to the new color.
     *
     * @param hardwareMap The hardware map for the opmode.
     * @param colorInt The integer representing the desired color
     */
    public SetHubLEDCommand(HardwareMap hardwareMap, @ColorInt int colorInt){
        hubList = hardwareMap.getAll(LynxModule.class);
        this.colorInt = colorInt;
    }

    @Override
    public void initialize() {
        super.initialize();
        hubList.forEach((hub) -> hub.setConstant(colorInt));
    }


}
