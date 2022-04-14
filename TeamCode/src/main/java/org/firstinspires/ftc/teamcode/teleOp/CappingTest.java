package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CappingMech;

@TeleOp
public class CappingTest extends CommandOpMode {


    private CappingMech cappingMech;




    @Override
    public void initialize(){
        cappingMech = new CappingMech(hardwareMap);


    }



    @Override
    public void run(){

        super.run();

        if(gamepad1.dpad_up) cappingMech.extend();
        else if(gamepad1.dpad_down) cappingMech.retract();
        else cappingMech.stop();


        if(gamepad2.dpad_left) cappingMech.incrementPosition();
        else if(gamepad1.dpad_right) cappingMech.decrementPosition();
    }
}
