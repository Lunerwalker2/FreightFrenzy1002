package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp (name="Duck Tester", group = "TeleOp")
public class DuckTester extends LinearOpMode {

    public DcMotor  carouselWheel = null;

    @Override
    public void runOpmode() {
        carouselWheel  = hardwareMap.get(DcMotor.class, "carouselWheel");

        
    }



}
