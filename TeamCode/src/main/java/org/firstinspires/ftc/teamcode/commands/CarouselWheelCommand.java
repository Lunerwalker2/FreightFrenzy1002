package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;

public class CarouselWheelCommand extends CommandBase {


    private final CarouselWheel carouselWheel;
    private final boolean forward;

    /**
     * True if left side, false if right side
     */
    public CarouselWheelCommand(CarouselWheel carouselWheel){
        this(carouselWheel, true);
    }

    public CarouselWheelCommand(CarouselWheel carouselWheel, boolean forward){
        this.carouselWheel = carouselWheel;
        this.forward = forward;
        addRequirements(carouselWheel);
    }



    @Override
    public void initialize(){
        carouselWheel.setWheelPower(0.6);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        carouselWheel.setWheelPower(0);
    }
}
