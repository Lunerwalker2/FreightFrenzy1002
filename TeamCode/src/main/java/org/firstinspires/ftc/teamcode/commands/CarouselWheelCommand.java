package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;

public class CarouselWheelCommand extends CommandBase {


    private CarouselWheel carouselWheel;
    private boolean leftSide;

    /**
     * True if left side, false if right side
     */
    public CarouselWheelCommand(CarouselWheel carouselWheel, boolean leftSide){
        this.carouselWheel = carouselWheel;
        this.leftSide = leftSide;
        addRequirements(carouselWheel);
    }



    @Override
    public void initialize(){
        if(leftSide) carouselWheel.leftForward();
        else carouselWheel.rightForward();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        carouselWheel.leftStop();
        carouselWheel.rightStop();
    }
}
