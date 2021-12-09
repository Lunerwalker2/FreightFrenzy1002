package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;

public class CarouselWheelCommand extends CommandBase {


    private final CarouselWheel carouselWheel;
    private final boolean leftSide;
    private final boolean forward;

    /**
     * True if left side, false if right side
     */
    public CarouselWheelCommand(CarouselWheel carouselWheel, boolean leftSide){
        this(carouselWheel, leftSide, true);
    }

    public CarouselWheelCommand(CarouselWheel carouselWheel, boolean leftSide, boolean forward){
        this.carouselWheel = carouselWheel;
        this.leftSide = leftSide;
        this.forward = forward;
        addRequirements(carouselWheel);
    }



    @Override
    public void initialize(){
        if(leftSide){
            if(forward) carouselWheel.leftForward();
            else carouselWheel.leftBackward();
        }
        else {
            carouselWheel.rightForward();
            carouselWheel.rightBackward();
        }
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
