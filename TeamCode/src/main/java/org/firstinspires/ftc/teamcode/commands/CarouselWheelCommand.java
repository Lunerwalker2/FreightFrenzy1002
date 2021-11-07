package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;

public class CarouselWheelCommand extends CommandBase {


    private CarouselWheel carouselWheel;
    private boolean leftSide;
    private boolean withTimeout;
    private long timeOutMs = 0;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /**
     * True if left side, false if right side
     */
    public CarouselWheelCommand(CarouselWheel carouselWheel, boolean leftSide, long timeOutMs){
        this.carouselWheel = carouselWheel;
        this.leftSide = leftSide;
        withTimeout = timeOutMs != 0;
        this.timeOutMs = timeOutMs;
        addRequirements(carouselWheel);
    }


    public CarouselWheelCommand(CarouselWheel carouselWheel, boolean leftSide){
        this(carouselWheel, leftSide, 0);
    }



    @Override
    public void initialize(){
        timer.reset();
        if(leftSide) carouselWheel.leftForward();
        else carouselWheel.rightForward();
    }

    @Override
    public boolean isFinished() {
        if(withTimeout){
            return timer.milliseconds() > timeOutMs;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted){
        carouselWheel.leftStop();
        carouselWheel.rightStop();
    }
}
