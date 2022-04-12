package org.firstinspires.ftc.teamcode.subsystems.interfaces;

public interface IntakeSide {


    void intake();

    void intakeUp();

    void intakeDown();

    void outtake();

//    boolean freightDetected();

    void stop();

    int currentPosition();

    void intakePower(double power);

}
