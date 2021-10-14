package org.firstinspires.ftc.teamcode.Auto;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class AutoBase extends LinearOpMode {
  
  
  SampleMecanumDrive drive;
  
  Pose2d startPose;
  
  AutoBase(){
      this(new Pose2d(0, 0, Math.toRadians(0)));
  }
  
  AutoBase(Pose2d startPose){
    drive = new SampleMecanumDrive(hardwareMap);
    this.startPose = startPose;
  }
  
  
  
}
