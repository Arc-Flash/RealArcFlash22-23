package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.outoftheboxrobotics.photoncore.PhotonCore;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class Blue1Auto extends LinearOpMode {
    @Override
    public void runOpMode(){
        PhotonCore.enable();
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory strafeRight = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeRight(26)
                .build();

        Trajectory forward = drivetrain.trajectoryBuilder(strafeRight.end())
                .forward(50)
                .build();

        TrajectorySequence strafeleft = drivetrain.trajectorySequenceBuilder(forward.end())
                .strafeLeft(42)
                .turn(Math.toRadians(90))
                .build();

   TrajectorySequence oblique1 = drivetrain.trajectorySequenceBuilder(strafeleft.end())
                .turn(Math.toRadians(-33.8))
                .strafeLeft(10)
                .turn(Math.toRadians(33.8))
                .strafeRight(10)
                .back(3)
                .turn(Math.toRadians(-33.8))
                .strafeLeft(8)
                .back(3)
                .turn(Math.toRadians(33.8))
                .strafeRight(10)
                .build();

//        TrajectorySequence oblique2 = drivetrain.trajectorySequenceBuilder(oblique1.end())
//                .turn(Math.toRadians(-33.8))
//                .strafeRight(5)
//                .build();











        drivetrain.followTrajectory(strafeRight);
        drivetrain.followTrajectory(forward);
        drivetrain.followTrajectorySequence(strafeleft);
        drivetrain.followTrajectorySequence(oblique1);

//








    }
}
