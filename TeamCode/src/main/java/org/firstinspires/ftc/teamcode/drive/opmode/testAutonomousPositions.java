package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "testAutonomousPositions", group = "Linear OpMode")

public class testAutonomousPositions extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        //initialization code

        Pose2d startPose = new Pose2d(-60, 13, 0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory scoringPositionTraj = drive.trajectoryBuilder(new Pose2d(-60,13,0))
                .forward(27)
                .build();

        Trajectory spikeRightTraj = drive.trajectoryBuilder(scoringPositionTraj.end())
                .splineTo(new Vector2d(-35, 0.5), 0)
                .build();

        Trajectory spikeLeftTraj = drive.trajectoryBuilder(scoringPositionTraj.end())
                .splineTo(new Vector2d(-35, 22.6), 0)
                .build();

        Trajectory spikeCenterTraj = drive.trajectoryBuilder(scoringPositionTraj.end())
                .splineTo(new Vector2d(-25, 12), 0)
                .build();

        Trajectory preparkFromCenterTraj = drive.trajectoryBuilder(spikeCenterTraj.end())
                .splineTo(new Vector2d(-55, 40), 0)
                .build();

        Trajectory preparkFromLeftTraj = drive.trajectoryBuilder(spikeLeftTraj.end())
                .splineTo(new Vector2d(-55, 40), 0)
                .build();

        Trajectory preparkFromRightTraj = drive.trajectoryBuilder(spikeRightTraj.end())
                .splineTo(new Vector2d(-55, 40), 0)
                .build();

        Trajectory finalPark = drive.trajectoryBuilder(new Pose2d(-55, 40))
                .splineTo(new Vector2d(-55, 57), 0)
                .build();



        waitForStart();

        if (isStopRequested())
            return;

        drive.followTrajectory(scoringPositionTraj);
        drive.followTrajectory(spikeCenterTraj);
        drive.followTrajectory(preparkFromCenterTraj);
        drive.followTrajectory(finalPark);
    } // ends runOpMode()


} // ends linearOpMode
