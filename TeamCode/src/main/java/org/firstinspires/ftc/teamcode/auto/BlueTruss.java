package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "BotBuilders")
public class BlueTruss extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-62, -34, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-62, -34, 0))
                .forward(26)
                .strafeLeft(40)
                .lineToLinearHeading(new Pose2d(-34, 48, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36, 0, Math.toRadians(270)))
                .forward(58)
                .build();

        waitForStart();

        drive.followTrajectorySequence(traj);
    }
}
