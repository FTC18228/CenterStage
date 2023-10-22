package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "BotBuilders")
public class BlueTrussPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecanumDrive drive = new BotBuildersMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-62, -34, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(-62, -34, 0))
                .forward(26)
                .strafeLeft(40)
                .lineToLinearHeading(new Pose2d(-34, 48, Math.toRadians(90)))
                .back(7)
                .splineToConstantHeading(new Vector2d(-58, 58), Math.toRadians(90))
                .build();

        waitForStart();

        drive.followTrajectorySequence(trajectorySequence);
    }
}
