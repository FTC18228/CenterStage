package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "BotBuilders")
public class RedTruss extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecanumDrive mecanumDrive = new BotBuildersMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(62, -34, Math.toRadians(180));
        mecanumDrive.setPoseEstimate(startPose);

        TrajectorySequence trajectorySequence = mecanumDrive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .strafeRight(40)
                .lineToLinearHeading(new Pose2d(34, 48, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, -58, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(34, 48, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, -58, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(34, 48, Math.toRadians(90)))
                .build();

        waitForStart();

        mecanumDrive.followTrajectorySequence(trajectorySequence);
    }
}
