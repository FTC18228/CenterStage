package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "BotBuilders")
public class RedBackdropPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(62, 11, Math.toRadians(180));
        mecanumDrive.setPoseEstimate(startPose);

        TrajectorySequence trajectorySequence = mecanumDrive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .splineTo(new Vector2d(34, 48), Math.toRadians(90))
                .back(9)
                .splineToConstantHeading(new Vector2d(58, 58), Math.toRadians(90))
                .build();

        waitForStart();

        mecanumDrive.followTrajectorySequence(trajectorySequence);
    }
}
