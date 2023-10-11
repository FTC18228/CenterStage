package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "BotBuilders")
public class BlueBackdropPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(-62, 11, 0));

        TrajectorySequence trajectorySequence = mecanumDrive.trajectorySequenceBuilder(new Pose2d(-62, 11, 0))
                .lineTo(new Vector2d(-33, 11))
                .splineTo(new Vector2d(-34, 48), Math.toRadians(90))
                .lineTo(new Vector2d(-34, 41))
                .splineToConstantHeading(new Vector2d(-58, 58), Math.toRadians(90))
                .build();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            mecanumDrive.followTrajectorySequence(trajectorySequence);
        }
    }
}
