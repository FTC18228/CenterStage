package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoOpBase;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Disable;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.ResetSlideEncoders;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "BotBuilders")
public class BlueBackdrop extends AutoOpBase {
    BotBuildersMecanumDrive mecanumDrive;
    GamepadEx gamepadEx1;
    DriveSubsystem driveSubsystem;
    LinearSlideSubSystem slideSubSystem;
    IntakeSubSystem intakeSubSystem;
    TrajectorySequenceFollowerCommand parkFollower;

    @Override
    public void initialize() {
        mecanumDrive = new BotBuildersMecanumDrive(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        driveSubsystem = new DriveSubsystem(mecanumDrive, gamepadEx1, telemetry);
        slideSubSystem = new LinearSlideSubSystem(hardwareMap);
        intakeSubSystem = new IntakeSubSystem(hardwareMap);

        Pose2d startPose = new Pose2d(-62, 11, 0);
        mecanumDrive.setPoseEstimate(startPose);

        TrajectorySequence trajectorySequence = mecanumDrive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-42, 11))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()-> {slideSubSystem.FlipDeposit();})
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(0, ()-> {slideSubSystem.OpenGate();})
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(0, ()-> {slideSubSystem.RetractDeposit();})
                .splineTo(new Vector2d(-34, 48), Math.toRadians(90))
                .lineTo(new Vector2d(-34, 28))
                .splineTo(new Vector2d(-58, 17), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58, -30), Math.toRadians(90))
                .lineTo(new Vector2d(-33, -58))
                .lineToLinearHeading(new Pose2d(-34, 48, Math.toRadians(90)))
                .lineTo(new Vector2d(-34, 43))
                .splineToConstantHeading(new Vector2d(-58, 58), Math.toRadians(90))
                .build();

        parkFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, trajectorySequence);

        slideSubSystem.ResetSlideEncoders();

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new ParallelCommandGroup(
                                new Disable(intakeSubSystem),
                                parkFollower
                        )
                ));
    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart() {

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
