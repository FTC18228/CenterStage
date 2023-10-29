package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoOpBase;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Disable;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.CloseGate;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.FlipDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.OpenGate;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.RetractDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.SlideCompress;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.SlideExtend;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "BotBuilders")
public class BlueTrussPark extends AutoOpBase {
    BotBuildersMecanumDrive mecanumDrive;
    GamepadEx gamepadEx1;
    DriveSubsystem driveSubsystem;
    LinearSlideSubSystem slideSubSystem;
    IntakeSubSystem intakeSubSystem;

    @Override
    public void initialize() {
        mecanumDrive = new BotBuildersMecanumDrive(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        driveSubsystem = new DriveSubsystem(mecanumDrive, gamepadEx1, telemetry);
        slideSubSystem = new LinearSlideSubSystem(hardwareMap);
        intakeSubSystem = new IntakeSubSystem(hardwareMap);

        Pose2d startPose = new Pose2d(-62, -34, 0);
        mecanumDrive.setPoseEstimate(startPose);

        TrajectorySequence deliverPurplePixelSequence = mecanumDrive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -34))
                .build();

        TrajectorySequence deliverYellowPixelSequence = mecanumDrive.trajectorySequenceBuilder(deliverPurplePixelSequence.end())
                .lineTo(new Vector2d(-36, 6))
                .lineToLinearHeading(new Pose2d(-34, 48, Math.toRadians(90)))
                .build();

        TrajectorySequence parkSequence = mecanumDrive.trajectorySequenceBuilder(deliverYellowPixelSequence.end())
                .lineTo(new Vector2d(-34, 41))
                .splineToConstantHeading(new Vector2d(-58, 58), Math.toRadians(90))
                .build();

        TrajectorySequenceFollowerCommand deliverPurplePixel = new TrajectorySequenceFollowerCommand(driveSubsystem, deliverPurplePixelSequence);
        TrajectorySequenceFollowerCommand deliverYellowPixel = new TrajectorySequenceFollowerCommand(driveSubsystem, deliverYellowPixelSequence);
        TrajectorySequenceFollowerCommand park = new TrajectorySequenceFollowerCommand(driveSubsystem, parkSequence);

        slideSubSystem.ResetSlideEncoders();

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new Disable(intakeSubSystem),
                                        new FlipDeposit(slideSubSystem),
                                        deliverPurplePixel
                                ),
                                new OpenGate(slideSubSystem),
                                new ParallelCommandGroup(
                                        new CloseGate(slideSubSystem),
                                        new RetractDeposit(slideSubSystem)
                                ),
                                new ParallelCommandGroup(
                                        new SlideExtend(slideSubSystem),
                                        new FlipDeposit(slideSubSystem),
                                        deliverYellowPixel
                                ),
                                new OpenGate(slideSubSystem),
                                new ParallelCommandGroup(
                                        new CloseGate(slideSubSystem),
                                        new RetractDeposit(slideSubSystem),
                                        new SlideCompress(slideSubSystem),
                                        park
                                )
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
