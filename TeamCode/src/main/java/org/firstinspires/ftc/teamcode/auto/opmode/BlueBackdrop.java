package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto.AutoOpBase;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Disable;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.CloseGate;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.FlipDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.OpenGate;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.RetractDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.SlideCompress;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.SlideExtend;
import org.firstinspires.ftc.teamcode.subsystem.Vision.VisionSubSystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "BotBuilders")
@Disabled
public class BlueBackdrop extends AutoOpBase {
    BotBuildersMecanumDrive mecanumDrive;
    GamepadEx gamepadEx1;
    DriveSubsystem driveSubsystem;
    LinearSlideSubSystem slideSubSystem;
    IntakeSubSystem intakeSubSystem;
    VisionSubSystem visionSubSystem;

    @Override
    public void initialize() {
        mecanumDrive = new BotBuildersMecanumDrive(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        driveSubsystem = new DriveSubsystem(mecanumDrive, gamepadEx1, telemetry);
        slideSubSystem = new LinearSlideSubSystem(hardwareMap);
        intakeSubSystem = new IntakeSubSystem(hardwareMap);
        visionSubSystem = new VisionSubSystem(hardwareMap, telemetry);

        Pose2d startPose = new Pose2d(-62, 11, 0);
        mecanumDrive.setPoseEstimate(startPose);

        TrajectorySequence deliverPurplePixelSequence = mecanumDrive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-42, 11))
                .build();

        TrajectorySequence deliverYellowPixelSequence = mecanumDrive.trajectorySequenceBuilder(deliverPurplePixelSequence.end())
                .splineTo(new Vector2d(-34, 48), Math.toRadians(90))
                .build();

        TrajectorySequence moveToStackSequence = mecanumDrive.trajectorySequenceBuilder(deliverYellowPixelSequence.end())
                .lineTo(new Vector2d(-34, 28))
                .splineTo(new Vector2d(-58, 17), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58, -30), Math.toRadians(90))
                .lineTo(new Vector2d(-33, -58))
                .build();

        TrajectorySequence deliverWhitePixelSequence = mecanumDrive.trajectorySequenceBuilder(moveToStackSequence.end())
                .lineToLinearHeading(new Pose2d(-34, 48, Math.toRadians(90)))
                .build();

        TrajectorySequence parkSequence = mecanumDrive.trajectorySequenceBuilder(deliverWhitePixelSequence.end())
                .lineTo(new Vector2d(-34, 43))
                .splineToConstantHeading(new Vector2d(-58, 58), Math.toRadians(90))
                .build();

        TrajectorySequenceFollowerCommand deliverPurplePixel = new TrajectorySequenceFollowerCommand(driveSubsystem, deliverPurplePixelSequence);
        TrajectorySequenceFollowerCommand deliverYellowPixel = new TrajectorySequenceFollowerCommand(driveSubsystem, deliverYellowPixelSequence);
        TrajectorySequenceFollowerCommand moveToStack = new TrajectorySequenceFollowerCommand(driveSubsystem, moveToStackSequence);
        TrajectorySequenceFollowerCommand deliverWhitePixel = new TrajectorySequenceFollowerCommand(driveSubsystem, deliverWhitePixelSequence);
        TrajectorySequenceFollowerCommand park = new TrajectorySequenceFollowerCommand(driveSubsystem, parkSequence);

        slideSubSystem.ResetSlideEncoders();

        //region CommandScheduler
        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                       // new Disable(intakeSubSystem),
                                        new FlipDeposit(slideSubSystem),
                                        deliverPurplePixel
                                ),
                                new OpenGate(slideSubSystem),
                                new ParallelCommandGroup(
                                        //new CloseGate(slideSubSystem),
                                        new RetractDeposit(slideSubSystem)
                                ),
                                new ParallelCommandGroup(
                                        //new SlideExtend(slideSubSystem),
                                        new FlipDeposit(slideSubSystem),
                                        deliverYellowPixel
                                ),
                                new OpenGate(slideSubSystem),
                                new ParallelCommandGroup(
                                        //new CloseGate(slideSubSystem),
                                        new RetractDeposit(slideSubSystem),
                                        //new SlideCompress(slideSubSystem),
                                        moveToStack
                                ),
                                new Intake(intakeSubSystem),
                                new ParallelCommandGroup(
                                        //new SlideExtend(slideSubSystem),
                                        new FlipDeposit(slideSubSystem),
                                        deliverWhitePixel
                                ),
                                new OpenGate(slideSubSystem),
                                new ParallelCommandGroup(
                                        //new CloseGate(slideSubSystem),
                                        new RetractDeposit(slideSubSystem),
                                        //new SlideCompress(slideSubSystem),
                                        park
                                )
                        )
                ));
        //endregion
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
