package org.firstinspires.ftc.teamcode.auto.opmode;

import android.transition.Slide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoOpBase;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystem.Drone.DroneSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Disable;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.AutoSlideExtend;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.CloseGate;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.FlipDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.OpenGate;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.RetractDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.SlideCompress;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.SlideExtend;
import org.firstinspires.ftc.teamcode.subsystem.Vision.CSVisionProcessor;
import org.firstinspires.ftc.teamcode.subsystem.Vision.VisionSubSystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class BasicRedFrontPark extends AutoOpBase {

    private BotBuildersMecanumDrive drive;
    private DriveSubsystem driveSubsystem;
    private IntakeSubSystem intakeSubsystem;

    private DroneSubSystem droneSubSystem;
    private VisionSubSystem visionSubSystem;

    private LinearSlideSubSystem linearSlideSubsystem;

    private GamepadEx gamepadEx1;
    private TrajectorySequenceFollowerCommand centerMoveForwardFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand rightFollower;

    private TrajectorySequenceFollowerCommand centerMoveToSideFollower;

    private TrajectorySequenceFollowerCommand leftMoveToSideFollower;

    private TrajectorySequenceFollowerCommand rightMoveToSideFollower;

    private TrajectorySequenceFollowerCommand moveOffBackCenterFollower;

    private TrajectorySequenceFollowerCommand moveOffBackLeftFollower;

    private TrajectorySequenceFollowerCommand moveOffBackRightFollower;

    @Override
    public void initialize() {

        gamepadEx1 = new GamepadEx(gamepad1);
        drive = new BotBuildersMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(drive, gamepadEx1, telemetry);

        intakeSubsystem = new IntakeSubSystem(hardwareMap);

        visionSubSystem = new VisionSubSystem(hardwareMap, telemetry);

        linearSlideSubsystem = new LinearSlideSubSystem(hardwareMap);

        droneSubSystem = new DroneSubSystem(hardwareMap);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(56, -34, Math.toRadians(180));

        drive.setPoseEstimate(startingPosition);

        //Define the movements of the robot that we need.
        TrajectorySequence centerMoveForward = drive.trajectorySequenceBuilder(startingPosition)
                //places purple pixel, then moves to backdrop and delivers yellow pixel
                .lineToLinearHeading(new Pose2d(26, -34, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36,-34,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(14,12.5 ,Math.toRadians(280)))
                .build();


        TrajectorySequence centerMoveAwayFromWall = drive.trajectorySequenceBuilder(centerMoveForward.end())
                //moves away from the backdrop giving room to close outtake
                .lineToLinearHeading(new Pose2d(14,5,Math.toRadians(280)))
                .build();


        TrajectorySequence centerMoveToSide = drive.trajectorySequenceBuilder(centerMoveAwayFromWall.end())
                //parks on the right side of backdrop and rotates for initialisation
                .lineToLinearHeading(new Pose2d(40,17,Math.toRadians(280)))
                .turn(Math.toRadians(-110))
                .build();


        TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(startingPosition)
                //places purple pixel, then moves to backdrop and delivers yellow pixel
                .lineToLinearHeading(new Pose2d(28, -37, Math.toRadians(225)))
                .lineToLinearHeading(new Pose2d(36,-34,Math.toRadians(225)))
                .lineToLinearHeading(new Pose2d(7,11.5,Math.toRadians(277)))
                .build();


        TrajectorySequence leftMoveAwayFromWall = drive.trajectorySequenceBuilder(moveLeft.end())
                //moves away from the backdrop giving room to close outtake
                .lineToLinearHeading(new Pose2d(40,0,Math.toRadians(277)))
                .build();


        TrajectorySequence leftMoveToSide = drive.trajectorySequenceBuilder(leftMoveAwayFromWall.end())
                //parks on the right side of backdrop and rotates for initialisation
                .lineToLinearHeading(new Pose2d(39,17,Math.toRadians(277)))
                .turn(Math.toRadians(-110))
                .build();


        TrajectorySequence moveRight = drive.trajectorySequenceBuilder(startingPosition)
                //places purple pixel, then moves to backdrop and delivers yellow pixel
                .lineToLinearHeading(new Pose2d(26.5, -42, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(40, -42, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(40,-20,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(23,13.5,Math.toRadians(277)))
                .build();


        TrajectorySequence rightMoveAwayFromWall = drive.trajectorySequenceBuilder(moveRight.end())
                //moves away from the backdrop giving room to close outtake
                .lineToLinearHeading(new Pose2d(40,-10,Math.toRadians(277)))
                .build();


        TrajectorySequence rightMoveToSide = drive.trajectorySequenceBuilder(rightMoveAwayFromWall.end())
                //parks on the right side of backdrop and rotates for initialisation
                .lineToLinearHeading(new Pose2d(40,17,Math.toRadians(277)))
                .turn(Math.toRadians(-110))
                .build();



        centerMoveForwardFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveForward);

        centerMoveToSideFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveToSide);

        leftMoveToSideFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, leftMoveToSide);

        leftFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveLeft);
        rightFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveRight);

        rightMoveToSideFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, rightMoveToSide);

        moveOffBackCenterFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveAwayFromWall);

        moveOffBackLeftFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, leftMoveAwayFromWall);

        moveOffBackRightFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, rightMoveAwayFromWall);
        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(
                                        new WaitCommand(0000),
                                        new Disable(intakeSubsystem),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                    new InstantCommand(()->{
                                                        telemetry.addData("Running", "Left");
                                                    }),
                                                    leftFollower,
                                                        new AutoSlideExtend(linearSlideSubsystem),
                                                        new FlipDeposit(linearSlideSubsystem),
                                                        new WaitCommand(1500),
                                                        new OpenGate(linearSlideSubsystem),
                                                        new WaitCommand(1000),
                                                        moveOffBackLeftFollower,
                                                        new SlideCompress(linearSlideSubsystem),
                                                        new WaitCommand(300),
                                                        new CloseGate(linearSlideSubsystem),
                                                        new RetractDeposit(linearSlideSubsystem),
                                                        leftMoveToSideFollower

                                                        ),
                                                new ConditionalCommand(
                                                        new SequentialCommandGroup(
                                                            rightFollower,
                                                                new AutoSlideExtend(linearSlideSubsystem),
                                                                new FlipDeposit(linearSlideSubsystem),
                                                                new WaitCommand(2000),
                                                                new OpenGate(linearSlideSubsystem),
                                                                new WaitCommand(2000),
                                                                moveOffBackRightFollower,
                                                                new SlideCompress(linearSlideSubsystem),
                                                                new WaitCommand(300),
                                                                new CloseGate(linearSlideSubsystem),
                                                                new RetractDeposit(linearSlideSubsystem),
                                                                rightMoveToSideFollower
                                                        ),
                                                        //doing the forward paths
                                                        new SequentialCommandGroup(
                                                                centerMoveForwardFollower,
                                                                new AutoSlideExtend(linearSlideSubsystem),
                                                                new FlipDeposit(linearSlideSubsystem),
                                                                new WaitCommand(2000),
                                                                new OpenGate(linearSlideSubsystem),
                                                                new WaitCommand(1000),
                                                                moveOffBackCenterFollower,
                                                                new SlideCompress(linearSlideSubsystem),
                                                                new WaitCommand(500),
                                                                new CloseGate(linearSlideSubsystem),
                                                                new RetractDeposit(linearSlideSubsystem),
                                                                centerMoveToSideFollower


                                                        ),
                                                        () -> {return visionSubSystem.getPosition() == CSVisionProcessor.StartingPosition.RIGHT;}
                                                ),
                                                () -> { return visionSubSystem.getPosition() == CSVisionProcessor.StartingPosition.LEFT;}

                                        ),


                                new InstantCommand(() ->{
                                    telemetry.addData("Position", visionSubSystem.getPosition());
                                    telemetry.update();
                                })

                        )



                ));

    }

    @Override
    public void run(){

        CommandScheduler.getInstance().run();

    }

    @Override
    public void preInit() {


    }

    @Override
    public void preStart(){

    }
}