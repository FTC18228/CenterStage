package org.firstinspires.ftc.teamcode.auto.opmode;

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
import org.firstinspires.ftc.teamcode.subsystem.Vision.CSVisionProcessor;
import org.firstinspires.ftc.teamcode.subsystem.Vision.VisionSubSystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class BasicRedBackPark extends AutoOpBase {

    private BotBuildersMecanumDrive drive;
    private DriveSubsystem driveSubsystem;
    private IntakeSubSystem intakeSubsystem;

    private VisionSubSystem visionSubSystem;

    private LinearSlideSubSystem linearSlideSubsystem;

    private DroneSubSystem droneSubSystem;

    private GamepadEx gamepadEx1;
    private TrajectorySequenceFollowerCommand centerMoveForwardFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand leftMoveBackFollower;
    private TrajectorySequenceFollowerCommand moveOffBackLeftFollower;

    private TrajectorySequenceFollowerCommand leftMoveParkFollower;
    private TrajectorySequenceFollowerCommand rightFollower;

    private TrajectorySequenceFollowerCommand centerMoveBackFollower;

    private TrajectorySequenceFollowerCommand centerMoveOffWallFollower;

    private TrajectorySequenceFollowerCommand rightMoveBackFollower;

    private TrajectorySequenceFollowerCommand rightMoveParkFollower;

    private TrajectorySequenceFollowerCommand centerMoveToWallFollower;

    private TrajectorySequenceFollowerCommand centerMoveParkFollower;
    private TrajectorySequenceFollowerCommand moveOffBackRightFollower;


    @Override
    public void initialize() {

        gamepadEx1 = new GamepadEx(gamepad1);
        drive = new BotBuildersMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(drive, gamepadEx1, telemetry);

        intakeSubsystem = new IntakeSubSystem(hardwareMap);

        visionSubSystem = new VisionSubSystem(hardwareMap, telemetry);

        droneSubSystem = new DroneSubSystem(hardwareMap);

        linearSlideSubsystem = new LinearSlideSubSystem(hardwareMap);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(56, -34, Math.toRadians(180));

        drive.setPoseEstimate(startingPosition);

        //Define the movements of the robot that we need.
        TrajectorySequence centerMoveForward = drive.trajectorySequenceBuilder(startingPosition)
                .forward(30)
               // .back(25)
                .build();

        TrajectorySequence centerMoveBack = drive.trajectorySequenceBuilder(centerMoveForward.end())
                .back(25)
                .build();

        TrajectorySequence centerMoveToSide = drive.trajectorySequenceBuilder(centerMoveBack.end())
                //moves to center then to back of field, then moves to center of backdrop and delivers yellow pixel
                .lineToLinearHeading(new Pose2d(-8,-45,Math.toRadians(280)))
                .lineToLinearHeading(new Pose2d(-15, 65, Math.toRadians(277)))
                .lineToLinearHeading(new Pose2d(5, 68, Math.toRadians(277)))
                .build();

        TrajectorySequence centerMoveAwayFromWall = drive.trajectorySequenceBuilder(centerMoveToSide.end())
                //moves away from the backdrop giving room to close outtake
                .lineToLinearHeading(new Pose2d(5,62,Math.toRadians(277)))
                .lineToLinearHeading(new Pose2d(-18,62,Math.toRadians(277)))
                .build();

        TrajectorySequence centerMovePark = drive.trajectorySequenceBuilder(centerMoveAwayFromWall.end())
                //parks on the right side of backdrop and rotates for initialisation.
                .lineToLinearHeading(new Pose2d(-18,70,Math.toRadians(277)))
                .turn(Math.toRadians(-110))
                .build();


        TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(28, -30, Math.toRadians(223)))
                .lineToLinearHeading(new Pose2d(40, -30, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(3, -30, Math.toRadians(180)))
                //.turn(Math.toRadians(107))
                .build();

        TrajectorySequence leftMoveBack = drive.trajectorySequenceBuilder(moveLeft.end())
                .lineToLinearHeading(new Pose2d(-19,65,Math.toRadians(277)))
                .lineToLinearHeading(new Pose2d(3, 70.5,Math.toRadians(277)))
                .build();

        TrajectorySequence leftMoveAwayFromWall = drive.trajectorySequenceBuilder(leftMoveBack.end())
                //moves away from the backdrop giving room to close outtake
                .lineToLinearHeading(new Pose2d(3,66,Math.toRadians(277)))
                .lineToLinearHeading(new Pose2d(-19,66,Math.toRadians(277)))
                .build();

        TrajectorySequence leftMovePark = drive.trajectorySequenceBuilder(leftMoveAwayFromWall.end())
                .lineToLinearHeading(new Pose2d(-17, 74,Math.toRadians(277)))
                .turn(Math.toRadians(-107))
                .build();

        TrajectorySequence moveRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(29, -33.5, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(34,-45,Math.toRadians(160)))
                .lineToLinearHeading(new Pose2d(-6,-35,Math.toRadians(277)))
                //.lineToLinearHeading(new Pose2d(-19,30,Math.toRadians(277)))
                .build();


        TrajectorySequence rightMoveBack = drive.trajectorySequenceBuilder(moveRight.end())
                .lineToLinearHeading(new Pose2d(-19,65,Math.toRadians(277)))
                .lineToLinearHeading(new Pose2d(12, 70.5,Math.toRadians(277)))
                .build();

        TrajectorySequence rightMoveAwayFromWall = drive.trajectorySequenceBuilder(rightMoveBack.end())
                //moves away from the backdrop giving room to close outtake
                .lineToLinearHeading(new Pose2d(14,66,Math.toRadians(277)))
                .lineToLinearHeading(new Pose2d(-19,66,Math.toRadians(277)))
                .build();

        TrajectorySequence rightMovePark = drive.trajectorySequenceBuilder(rightMoveAwayFromWall.end())
                .lineToLinearHeading(new Pose2d(-17.5, 74,Math.toRadians(277)))
                .turn(Math.toRadians(-110))
                .build();

        leftFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveLeft);
        leftMoveBackFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, leftMoveBack);
        moveOffBackLeftFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, leftMoveAwayFromWall);
        leftMoveParkFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, leftMovePark);

        centerMoveForwardFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveForward);
        centerMoveBackFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveBack);
        centerMoveToWallFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveToSide);
        centerMoveOffWallFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveAwayFromWall);
        centerMoveParkFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMovePark);

        rightFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveRight);
        rightMoveBackFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, rightMoveBack);
        moveOffBackRightFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, rightMoveAwayFromWall);
        rightMoveParkFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, rightMovePark);




        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(
                                        new WaitCommand(5000),
                                        new Disable(intakeSubsystem),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                    new InstantCommand(()->{
                                                        telemetry.addData("Running", "Left");
                                                    }),
                                                    leftFollower,
                                                    leftMoveBackFollower,
                                                    new AutoSlideExtend(linearSlideSubsystem),
                                                    new FlipDeposit(linearSlideSubsystem),
                                                    new WaitCommand(1500),
                                                    new OpenGate(linearSlideSubsystem),
                                                    new WaitCommand(1000),
                                                    moveOffBackLeftFollower,
                                                    new WaitCommand(500),
                                                    new CloseGate(linearSlideSubsystem),
                                                    new RetractDeposit(linearSlideSubsystem),
                                                    new SlideCompress(linearSlideSubsystem),
                                                    leftMoveParkFollower
                                                ),
                                                new ConditionalCommand(
                                                        new SequentialCommandGroup(
                                                            rightFollower,
                                                            rightMoveBackFollower,
                                                            new AutoSlideExtend(linearSlideSubsystem),
                                                            new FlipDeposit(linearSlideSubsystem),
                                                            new WaitCommand(1500),
                                                            new OpenGate(linearSlideSubsystem),
                                                            new WaitCommand(1000),
                                                            moveOffBackRightFollower,
                                                            new WaitCommand(500),
                                                            new CloseGate(linearSlideSubsystem),
                                                            new RetractDeposit(linearSlideSubsystem),
                                                            new SlideCompress(linearSlideSubsystem),
                                                            rightMoveParkFollower
                                                        ),
                                                        //doing the forward paths
                                                        new SequentialCommandGroup(
                                                                centerMoveForwardFollower,
                                                                centerMoveBackFollower,
                                                                centerMoveToWallFollower,
                                                                new AutoSlideExtend(linearSlideSubsystem),
                                                                new FlipDeposit(linearSlideSubsystem),
                                                                new WaitCommand(1500),
                                                                new OpenGate(linearSlideSubsystem),
                                                                new WaitCommand(500),
                                                                centerMoveOffWallFollower,
                                                                new CloseGate(linearSlideSubsystem),
                                                                new RetractDeposit(linearSlideSubsystem),
                                                                new SlideCompress(linearSlideSubsystem),
                                                                centerMoveParkFollower
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