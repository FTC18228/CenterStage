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

    private GamepadEx gamepadEx1;
    private TrajectorySequenceFollowerCommand centerMoveForwardFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand rightFollower;

    private TrajectorySequenceFollowerCommand centerMoveBackFollower;

    private TrajectorySequenceFollowerCommand rightMoveBackFollower;

    private TrajectorySequenceFollowerCommand rightMoveParkFollower;

    private TrajectorySequenceFollowerCommand centerMoveToWallFollower;

    private TrajectorySequenceFollowerCommand centerMoveParkFollower;


    @Override
    public void initialize() {

        gamepadEx1 = new GamepadEx(gamepad1);
        drive = new BotBuildersMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(drive, gamepadEx1, telemetry);

        intakeSubsystem = new IntakeSubSystem(hardwareMap);

        visionSubSystem = new VisionSubSystem(hardwareMap, telemetry);


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
                .lineToLinearHeading(new Pose2d(-15, 65, Math.toRadians(275)))
                .lineToLinearHeading(new Pose2d(6, 67, Math.toRadians(275)))
                .build();

        TrajectorySequence centerMovePark = drive.trajectorySequenceBuilder(centerMoveToSide.end())
                //parks on the right side of backdrop and rotates for initialisation
                .lineToLinearHeading(new Pose2d(-18,65,Math.toRadians(275)))
                .lineToLinearHeading(new Pose2d(-18,70,Math.toRadians(275)))

                .build();


        TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(28, -30, Math.toRadians(223)))
                .lineToLinearHeading(new Pose2d(40, -30, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(3, -30, Math.toRadians(180)))
                .turn(Math.toRadians(107))
                .build();

        TrajectorySequence moveRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(29, -33.5, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(34,-45,Math.toRadians(160)))
                .lineToLinearHeading(new Pose2d(-6,-35,Math.toRadians(277)))
                .lineToLinearHeading(new Pose2d(-19,30,Math.toRadians(277)))
                .build();


        TrajectorySequence rightMoveBack = drive.trajectorySequenceBuilder(moveRight.end())
                .lineToLinearHeading(new Pose2d(-19,65,Math.toRadians(280)))
                .lineToLinearHeading(new Pose2d(12, 69,Math.toRadians(277)))

                .build();

        TrajectorySequence rightMovePark = drive.trajectorySequenceBuilder(rightMoveBack.end())
                .lineToLinearHeading(new Pose2d(-19,65,Math.toRadians(277)))
                .lineToLinearHeading(new Pose2d(-19, 71,Math.toRadians(277)))
                .build();

        leftFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveLeft);

        centerMoveForwardFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveForward);
        centerMoveBackFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveBack);
        centerMoveToWallFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveToSide);
        centerMoveParkFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMovePark);

        rightFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveRight);
        rightMoveBackFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, rightMoveBack);
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
                                                    leftFollower
                                                ),
                                                new ConditionalCommand(
                                                        new SequentialCommandGroup(
                                                            rightFollower,
                                                            rightMoveBackFollower,
                                                            new AutoSlideExtend(linearSlideSubsystem),
                                                            new FlipDeposit(linearSlideSubsystem),
                                                            new WaitCommand(2000),
                                                            new OpenGate(linearSlideSubsystem),
                                                            new WaitCommand(1000),
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
                                                                new WaitCommand(2000),
                                                                new OpenGate(linearSlideSubsystem),
                                                                new WaitCommand(1000),
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