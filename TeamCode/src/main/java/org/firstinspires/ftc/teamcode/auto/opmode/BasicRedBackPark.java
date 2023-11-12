package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.subsystem.Vision.CSVisionProcessor;
import org.firstinspires.ftc.teamcode.subsystem.Vision.VisionSubSystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class BasicRedBackPark extends AutoOpBase {

    private BotBuildersMecanumDrive drive;
    private DriveSubsystem driveSubsystem;
    private IntakeSubSystem intakeSubsystem;

    private VisionSubSystem visionSubSystem;
    private GamepadEx gamepadEx1;
    private TrajectorySequenceFollowerCommand centerMoveForwardFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand rightFollower;

    private TrajectorySequenceFollowerCommand centerMoveBackFollower;

    private TrajectorySequenceFollowerCommand centerMoveToWallFollower;

    @Override
    public void initialize() {

        gamepadEx1 = new GamepadEx(gamepad1);
        drive = new BotBuildersMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(drive, gamepadEx1, telemetry);

        intakeSubsystem = new IntakeSubSystem(hardwareMap);

        visionSubSystem = new VisionSubSystem(hardwareMap, telemetry);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(56, -34, Math.toRadians(180));

        drive.setPoseEstimate(startingPosition);

        //Define the movements of the robot that we need.
        TrajectorySequence centerMoveForward = drive.trajectorySequenceBuilder(startingPosition)
                .forward(30)
                .back(25)
                .build();

        TrajectorySequence centerMoveBack = drive.trajectorySequenceBuilder(centerMoveForward.end())
                .back(25)
                .build();

        TrajectorySequence centerMoveToSide = drive.trajectorySequenceBuilder(centerMoveBack.end())
                .lineToLinearHeading(new Pose2d(-7,-45,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-10, 54, Math.toRadians(270)))
                .build();


        TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(28, -34, Math.toRadians(225)))
                .back(13)
                .turn(Math.toRadians(45))
                .back(2)
                //.lineToLinearHeading(new Pose2d(-5,-30,Math.toRadians(270)))
                //.lineToLinearHeading(new Pose2d(-11, 54, Math.toRadians(270)))
                .build();

        TrajectorySequence moveRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(29, -34, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(34,-45,Math.toRadians(180)))
               // .lineToLinearHeading(new Pose2d(-5,-30,Math.toRadians(270)))
               // .lineToLinearHeading(new Pose2d(-11, 57, Math.toRadians(270)))
                .build();



        centerMoveForwardFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveForward);

        centerMoveBackFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveBack);
        centerMoveToWallFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, centerMoveToSide);

        leftFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveLeft);
        rightFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveRight);

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(

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
                                                            new InstantCommand(() -> telemetry.addData("Running", "Right"))
                                                        ),
                                                        //doing the forward paths
                                                        new SequentialCommandGroup(
                                                                centerMoveForwardFollower//,
                                                               // centerMoveBackFollower,
                                                               //new Disable(intakeSubsystem),
                                                               // centerMoveToWallFollower
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