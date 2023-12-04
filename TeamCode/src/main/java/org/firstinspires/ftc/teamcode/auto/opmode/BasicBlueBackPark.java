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
import org.firstinspires.ftc.teamcode.subsystem.Drone.DroneSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Disable;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.Vision.CSVisionProcessor;
import org.firstinspires.ftc.teamcode.subsystem.Vision.VisionSubSystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class BasicBlueBackPark extends AutoOpBase {

    private BotBuildersMecanumDrive drive;
    private DriveSubsystem driveSubsystem;
    private IntakeSubSystem intakeSubsystem;

    private DroneSubSystem droneSubSystem;

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

        droneSubSystem = new DroneSubSystem(hardwareMap);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(-36, 62, Math.toRadians(270));

        drive.setPoseEstimate(startingPosition);

        //Define the movements of the robot that we need.
        TrajectorySequence centerMoveForward = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(-36,32, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-36,48, Math.toRadians(270)))
                .build();

        TrajectorySequence centerMoveBack = drive.trajectorySequenceBuilder(centerMoveForward.end())
                .lineToLinearHeading(new Pose2d(-45,48, Math.toRadians(270)))
                .build();

        TrajectorySequence centerMoveToSide = drive.trajectorySequenceBuilder(centerMoveBack.end())

                .lineToLinearHeading(new Pose2d(-45,5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-45,0, Math.toRadians(180)))
                //.lineToLinearHeading(new Pose2d(54, -10, Math.toRadians(180)))
                .build();


        TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(-33, 36, Math.toRadians(310)))
                .lineToLinearHeading(new Pose2d(-36,50,Math.toRadians(270)))
                //.lineToLinearHeading(new Pose2d(-40,20,Math.toRadians(270)))
                //.lineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(180)))
                //.lineToLinearHeading(new Pose2d(54, -5, Math.toRadians(180)))
                .build();



        TrajectorySequence moveRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToLinearHeading(new Pose2d(-35, 38, Math.toRadians(225)))
                .lineToLinearHeading(new Pose2d(-34,50,Math.toRadians(270)))
                //.lineToLinearHeading(new Pose2d(-34,20,Math.toRadians(270)))
                //.lineToLinearHeading(new Pose2d(-36, 0, Math.toRadians(180)))
                //.lineToLinearHeading(new Pose2d(54, -5, Math.toRadians(180)))
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
                                                                centerMoveForwardFollower
                                                                //centerMoveBackFollower
                                                                //new Disable(intakeSubsystem),
                                                                //centerMoveToWallFollower
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