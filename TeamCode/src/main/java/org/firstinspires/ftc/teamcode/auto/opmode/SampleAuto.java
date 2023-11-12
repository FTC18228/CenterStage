package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
        import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.SlideExtend;
import org.firstinspires.ftc.teamcode.subsystem.Vision.CSVisionProcessor;
import org.firstinspires.ftc.teamcode.subsystem.Vision.VisionSubSystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
@Disabled
public class SampleAuto extends AutoOpBase {

    private BotBuildersMecanumDrive drive;
    private DriveSubsystem driveSubsystem;
    private IntakeSubSystem intakeSubsystem;

    private VisionSubSystem visionSubSystem;
    private GamepadEx gamepadEx1;
    private TrajectorySequenceFollowerCommand parkFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand rightFollower;

    @Override
    public void initialize() {

        gamepadEx1 = new GamepadEx(gamepad1);
        drive = new BotBuildersMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(drive, gamepadEx1, telemetry);

        intakeSubsystem = new IntakeSubSystem(hardwareMap);

        visionSubSystem = new VisionSubSystem(hardwareMap, telemetry);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(72, 0, Math.toRadians(0));

        drive.setPoseEstimate(startingPosition);

        //Define the movements of the robot that we need.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)
                .forward(11)
                .build();

        TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(startingPosition)
                .turn(Math.toRadians(45))
                .forward(11)
                .build();

        TrajectorySequence moveRight = drive.trajectorySequenceBuilder(startingPosition)
                .turn(Math.toRadians(-45))
                .forward(11)
                .build();



        parkFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveForward);
        leftFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveLeft);
        rightFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveRight);

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
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
                                                        parkFollower,
                                                        () -> {return visionSubSystem.getPosition() == CSVisionProcessor.StartingPosition.RIGHT;}
                                                ),
                                                () -> { return visionSubSystem.getPosition() == CSVisionProcessor.StartingPosition.LEFT;}

                                        )
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