package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.arcrobotics.ftclib.command.CommandScheduler;
        import com.arcrobotics.ftclib.command.ParallelCommandGroup;
        import com.arcrobotics.ftclib.command.WaitUntilCommand;
        import com.arcrobotics.ftclib.gamepad.GamepadEx;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
        import org.firstinspires.ftc.teamcode.subsystem.Drive.DriveSubsystem;
        import org.firstinspires.ftc.teamcode.subsystem.Drive.TrajectorySequenceFollowerCommand;
        import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Intake;
        import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
        import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class SampleAuto extends AutoOpBase {

    private BotBuildersMecanumDrive drive;

    private DriveSubsystem driveSubsystem;

    private GamepadEx gamepadEx1;
    private TrajectorySequenceFollowerCommand parkFollower;
    private IntakeSubSystem intakeSubsystem;

    @Override
    public void initialize() {

        gamepadEx1 = new GamepadEx(gamepad1);
        drive = new BotBuildersMecanumDrive(hardwareMap);
        driveSubsystem = new DriveSubsystem(drive, gamepadEx1, telemetry);

        intakeSubsystem = new IntakeSubSystem(hardwareMap, "intake");

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(72, 0, Math.toRadians(0));

        drive.setPoseEstimate(startingPosition);

        //Define the movements of the robot that we need.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)
                .forward(11)

                .build();

        parkFollower = new TrajectorySequenceFollowerCommand(driveSubsystem, moveForward);

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new ParallelCommandGroup(
                                new Intake(intakeSubsystem),
                                parkFollower
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