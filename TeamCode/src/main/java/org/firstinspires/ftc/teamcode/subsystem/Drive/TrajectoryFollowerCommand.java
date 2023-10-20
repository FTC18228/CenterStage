package org.firstinspires.ftc.teamcode.subsystem.Drive;



        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.arcrobotics.ftclib.command.CommandBase;


public class TrajectoryFollowerCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final Trajectory trajectory;

    public TrajectoryFollowerCommand(DriveSubsystem  drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectory(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            //drive..stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}