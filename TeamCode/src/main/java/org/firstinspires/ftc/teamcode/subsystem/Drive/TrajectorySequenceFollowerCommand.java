package org.firstinspires.ftc.teamcode.subsystem.Drive;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceFollowerCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final TrajectorySequence trajectorySeq;

    public TrajectorySequenceFollowerCommand(DriveSubsystem  drive, TrajectorySequence trajectory) {
        this.drive = drive;
        this.trajectorySeq = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(trajectorySeq);
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