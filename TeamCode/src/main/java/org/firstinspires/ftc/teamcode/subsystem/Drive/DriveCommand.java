package org.firstinspires.ftc.teamcode.subsystem.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;


import java.util.function.DoubleSupplier;


public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX;
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {

        this.drive = driveSubsystem;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        drive.drive(this.leftX, this.leftY, this.rightX);
    }
}
