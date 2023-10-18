package org.firstinspires.ftc.teamcode.subsystem.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;

public class Outtake extends CommandBase {
    private final IntakeSubSystem intakeSubSystem;
    public Outtake(IntakeSubSystem subSystem) {
        intakeSubSystem = subSystem;
        addRequirements(intakeSubSystem);
    }
    @Override
    public void initialize() {
        intakeSubSystem.Outtake();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
