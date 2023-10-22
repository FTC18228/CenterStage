package org.firstinspires.ftc.teamcode.subsystem.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;

public class Intake extends CommandBase {
    private final IntakeSubSystem intakeSubSystem;

    public Intake (IntakeSubSystem subSystem) {
        intakeSubSystem = subSystem;
        addRequirements(intakeSubSystem);
    }

    @Override
    public void execute() {
        intakeSubSystem.Intake();
    }

    public boolean isFinished() {
        return true;
    }

}
