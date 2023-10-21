package org.firstinspires.ftc.teamcode.subsystem.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;

public class Disable extends CommandBase {
    private final IntakeSubSystem intakeSubSystem;

    public Disable(IntakeSubSystem subSystem) {
        intakeSubSystem = subSystem;
        addRequirements(intakeSubSystem);
    }

    @Override
    public void initialize() {
        intakeSubSystem.Disable();
    }

    @Override
    public void execute() {

    }

    public boolean isFinished() {
        return true;
    }

}
