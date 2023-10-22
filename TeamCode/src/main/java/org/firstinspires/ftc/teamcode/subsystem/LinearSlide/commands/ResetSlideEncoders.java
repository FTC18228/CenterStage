package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class ResetSlideEncoders extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;
    public ResetSlideEncoders(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;
        addRequirements(slideSubSystem);
    }

    @Override
    public void initialize() {
        slideSubSystem.ResetSlideEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
