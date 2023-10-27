package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class ManualSlideRetract extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;


    public ManualSlideRetract(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;

        addRequirements(slideSubSystem);
    }

    @Override
    public void execute() {
        slideSubSystem.ManualSlideRetract();
    }

    @Override
    public boolean isFinished() {

        return slideSubSystem.IsRetracted();
    }
}
