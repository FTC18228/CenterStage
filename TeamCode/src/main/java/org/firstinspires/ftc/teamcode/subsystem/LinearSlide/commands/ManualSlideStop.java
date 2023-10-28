package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class ManualSlideStop extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;


    public ManualSlideStop(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;
        addRequirements(slideSubSystem);
    }

    @Override
    public void execute() {
        slideSubSystem.SlideOff();
    }

    @Override
    public boolean isFinished() {

        return true;
    }
}
