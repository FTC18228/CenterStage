package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class ManualSlideExtend extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;


    public ManualSlideExtend(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;
        addRequirements(slideSubSystem);
    }

    @Override
    public void execute() {
        slideSubSystem.ManualSlideExtend();
    }

    @Override
    public boolean isFinished() {

        return slideSubSystem.IsExtended();
    }
}
