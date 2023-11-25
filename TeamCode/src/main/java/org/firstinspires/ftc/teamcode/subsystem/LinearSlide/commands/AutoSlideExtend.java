package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class AutoSlideExtend extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;

    private final int AUTO_HEIGHT = 250;

    public AutoSlideExtend(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;
        addRequirements(slideSubSystem);
    }

    @Override
    public void execute() {
        slideSubSystem.SlideAutoHeight(AUTO_HEIGHT);
    }

    @Override
    public boolean isFinished() {
        return slideSubSystem.IsAutoExtended(AUTO_HEIGHT);
    }
}
