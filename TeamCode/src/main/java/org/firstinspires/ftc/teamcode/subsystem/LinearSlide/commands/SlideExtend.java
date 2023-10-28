package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class SlideExtend extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;

    public SlideExtend(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;
        addRequirements(slideSubSystem);
    }

    @Override
    public void execute() {
        slideSubSystem.SlideExtend();
    }

    @Override
    public boolean isFinished() {
        return slideSubSystem.IsExtended();
    }
}
