package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class FlipDeposit extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;

    public FlipDeposit(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;
        addRequirements(slideSubSystem);
    }

    @Override
    public void initialize() {
        slideSubSystem.FlipDeposit();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
