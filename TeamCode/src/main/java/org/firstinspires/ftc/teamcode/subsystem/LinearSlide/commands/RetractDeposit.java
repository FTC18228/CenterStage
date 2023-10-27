package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class RetractDeposit extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;
    private Servo depositServo;

    public RetractDeposit(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;

        addRequirements(slideSubSystem);
    }

    @Override
    public void initialize() {
        slideSubSystem.RetractDeposit();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
