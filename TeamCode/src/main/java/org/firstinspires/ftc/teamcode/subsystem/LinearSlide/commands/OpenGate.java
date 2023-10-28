package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class OpenGate extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;
    public OpenGate(LinearSlideSubSystem subSystem) {
        slideSubSystem = subSystem;
        addRequirements(slideSubSystem);
    }

    @Override
    public void initialize() {
        slideSubSystem.OpenGate();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
