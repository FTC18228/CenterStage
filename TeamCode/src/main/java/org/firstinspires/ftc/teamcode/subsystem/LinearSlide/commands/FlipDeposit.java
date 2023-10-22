package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class FlipDeposit extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;
    private Servo depositServo;

    public FlipDeposit(LinearSlideSubSystem subSystem, HardwareMap hMap) {
        slideSubSystem = subSystem;
        depositServo = hMap.get(Servo.class, "depositServo");
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
