package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class CloseGate extends CommandBase {
    private final LinearSlideSubSystem slideSubSystem;
    private final Servo gateServo;

    public CloseGate(LinearSlideSubSystem subSystem, HardwareMap hMap) {
        slideSubSystem = subSystem;
        gateServo = hMap.get(Servo.class, "gateServo");
        addRequirements(slideSubSystem);
    }

    @Override
    public void initialize() {
        slideSubSystem.CloseGate();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
