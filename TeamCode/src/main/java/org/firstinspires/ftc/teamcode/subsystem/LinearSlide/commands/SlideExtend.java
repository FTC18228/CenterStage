package org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;

public class SlideExtend extends CommandBase {

    private final LinearSlideSubSystem slideSubSystem;
    private final DcMotor slideMotor;

    public SlideExtend(LinearSlideSubSystem subSystem, HardwareMap hMap) {
        slideSubSystem = subSystem;
        slideMotor = hMap.get(DcMotor.class, "slideMotor");
        addRequirements(slideSubSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        slideSubSystem.SlideExtend();
    }

    @Override
    public boolean isFinished() {
        if(slideSubSystem.isSlideInMaxPos(slideMotor.getCurrentPosition())) {return true;}
        return false;
    }
}
