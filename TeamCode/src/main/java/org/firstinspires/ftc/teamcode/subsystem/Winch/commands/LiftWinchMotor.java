package org.firstinspires.ftc.teamcode.subsystem.Winch.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Winch.WinchSubSystem;

public class LiftWinchMotor extends CommandBase {

    private final WinchSubSystem winchSubSystem;

    public LiftWinchMotor(WinchSubSystem winchSubSystem) {
        this.winchSubSystem = winchSubSystem;
        addRequirements(this.winchSubSystem);
    }

    @Override
    public void initialize(){winchSubSystem.WinchForward();}

    @Override
    public boolean isFinished(){return true;}
}