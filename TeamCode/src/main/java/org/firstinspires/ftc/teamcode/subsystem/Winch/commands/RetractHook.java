package org.firstinspires.ftc.teamcode.subsystem.Winch.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Winch.WinchSubSystem;

public class RetractHook extends CommandBase {

    private final WinchSubSystem winchSubSystem;

    public RetractHook(WinchSubSystem winchSubSystem) {
        this.winchSubSystem = winchSubSystem;
        addRequirements(this.winchSubSystem);
    }

    @Override
    public void initialize(){winchSubSystem.RetractHook();}

    @Override
    public boolean isFinished(){return true;}
}