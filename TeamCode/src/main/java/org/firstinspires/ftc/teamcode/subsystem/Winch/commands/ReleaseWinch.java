package org.firstinspires.ftc.teamcode.subsystem.Winch.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Winch.WinchSubSystem;

public class ReleaseWinch extends CommandBase {

    private final WinchSubSystem winchSubSystem;

    public ReleaseWinch(WinchSubSystem winchSubSystem) {
        this.winchSubSystem = winchSubSystem;
        addRequirements(this.winchSubSystem);
    }

    @Override
    public void initialize() {
        winchSubSystem.ReleaseWinch();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
