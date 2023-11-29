package org.firstinspires.ftc.teamcode.subsystem.Winch.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Winch.WinchSubSystem;

public class LiftWinchReverse extends CommandBase {

    private final WinchSubSystem winchSubSystem;

    public LiftWinchReverse(WinchSubSystem winchSubSystem) {
        this.winchSubSystem = winchSubSystem;
        addRequirements(this.winchSubSystem);
    }

    @Override
    public void initialize(){winchSubSystem.WinchReverse();}

    @Override
    public boolean isFinished(){return true;}
}