package org.firstinspires.ftc.teamcode.subsystem.Vision.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.subsystem.Vision.VisionSubSystem;

public class StopStreamingCommand extends CommandBase
{

    VisionSubSystem vision;

    public StopStreamingCommand(VisionSubSystem visionSubSystem){

        vision = visionSubSystem;
    }

    @Override
    public void execute() {
        vision.StopStreaming();
    }

    @Override
    public boolean isFinished() {

        return true;
    }
}
