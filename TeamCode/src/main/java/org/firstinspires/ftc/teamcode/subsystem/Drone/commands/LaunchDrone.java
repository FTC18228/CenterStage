package org.firstinspires.ftc.teamcode.subsystem.Drone.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Drone.DroneSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.Winch.WinchSubSystem;

public class LaunchDrone extends CommandBase {

    private final DroneSubSystem droneSubSystem;

    public LaunchDrone(DroneSubSystem droneSubSystem) {
        this.droneSubSystem = droneSubSystem;
        addRequirements(this.droneSubSystem);
    }

    @Override
    public void initialize(){droneSubSystem.Launch();}

    @Override
    public boolean isFinished(){return true;}
}