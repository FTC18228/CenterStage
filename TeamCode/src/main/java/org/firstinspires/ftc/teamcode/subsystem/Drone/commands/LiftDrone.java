package org.firstinspires.ftc.teamcode.subsystem.Drone.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Drone.DroneSubSystem;

public class LiftDrone extends CommandBase {

    private final DroneSubSystem droneSubSystem;

    public LiftDrone(DroneSubSystem droneSubSystem) {
        this.droneSubSystem = droneSubSystem;
        addRequirements(this.droneSubSystem);
    }

    @Override
    public void initialize(){droneSubSystem.LiftDrone();}

    @Override
    public boolean isFinished(){return true;}
}
