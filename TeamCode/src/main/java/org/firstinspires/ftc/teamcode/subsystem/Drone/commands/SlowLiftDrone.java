package org.firstinspires.ftc.teamcode.subsystem.Drone.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Drone.DroneSubSystem;

public class SlowLiftDrone extends CommandBase {

    private DroneSubSystem droneSubSystem;

    public SlowLiftDrone(DroneSubSystem droneSubSystem){
        this.droneSubSystem = droneSubSystem;
        addRequirements(this.droneSubSystem);
    }

    @Override
    public void initialize(){droneSubSystem.SlowLiftDrone();}

    @Override
    public boolean isFinished(){return true;}
}
