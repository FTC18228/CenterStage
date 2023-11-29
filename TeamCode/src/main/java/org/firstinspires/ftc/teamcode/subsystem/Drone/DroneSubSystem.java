package org.firstinspires.ftc.teamcode.subsystem.Drone;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneSubSystem extends SubsystemBase {

    private final Servo droneServo;

    public DroneSubSystem(final HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(Servo.class,"droneServo");
        droneServo.setPosition(0);
    }

    public void Launch() {
        droneServo.setPosition(1);
    }
}
