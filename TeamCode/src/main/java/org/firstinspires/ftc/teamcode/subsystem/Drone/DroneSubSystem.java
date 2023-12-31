package org.firstinspires.ftc.teamcode.subsystem.Drone;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneSubSystem extends SubsystemBase {

    private final Servo droneServo;
    private final Servo liftServo;

    public DroneSubSystem(final HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(Servo.class,"droneServo");
        liftServo = hardwareMap.get(Servo.class,"liftServo");
        droneServo.setPosition(0);
    }

    public void LiftDrone() {liftServo.setPosition(0.15);}

    public void SlowLiftDrone() {
        double i = 0;
        while(i < 0.15){
            liftServo.setPosition(i);
            i += 0.01;
        }
    }

    public void Launch() {
        droneServo.setPosition(0.3);
    }
}
