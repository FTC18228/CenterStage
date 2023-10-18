package org.firstinspires.ftc.teamcode.subsystem.LinearSlide;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearSlideSubSystem extends SubsystemBase {
    private final DcMotor slideMotor;
    private final Servo depositServo;
    public LinearSlideSubSystem(final HardwareMap hardwareMap, final String name) {
        slideMotor = hardwareMap.get(DcMotor.class, name);
        depositServo = hardwareMap.get(Servo.class, name);
    }

    public void Extend() {

    }

    public void Compress() {

    }

    public void Deliver() {
        
    }
}
