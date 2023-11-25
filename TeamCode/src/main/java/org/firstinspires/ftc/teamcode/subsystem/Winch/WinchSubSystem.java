package org.firstinspires.ftc.teamcode.subsystem.Winch;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WinchSubSystem extends SubsystemBase {

    private final Servo liftServo;
    private final Servo extendServo;

    public WinchSubSystem(final HardwareMap hMap){
        liftServo = hMap.get(Servo.class, "liftServo");
        extendServo = hMap.get(Servo.class, "extendServo");
        liftServo.setDirection(Servo.Direction.REVERSE);
        liftServo.setPosition(0);
        extendServo.setPosition(0);
    }

    public void LiftWinch(){
        liftServo.setPosition(0.333);
    }
    public void ReleaseWinch(){
        liftServo.setPosition(0);
    }

    public void ExtendHook(){
        extendServo.setPosition(0.5);
    }

    public void RetractHook(){
        extendServo.setPosition(0);
    }
}
