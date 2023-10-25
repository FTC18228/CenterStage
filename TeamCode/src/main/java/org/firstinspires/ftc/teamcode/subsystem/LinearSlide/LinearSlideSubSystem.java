package org.firstinspires.ftc.teamcode.subsystem.LinearSlide;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearSlideSubSystem extends SubsystemBase {
    private final DcMotor slideMotor;
    private final Servo depositServo;
    private final Servo gateServo;
    public LinearSlideSubSystem(final HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
    }

    //region SlideChecks
        public void ResetSlideEncoders() {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        //Check if linear slides are inside robot
        public boolean isSlideInInitialPos(int currentPos) {
            if(currentPos == 0) {return true;}
            return false;
        }

        //Check if linear slides are extended
        public boolean isSlideInMaxPos(int currentPos) {
            if(currentPos == 1) {return true;}
            return false;
        }
    //endregion

    //region Movement
    public void SlideExtend() {
     slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     slideMotor.setTargetPosition(1);
     slideMotor.setPower(1);
    }

    public void SlideCompress() {
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
    }

    public void FlipDeposit() {
        depositServo.setPosition(0.8);
    }

    public void RetractDeposit() {
        depositServo.setPosition(0);
    }

    public void OpenGate() {
        gateServo.setPosition(1);
    }

    public void CloseGate() {
        gateServo.setPosition(0);
    }
    //endregion
}
