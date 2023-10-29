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
        depositServo = hardwareMap.get(Servo.class, "arm");
        gateServo = hardwareMap.get(Servo.class, "release");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        depositServo.setPosition(0);
        gateServo.setPosition(0);
    }

    //region SlideChecks
        public void ResetSlideEncoders() {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public boolean IsExtended(){

            if(slideMotor.getCurrentPosition() < 2050){
                return false;
            }
            return true;
        }

        public boolean IsRetracted(){
            if(slideMotor.getCurrentPosition() > 0){
                return false;
            }
            return true;
        }
    //endregion

    //region Movement
    public void SlideExtend() {
     slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     slideMotor.setTargetPosition(1);
     slideMotor.setPower(1);
    }

    public void ManualSlideExtend(){
        slideMotor.setPower(1);
    }

    public void ManualSlideRetract(){
        slideMotor.setPower(-1);
    }

    public void SlideOff(){
        slideMotor.setPower(0);
    }

    public void SlideCompress() {
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
    }

    public void FlipDeposit() {
        //This is a 5 turn servo
        depositServo.setPosition(0.5);
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
