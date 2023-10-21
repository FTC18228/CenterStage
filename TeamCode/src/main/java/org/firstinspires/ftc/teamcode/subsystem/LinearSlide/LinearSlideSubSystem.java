package org.firstinspires.ftc.teamcode.subsystem.LinearSlide;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearSlideSubSystem extends SubsystemBase {
    private final DcMotor slideMotor;
    private final Servo depositServo;
    public LinearSlideSubSystem(final HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        depositServo = hardwareMap.get(Servo.class, "depositServo");
    }
    //region SlideChecks
        //Check if linear slides are inside robot
        public boolean isSlideInInitialPos(int currentPos) {
            if(currentPos == 0){return true;}
            return false;
        }

        //Check if linear slides are extended
        public boolean isSlideInMaxPos(int currentPos) {
            if(currentPos == 1){return true;}
            return false;
        }
    //endregion

    //region Movement
    public void Extend() {
     slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     slideMotor.setTargetPosition(1);
     slideMotor.setPower(1);
    }

    public void Compress() {
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
    }

    public void Deliver() {
        depositServo.setPosition(0.8);

    }
    //endregion
}
