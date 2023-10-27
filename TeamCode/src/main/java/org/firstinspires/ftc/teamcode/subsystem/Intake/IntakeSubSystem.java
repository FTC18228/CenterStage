package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubSystem extends SubsystemBase {
    private final DcMotor intakeMotor;
    private final CRServo intakeServo;
    public IntakeSubSystem(final HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    //Intake pixels
    public void Intake() {
        intakeMotor.setPower(1);
        intakeServo.setPower(1);
    }

    //Eject pixels
    public void Outtake() {
        intakeMotor.setPower(-1);
        intakeServo.setPower(-1);
    }

    //Turns off intake
    public void Disable() {
        intakeMotor.setPower(0);
        intakeServo.setPower(0);
    }
}
