package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubSystem extends SubsystemBase {
    private final DcMotor intakeMotor;
    public IntakeSubSystem(final HardwareMap hardwareMap, final String name) {

        intakeMotor = hardwareMap.get(DcMotor.class, name);
    }

    //Intake pixels
    public void Intake() {
        intakeMotor.setPower(1);
    }

    //Eject pixels
    public void Outtake() {
        intakeMotor.setPower(-1);
    }
}
