package org.firstinspires.ftc.teamcode.subsystem.Vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

public class VisionSubSystem extends SubsystemBase {
    private final Camera webcam;
    public VisionSubSystem(final HardwareMap hardwareMap, final String name) {
        webcam = hardwareMap.get(Camera.class, name);
    }
    @Override
    public void periodic() {

    }
}
