package org.firstinspires.ftc.teamcode.subsystem.Vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class VisionSubSystem extends SubsystemBase {
    //private final Camera webcam;
    private final CSVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    private CSVisionProcessor.StartingPosition startingPos;

    private Telemetry opTelemetry;

    public VisionSubSystem(final HardwareMap hardwareMap, Telemetry telemetry) {

        opTelemetry = telemetry;
        visionProcessor = new CSVisionProcessor(100, 50,150, 150, 50,200,150);

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);

    }

    public CSVisionProcessor.StartingPosition getPosition(){
        return startingPos;
    }

    public void StopStreaming(){
        visionPortal.stopStreaming();
    }

    @Override
    public void periodic() {
        startingPos = visionProcessor.getStartingPosition();
        opTelemetry.addData("Vision Pos", startingPos);
        opTelemetry.update();
    }

    public void ScanAprilTag() {

    }
}
