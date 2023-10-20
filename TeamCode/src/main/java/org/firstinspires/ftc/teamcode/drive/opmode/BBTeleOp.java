package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class BBTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecanumDrive drive = new BotBuildersMecanumDrive(hardwareMap);

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();


            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            if(gamepad1.a) {
                intakeMotor.setPower(1);
                intakeServo.setPower(1);
            }
            else {
                intakeMotor.setPower(0);
                intakeServo.setPower(0);
            }
            if(gamepad1.b){
                intakeMotor.setPower(-1);
                intakeServo.setPower(-1);
            }else{
                intakeMotor.setPower(0);
                intakeServo.setPower(0);
            }


        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
