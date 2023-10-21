package org.firstinspires.ftc.teamcode.subsystem.Drive;


import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.localization.Localizer;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
        import com.arcrobotics.ftclib.command.SubsystemBase;
        import com.arcrobotics.ftclib.gamepad.GamepadEx;
        import com.arcrobotics.ftclib.gamepad.GamepadKeys;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
        import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
        import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

        import java.util.List;
        import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {

    BotBuildersMecanumDrive driveBase;
    GamepadEx gp1;
    Telemetry telemetry;

    public DriveSubsystem(BotBuildersMecanumDrive drive, GamepadEx gamepad, Telemetry tele) {

        gp1 = gamepad;
        driveBase = drive;
        telemetry = tele;
    }

    public void Realign(){
        telemetry.addData("IMU", "RESET");
       // driveBase.ReAlignIMU();
    }

    public void setPoseEstimate(Pose2d pose) {
        driveBase.setPoseEstimate(pose);
    }

    public void update() {
        driveBase.update();
    }

    public void updatePoseEstimate() {
        driveBase.updatePoseEstimate();
    }

    public Pose2d getPoseEstimate() {
        return driveBase.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return driveBase.trajectoryBuilder(startPose);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose){
        return driveBase.trajectorySequenceBuilder(startPose);
    }

    public void followTrajectorySequence(TrajectorySequence seq){
        driveBase.followTrajectorySequence(seq);
    }


    public void followTrajectorySequenceAsync(TrajectorySequence seq){
        driveBase.followTrajectorySequenceAsync(seq);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return driveBase.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return driveBase.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        driveBase.followTrajectoryAsync(trajectory);
    }

    public boolean isBusy() {
        return driveBase.isBusy();
    }

    public void turn(double radians) {
        driveBase.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return driveBase.getWheelVelocities();
    }



    public Pose2d getPoseVelocity() {
        return driveBase.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return driveBase.getLocalizer();
    }


    public void drive(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        Pose2d poseEstimate = driveBase.getPoseEstimate();

        Vector2d input = new Vector2d(
                -leftY.getAsDouble(),
                -leftX.getAsDouble()

        ).rotated(-poseEstimate.getHeading()); //You can turn off field oriented controls

        Pose2d vel = new Pose2d(
                input.getX(),
                input.getY(),
                -rightX.getAsDouble()  //TODO: Look at rotation speed.
        );


        driveBase.setWeightedDrivePower(vel);
        driveBase.update();

    }
    public void drive() {
        //telemetry.addData("DRIVE","Driving");
        Pose2d poseEstimate = driveBase.getPoseEstimate();
        double velocity = 0.5;

        Vector2d input = new Vector2d(
                -gp1.gamepad.left_stick_y * 0.95,
                -gp1.gamepad.left_stick_x * 0.95

        ).rotated(-poseEstimate.getHeading());

        if(gp1.gamepad.b){
            input = new Vector2d(
                    -gp1.gamepad.left_stick_y,
                    -gp1.gamepad.left_stick_x

            ).rotated(-poseEstimate.getHeading());
        }

        Pose2d vel = new Pose2d(
                input.getX(),
                input.getY(),
                -gp1.gamepad.right_stick_x * velocity
        );

        // driveBase.DumpData(telemetry);
        driveBase.setWeightedDrivePower(vel);
        driveBase.update();

    }
}