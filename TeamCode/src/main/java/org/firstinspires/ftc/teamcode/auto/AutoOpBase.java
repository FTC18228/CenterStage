package org.firstinspires.ftc.teamcode.auto;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoOpBase extends LinearOpMode {
    public AutoOpBase() {
    }

    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    public void runOpMode() throws InterruptedException {
        this.initialize();
        this.preInit();
        while(!this.isStarted() && !this.isStopRequested()) {
            this.run();
        }

        this.waitForStart();
        this.preStart();

        while(!this.isStopRequested() && this.opModeIsActive()) {
            this.run();
        }

        this.reset();
    }

    public abstract void initialize();

    public abstract void preInit();

    public abstract void preStart();

    public static void disable() {
        Robot.disable();
    }

    public static void enable() {
        Robot.enable();
    }
}