package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drive.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Drone.DroneSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Disable;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.CloseGate;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.FlipDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.ManualSlideExtend;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.ManualSlideRetract;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.ManualSlideStop;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.OpenGate;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.RetractDeposit;
import org.firstinspires.ftc.teamcode.subsystem.Winch.WinchSubSystem;

import java.util.function.BooleanSupplier;

@TeleOp(group ="drive")
public class SchedularOpMode  extends CommandOpMode {

    private GamepadEx gp1;
    private GamepadEx gp2;

    private DriveCommand driveCommand;

    private BotBuildersMecanumDrive bbMec;

    DriveSubsystem driveSubsystem;
    IntakeSubSystem intakeSubSystem;
    LinearSlideSubSystem slideSubSystem;
    WinchSubSystem winchSubSystem;
    DroneSubSystem droneSubSystem;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        bbMec = new BotBuildersMecanumDrive(hardwareMap);

        //TODO: setup any init commands

        gp1= new GamepadEx(gamepad1);
        gp2= new GamepadEx(gamepad2);
        driveSubsystem = new DriveSubsystem(bbMec, gp1, telemetry);

        //TODO: add other subsystems.

        intakeSubSystem = new IntakeSubSystem(hardwareMap);
        slideSubSystem = new LinearSlideSubSystem(hardwareMap);
        droneSubSystem = new DroneSubSystem(hardwareMap);
        winchSubSystem = new WinchSubSystem(hardwareMap);

        //TODO: Control scheme
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new Intake(intakeSubSystem)
        ).whenReleased(
                new Disable(intakeSubSystem)
        );

        gp2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new Intake(intakeSubSystem)
        ).whenReleased(
                new Disable(intakeSubSystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new Outtake(intakeSubSystem)
        ).whenReleased(
                new Disable(intakeSubSystem)
        );

        gp2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new Outtake(intakeSubSystem)
        ).whenReleased(
                new Disable(intakeSubSystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
                new FlipDeposit(slideSubSystem),
                new RetractDeposit(slideSubSystem)
        );

        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new FlipDeposit(slideSubSystem)
        ).whenReleased(
                new RetractDeposit(slideSubSystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(
                new OpenGate(slideSubSystem),
                new CloseGate(slideSubSystem)
        );

        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(
                new OpenGate(slideSubSystem),
                new CloseGate(slideSubSystem)
        );

        /*gp1.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(
                new FlipDeposit(slideSubsystem),
                new RetractDeposit(slideSubsystem)
        );*/

        driveCommand = new DriveCommand(
                driveSubsystem, () -> -gp1.getLeftY(),
                gp1::getLeftX, gp1::getRightX
        );

        schedule(driveCommand);

        new Trigger(new BooleanSupplier(){
            @Override
            public boolean getAsBoolean(){
                return gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
            }
            }).whenActive(
                    new ManualSlideExtend(slideSubSystem)
        );

        new Trigger(new BooleanSupplier(){
            @Override
            public boolean getAsBoolean(){
                return gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
            }
        }).whenActive(
                new ManualSlideExtend(slideSubSystem)
        );

        new Trigger(new BooleanSupplier(){
            @Override
            public boolean getAsBoolean(){
                return gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
            }).whenActive(
                    new ManualSlideRetract(slideSubSystem)
        );

        new Trigger(new BooleanSupplier(){
            @Override
            public boolean getAsBoolean(){
                return gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
        }).whenActive(
                new ManualSlideRetract(slideSubSystem)
        );

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 0.5 && gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.5;
            }
        }).whenActive(
                new ManualSlideStop(slideSubSystem)
        );

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 0.5 && gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.5;
            }
        }).whenActive(
                new ManualSlideStop(slideSubSystem)
        );

        //driveCommand = new DriveC




    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        //
    }


}
