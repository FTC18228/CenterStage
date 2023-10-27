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
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Disable;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Intake.Commands.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.LinearSlideSubSystem;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.FlipDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.ManualSlideExtend;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.ManualSlideRetract;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.ManualSlideStop;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.RetractDeposit;
import org.firstinspires.ftc.teamcode.subsystem.LinearSlide.commands.SlideExtend;

import java.util.function.BooleanSupplier;

@TeleOp(group ="drive")
public class SchedularOpMode  extends CommandOpMode {

    private GamepadEx gp1;
    private GamepadEx gp2;

    private DriveCommand driveCommand;

    private BotBuildersMecanumDrive bbMec;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        bbMec = new BotBuildersMecanumDrive(hardwareMap);

        //TODO: setup any init commands

        gp1= new GamepadEx(gamepad1);
        gp2= new GamepadEx(gamepad2);


        DriveSubsystem drive = new DriveSubsystem(bbMec,gp1, telemetry);

        //TODO: add other subsystems.

        IntakeSubSystem intakeSubSystem = new IntakeSubSystem(hardwareMap);
        LinearSlideSubSystem slideSubsystem = new LinearSlideSubSystem(hardwareMap);

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new Intake(intakeSubSystem)
        ).whenReleased(
                new Disable(intakeSubSystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new Outtake(intakeSubSystem)
        ).whenReleased(
                new Disable(intakeSubSystem)
        );

       gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new FlipDeposit(slideSubsystem)
        ).whenReleased(
                new RetractDeposit(slideSubsystem)
        );

        /*gp1.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(
                new FlipDeposit(slideSubsystem),
                new RetractDeposit(slideSubsystem)
        );*/

        driveCommand = new DriveCommand(
                drive, () -> -gp1.getLeftY(),
                gp1::getLeftX, gp1::getRightX
        );


        schedule(driveCommand);


        new Trigger(new BooleanSupplier(){
            @Override
            public boolean getAsBoolean(){
                return gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
            }
        }).whenActive(
                new ManualSlideExtend(slideSubsystem)
        );

        new Trigger(new BooleanSupplier(){
            @Override
            public boolean getAsBoolean(){
                return gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
        }).whenActive(
                new ManualSlideRetract(slideSubsystem)
        );

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) <= 0.5 && gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.5;
            }
        }).whenActive(
                new ManualSlideStop(slideSubsystem)
        );

        //driveCommand = new DriveC




    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        //
    }


}
