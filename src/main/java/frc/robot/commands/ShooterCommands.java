package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class ShooterCommands {
    public ShooterCommands() {}

    public static Command flywheelOutCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelOut(),
            Robot.shooter
        );
    }

    public static Command flywheelInCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelIn(),
            Robot.shooter
        );
    }

    public static Command flywheelIntakeInCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelIntakeIn(), 
            Robot.shooter
        );
    }

    public static Command flywheelIntakeOutCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelIntakeOut(),
            Robot.shooter
        );
    }
}
