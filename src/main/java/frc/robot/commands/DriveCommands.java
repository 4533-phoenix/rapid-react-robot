package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class DriveCommands {
    public DriveCommands() {}

    public static Command toggleTurbo() {
        return new InstantCommand(
            () -> Robot.drive.toggleTurbo(),
            Robot.drive
        );
    }

    public static Command toggleDriveMode() {
        return new InstantCommand(
            () -> Robot.drive.toggleDriveMode(),
            Robot.drive
        );
    }

    public static Command quarterTrue() {
        return new InstantCommand(
            () -> Robot.drive.quarterTrue(),
            Robot.drive
        );
    }

    public static Command quarterFalse() {
        return new InstantCommand(
            () -> Robot.drive.quarterFalse(),
            Robot.drive
        );
    }
}
