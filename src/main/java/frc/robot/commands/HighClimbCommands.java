package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/**
 * The class for the high climb commands.
 */
public class HighClimbCommands {
    public HighClimbCommands() {}

    /**
     * Returns the command that lowers the high climb hook.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command highClimberDown() {
        return new InstantCommand(
            () -> Robot.highClimber.highClimberDown(),
            Robot.highClimber
        );
    }

    /**
     * Returns the command that raises the high climb hook.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command highClimberUp() {
        return new InstantCommand(
            () -> Robot.highClimber.highClimberUp(),
            Robot.highClimber
        );
    }
    
    /**
     * Returns the command that stops the high climb hook
     * motor from running.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command highClimberStop() {
        return new InstantCommand(
            () -> Robot.highClimber.highClimberStop(),
            Robot.highClimber
        );
    }
}
