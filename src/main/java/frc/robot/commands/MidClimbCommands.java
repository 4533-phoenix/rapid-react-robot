package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/**
 * The class for the mid climb commands.
 */
public class MidClimbCommands {
    public MidClimbCommands() {}
    
    /**
     * Returns the command that lowers the mid 
     * climb hook.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command climberDown() {
        return new InstantCommand(
            () -> Robot.midClimber.climberDown(),
            Robot.midClimber
        );
    }

    /**
     * Returns the command that raises the mid
     * climb hook.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command climberUp() {
        return new InstantCommand(
            () -> Robot.midClimber.climberUp(),
            Robot.midClimber
        );
    } 
    
    /**
     * Returns the command that stops the mid
     * climb hook motor from running.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command climberStop() {
        return new InstantCommand(
            () -> Robot.midClimber.climberStop(),
            Robot.midClimber
        );
    }
}
