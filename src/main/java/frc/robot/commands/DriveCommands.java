package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Robot;

/**
 * The class for the drive commands.
 */
public class DriveCommands {
    public DriveCommands() {}

    /**
     * Returns the command that sets the drive system
     * state of turbo drive to true.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command setTurboTrue() {
        return new InstantCommand(
            () -> Robot.drive.setTurboTrue(),
            Robot.drive
        );
    }

    /**
     * Returns the command that sets the drive system
     * state of turbo drive to false.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command setTurboFalse() {
        return new InstantCommand(
            () -> Robot.drive.setTurboFalse(),
            Robot.drive
        );
    }

    /**
     * Returns the command that toggles the drive
     * system's current drive mode between normal
     * and inverted.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command toggleDriveMode() {
        return new InstantCommand(
            () -> Robot.drive.toggleDriveMode(),
            Robot.drive
        );
    }

    /**
     * Returns the command that sets the drive system
     * state of quarter drive to true.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command quarterTrue() {
        return new InstantCommand(
            () -> Robot.drive.quarterTrue(),
            Robot.drive
        );
    }

    /**
     * Returns the command that sets the drive system
     * state of quarter drive to false.
     *  
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command quarterFalse() {
        return new InstantCommand(
            () -> Robot.drive.quarterFalse(),
            Robot.drive
        );
    }

    /**
     * Returns the command that sets the robot to use
     * PID control to reach the passed in setpoint 
     * for the passed in type of PID control 
     * (determines what PID control will
     * read, ex. type kPosition will read the robot's
     * position for PID calculations) using the PID
     * constants at the passed in PID slot ID.
     * 
     * @param setpoint The setpoint to set the PID control at.
     * @param type The type of value that PID control will read.
     * @param slotID The slot ID for the desired PID slot constants.
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command pidDriveCommand(double setpoint, ControlType type, int slotID) {
        return new InstantCommand (
            () -> Robot.drive.pidDrive(setpoint, type, slotID),
            Robot.drive
        );
    }
}
