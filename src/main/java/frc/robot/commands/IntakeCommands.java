package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/**
 * The class for the intake commands.
 */
public class IntakeCommands {
    public IntakeCommands() {}

	/**
	 * Returns the command that runs the intake
	 * to intake.
	 * 
	 * @return The command as an
	 * {@link InstantCommand} object.
	 */
    public static Command intakeInCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeIn(),
			Robot.intake
		);
	}

	/**
	 * Returns the command that runs the intake
	 * to intake. Runs indefinitely until the 
	 * command ends, which is useful for when
	 * you want to tell the command to run once.
	 * 
	 * @return The command as a
	 * {@link RunCommand} object.
	 */
	public static Command intakeRunCommand() {
		return new RunCommand(
			() -> Robot.intake.intakeIn(),
			Robot.intake
		);
	}

	/**
	 * Returns the command that runs the intake
	 * to outtake.
	 * 
	 * @return The command as an
	 * {@link InstantCommand} object.
	 */
	public static Command intakeOutCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeOut(),
			Robot.intake
		);
	}

	/**
	 * Returns the command that stops the intake 
	 * from running.
	 * 
	 * @return The command as an
	 * {@link InstantCommand} object.
	 */
	public static Command intakeStopCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeStop(),
			Robot.intake
		);
	}

	/**
	 * Returns the command that raises the intake.
	 * 
	 * @return The command as an
	 * {@link InstantCommand} object.
	 */
	public static Command intakeLiftUpCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeLiftUp(),
			Robot.intake
		);
	}

	/**
	 * Returns the command that lowers the intake.
	 * 
	 * @return The command as an
	 * {@link InstantCommand} object.
	 */
	public static Command intakeLiftDownCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeLiftDown(),
			Robot.intake
		);
	}

	/**
	 * Returns the command that stops the intake 
	 * raising and lowering motor from running.
	 * 
	 * @return The command as an
	 * {@link InstantCommand} object.
	 */
	public static Command intakeLiftStopCommand() {
		return new InstantCommand(
			() -> Robot.intake.intakeLiftStop(),
			Robot.intake
		);
	}
}
