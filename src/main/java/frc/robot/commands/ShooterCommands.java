package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.Robot;

/**
 * The class for the shooter commands.
 */
public class ShooterCommands {
    public ShooterCommands() {}

    /**
     * Returns the command that runs the flywheel
     * to shoot out balls.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command flywheelOutCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelOut(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that runs the flywheel
     * to reverse balls out of the flywheel turret.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command flywheelInCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelIn(),
            Robot.shooter
        );
    }
    
    /**
     * Returns the command that runs the flywheel
     * to shoot out balls. Runs indefinitely until
     * the command ends, which is useful for when
     * you want to tell the command to run once.
     * 
     * @return The command as a
     * {@link RunCommand} object.
     */
    public static Command flywheelRunCommand() {
        return new RunCommand(
            () -> Robot.shooter.flywheelOut(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that stops the flywheel
     * motors from running.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command flywheelStopCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelStop(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that runs the flywheel intake
     * to intake.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command flywheelIntakeInCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelIntakeIn(), 
            Robot.shooter
        );
    }

    /**
     * Returns the command that runs the flywheel intake
     * to outtake.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command flywheelIntakeOutCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelIntakeOut(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that stops the flywheel intake
     * motor from running.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command flywheelIntakeStopCommand() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelIntakeStop(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that runs the flywheel and flywheel intake
     * for a specific amount of time to shoot balls based on the 
     * passed in number of balls to shoot and the passed in whether 
     * the flywheel is currently running.
     * 
     * @param balls The number of balls to shoot.
     * @param flywheelRunning Whether or not the flywheel is currently running.
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command flywheelAndIntakeRunCommand(int balls, boolean flywheelRunning) {
        return new InstantCommand(
            () -> Robot.shooter.flywheelAndFlywheelIntakeRun(balls, flywheelRunning),
            Robot.shooter
        );
    }

    /**
     * Returns the command that stops the flywheel and
     * flywheel intake motors from running.
     * 
     * @return The command as a
     * {@link SequentialCommandGroup} object.
     */
    public static Command flywheelAndIntakeStopCommand() {
        return new SequentialCommandGroup(
            flywheelStopCommand(),
            flywheelIntakeStopCommand()
        );
    }

    /**
     * Returns the command that moves the hood up by
     * 0.5 degrees.
     * 
     * @return The command as an
     * {@link RunCommand} object.
     */
    public static Command hoodUp() {
        return new RunCommand(
            () -> Robot.shooter.hoodUp(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that moves the hood down by
     * 0.5 degrees.
     * 
     * @return The command as an
     * {@link RunCommand} object.
     */
    public static Command hoodDown() {
        return new RunCommand(
            () -> Robot.shooter.hoodDown(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that stops the hood servo 
     * motor from moving via setting it to its 
     * current angle.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command hoodStop() {
        return new InstantCommand(
            () -> Robot.shooter.hoodStop(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that turns the robot
     * to face the hub via using vision.
     * 
     * @return The command as a
     * {@link FunctionalCommand} object.
     */
    public static Command autoFlywheelPos() {
        return new FunctionalCommand(
            () -> {}, 
            () -> Robot.shooter.setFlywheelPos(), 
            (interrupt) -> Robot.shooter.stopFlywheelPos(), 
            () -> Robot.shooter.flywheelReachedPosition(), 
            Robot.shooter
        );
    }

    /**
     * Returns the command that sets the hood angle
     * to the passed in angle.
     * 
     * @param angle The angle to set the hood to (degrees).
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command setHoodAngleCommand(double angle) {
        return new InstantCommand(
            () -> Robot.shooter.setHoodAngle(angle),
            Robot.shooter
        );
    }

    /**
     * @deprecated With the method {@link #autoFlywheelPos()}
     * using the PhotonVision library to work, there is not 
     * much need for this method once the PhotonVision 
     * library is fully implemented.
     * <p>
     * Returns the command that turns the robot
     * to face the hub via using vision.
     * 
     * @return The command as a
     * {@link FunctionalCommand} object.
     */
    public static Command oldAutoFlywheelPos() {
        return new FunctionalCommand(
            () -> {},
            () -> Robot.shooter.autoTurretSwivel(),
            (interrupt) -> Robot.shooter.stopFlywheelPos(),
            () -> Robot.shooter.turretReachedPosition(),
            Robot.shooter
        );
    }

    /**
     * Returns the command that shoots the balls into the
     * hub based on the passed in hood angle, the passed
     * in number of balls to shoot, and the passed in
     * whether the flywheel is currently running.
     * 
     * @param hoodAngle The angle to set the hood to (degrees).
     * @param balls The number of balls to shoot.
     * @param flywheelRunning Whether or not the flywheel is currently running.
     * @return The command as a
     * {@link SequentialCommandGroup} object.
     */
    public static Command autoShoot(double hoodAngle, int balls, boolean flywheelRunning) {
        return new SequentialCommandGroup(
            setHoodAngleCommand(hoodAngle),
            oldAutoFlywheelPos(),
            flywheelAndIntakeRunCommand(balls, flywheelRunning),
            flywheelAndIntakeStopCommand()
        );
    }
}
