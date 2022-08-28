package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSystem;

/**
 * The class for the autonomous commands.
 */
public class AutoCommands {
    public AutoCommands() {
    }

    // Test Commands:

    /**
     * @deprecated With odometry test class, there is not
     * much need for this method.
     * <p>
     * Returns the test command for testing odometry.
     * 
     * @return The test command as a 
     * {@link SequentialCommandGroup} object.
     */
    public static Command testDriveToPos() {
        return new SequentialCommandGroup(
            driveToPosAutoCommand(0.5, 1),
            driveToPosAutoCommand(0, 0),
            driveToPosAutoCommand(-0.5, 1),
            driveToPosAutoCommand(0, 0),
            driveToPosAutoCommand(-0.5, -1),
            driveToPosAutoCommand(0, 0),
            driveToPosAutoCommand(0.5, -1),
            driveToPosAutoCommand(0, 0)
        );
    }

    // Normal Commands:

    /**
     * @deprecated With the use of {@link #shootAndDriveOffTarmac()}, there
     * is not much need for this method.
     * <p>
     * Returns the command to just drive off of the tarmac.
     * 
     * @return The command as a 
     * {@link SequentialCommandGroup} object.
     */
    public static Command basicDriveOffTarmac() {
        return new SequentialCommandGroup(
            voltageDriveCommand(-5, -5),
            new WaitCommand(3),
            stopDriveCommand()
        );
    }

    /**
     * @deprecated With the use of odometry, there is not much need
     * for the manual driving to different positions used in this
     * method for autonomous.
     * <p>
     * Returns the command for a two ball that aims to shoot a ball, 
     * drive off of the tarmac, intake a ball behind us, and shoot said ball.
     * 
     * @return The command as a 
     * {@link SequentialCommandGroup} object.
     */
    public static Command shootAndDriveOffTarmac() {
        return new SequentialCommandGroup(
            // ShooterCommands.autoFlywheelPos(),
            ShooterCommands.autoShoot(32.0, 1, false),

            // Makes the robot go fast here to make the intake fall down.
            oldDriveDistanceAutoCommand(500, Direction.FORWARD).withTimeout(0.1),

            oldAngularTurnAutoCommand(0.20, 150, Direction.LEFT).withTimeout(2.5),
            oldGetBallAutoCommand(60, Direction.FORWARD).withTimeout(2.25),
            oldAngularTurnAutoCommand(0.2, 25, Direction.RIGHT).withTimeout(2.5),
            // ShooterCommands.oldAutoFlywheelPos(),
            ShooterCommands.autoShoot(37.0, 1, false)
        );
    }

    /**
     * @deprecated With the use of odometry, there is not much need
     * for the manual driving to different positions used in this 
     * method for autonomous.
     * <p>
     * Returns the command for a three ball auto that aims to drive 
     * off of the tarmac, intake a ball behind us, shoot two balls, 
     * intake another ball, and then shoot said ball.
     *  
     * @return The command as a 
     * {@link SequentialCommandGroup} object.
     */
    public static Command threeBallTest() {
        return new SequentialCommandGroup(
            oldDriveDistanceAutoCommand(500, Direction.FORWARD).withTimeout(0.1),
            oldAngularTurnAutoCommand(0.20, 150, Direction.LEFT).withTimeout(2.5),
            oldGetBallAutoCommand(20, Direction.FORWARD).withTimeout(2.25),
            oldAngularTurnAutoCommand(0.2, 25, Direction.RIGHT).withTimeout(2.5),
            ShooterCommands.autoShoot(38.0, 2, false),
            oldAngularTurnAutoCommand(0.2, 80, Direction.RIGHT).withTimeout(2.5),
            oldDriveDistanceAutoCommand(1000, Direction.FORWARD).withTimeout(0.1),
            oldGetBallAutoCommand(20, Direction.FORWARD).withTimeout(2.25),
            oldAngularTurnAutoCommand(0.2, 80, Direction.LEFT),
            ShooterCommands.autoShoot(38.0, 1, false)
        );
    }

    /**
     * Returns the command that drives the distance to the
     * position calculated by the odometry code
     * ({@link DriveSystem#driveToPosition(double, double)}).
     * 
     * @return The command as a 
     * {@link FunctionalCommand} object.
     */
    public static Command driveDistanceAutoCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> Robot.drive.driveDistance(),
            (interrupt) -> Robot.drive.pidDrive(0.0, ControlType.kPosition, Constants.POSITION_SLOT_ID),
            () -> Robot.drive.reachedPosition(),
            Robot.drive
        );
    }

    /**
     * Returns the command that turns to the angle towards
     * the position calculated by the odometry code
     * ({@link DriveSystem#driveToPosition(double, double)}).
     * 
     * @return The command as a 
     * {@link FunctionalCommand} object.
     */
    public static Command turnCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> Robot.drive.turn(0.1),
            (interrupt) -> Robot.drive.percent(0.0, 0.0),
            () -> Robot.drive.reachedTurn(),
            Robot.drive
        );
    }

    /**
     * Returns the command that runs the odometry code
     * that will calculate the angle needed to turn to
     * the target position and the distance needed to drive
     * to the target position after turning the calculated
     * angle.
     * 
     * @param xMeters The x-coordinate of the target position in meters.
     * @param yMeters The y-coordinate of the target position in meters.
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command driveToPosition(double xMeters, double yMeters) {
        return new InstantCommand(
            () -> Robot.drive.driveToPosition(xMeters, yMeters),
            Robot.drive
        );
    }

    /**
     * Returns the command that will drive to a target position 
     * based on what the odometry code calculates.
     * 
     * @param xMeters The x-coordinate of the target position in meters.
     * @param yMeters The y-coordinate of the target position in meters.
     * @return The command as a 
     * {@link SequentialCommandGroup} object.
     */
    public static Command driveToPosAutoCommand(double xMeters, double yMeters) {
        return new SequentialCommandGroup(
            driveToPosition(xMeters, yMeters),
            turnCommand(),
            driveDistanceAutoCommand()
        );
    }

    /**
     * Returns the command that will drive to a target position
     * with the robot's intake on to intake the ball at said
     * target position.
     * 
     * @param xMeters The x-coordinate of the target position in meters.
     * @param yMeters The y-coordinate of the target position in meters.
     * @return The command as a
     * {@link ParallelCommandGroup} object.
     */
    public static Command getBallAutoCommand(double xMeters, double yMeters) {
        return new ParallelCommandGroup(
            driveToPosAutoCommand(xMeters, yMeters),
            IntakeCommands.intakeRunCommand()
        );
    }

    public static Command oldGetBallAutoCommand(double distance, Direction direction) {
        return new ParallelCommandGroup(
            oldDriveDistanceAutoCommand(distance, direction),
            IntakeCommands.intakeRunCommand()
            // ShooterCommands.flywheelRunCommand()
        );
    }

    /**
     * @deprecated With the use of odometry, there is not much need
     * for the manual driving to different positions used in this
     * method for autonomous.
     * <p>
     * Returns the command that will drive a curve based on the 
     * passed in values for the left and right sides of the drive train
     * and the passed in direction to drive in.
     * 
     * @param left The distance for the left side of the drive train to drive (inches).
     * @param right The distance for the right side of the drive train to drive (inches).
     * @param direction The direction for the robot to drive (forwards or backwards).
     * @return The command as a 
     * {@link FunctionalCommand} object.
     */
    public static Command oldCurveTurnAutoCommand(double left, double right, Direction direction) {
        return new FunctionalCommand(
            () -> {}, 
            () -> Robot.drive.driveCurve(left, right, direction), 
            (interrupt) -> Robot.drive.tank(0,0), 
            () -> Robot.drive.reachedCurve(left, right), 
            Robot.drive
        );
    }

    /**
     * @deprecated With the use of odometry, there is not much need
     * for the manual driving to different positions used in this 
     * method for autonomous.
     * <p>
     * Returns the command that will drive the robot in a circle based
     * on the angle (determines arc of the circle to drive), the direction, 
     * and the radius of the circle.
     * 
     * @param angle The angle that determines the arc of the circle to drive.
     * @param direction The direction to drive (left or right).
     * @param radius The radius of the circle (inches).
     * @return The command as a
     * {@link FunctionalCommand} object.
     */
    public static Command oldCircleTurnAutoCommand(double angle, Direction direction, double radius) {
        return new FunctionalCommand(
            () -> {},
            () -> Robot.drive.driveCircle(angle, direction, radius), 
            (interrupted) -> Robot.drive.tank(0,0), 
            () -> Robot.drive.reachedCircle(angle, radius, direction), 
            Robot.drive
        );
    }

    /**
     * Returns the command that will activate the flywheel motors.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command activateFlywheel() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelOut(), 
            Robot.shooter
        );
    }

    /**
     * @deprecated With the use of odometry, there is not much need
     * for the manual driving to different positions used in this
     * method for autonomous.
     * <p>
     * Returns the command that will drive the robot the passed in
     * distance in the passed in direction.
     * 
     * @param distance The distance for the robot to drive (inches).
     * @param direction The direction for the robot to drive in (forwards or backwards).
     * @return The command as a
     * {@link FunctionalCommand} object.
     */
	public static Command oldDriveDistanceAutoCommand(double distance, Direction direction) {
		return new FunctionalCommand(
			() -> Robot.drive.resetPosition(),
			() -> Robot.drive.oldDriveDistance(distance, direction),
			(interrupt) -> Robot.drive.percent(0.0, 0.0),
			() -> Robot.drive.oldReachedPosition(),
			Robot.drive
		);
	}

    /**
     * @deprecated With the use of odometry, there is not much need
     * for the manual turing to different angles used in this
     * method for autonomous.
     * <p>
     * Returns the command that will turn the robot at the passed in
     * speed to the passed in angle in the passed in direction.
     * 
     * @param speed The speed for the robot to turn at (percent of throttle).
     * @param angle The angle for the robot to turn to (degrees).
     * @param direction The direction for the robot to turn in (left or right).
     * @return The command as a
     * {@link FunctionalCommand} object.
     */
	public static Command oldAngularTurnAutoCommand(double speed, double angle, Direction direction) {
		return new FunctionalCommand(
			() -> {},
			() -> Robot.drive.oldTurn(speed, direction),
			(interrupt) -> Robot.drive.percent(0.0, 0.0),
			() -> Robot.drive.oldReachedTurn(angle),
			Robot.drive
		);
	}

    /**
     * Returns the command that drives the left and right sides 
     * of the drive train at the passed in voltage levels.
     * 
     * @param leftVolt The voltage value for the left side of the drive train.
     * @param rightVolt The voltage value for the right side of the drive train.
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command voltageDriveCommand(double leftVolt, double rightVolt) {
        return new InstantCommand(
            () -> Robot.drive.voltage(leftVolt, rightVolt),
            Robot.drive
        );
    }

    /**
     * Returns the command that stops the robot via setting
     * the PID setpoint for the drive train to 0 RPM via
     * the {@link DriveSystem#tank(double, double)} method.
     * 
     * @return The command as an
     * {@link InstantCommand} object.
     */
    public static Command stopDriveCommand() {
        return new InstantCommand(
            () -> Robot.drive.tank(0, 0),
            Robot.drive
        );
    }



    // Autonomous methods with exact ball positions.

    // public static Command threeBallAutoBlue() {
    //     return new SequentialCommandGroup(
    //         ShooterCommands.autoShoot(20, 1),
    //         getBallAutoCommand(-25.91/DriveSystem.INCHES_PER_METER, -149.79/DriveSystem.INCHES_PER_METER),
    //         getBallAutoCommand(-124.95/DriveSystem.INCHES_PER_METER, -87.30/DriveSystem.INCHES_PER_METER),       
    //         IntakeCommands.intakeStopCommand(),
    //         oldAngularTurnAutoCommand(0.2, 90, Direction.RIGHT),
    //         ShooterCommands.autoShoot(35, 2)
    //     );
    // }

    // public static Command threeBallAutoRed() {
    //     return new SequentialCommandGroup(
    //         ShooterCommands.autoShoot(20, 1),
    //         getBallAutoCommand(25.91/DriveSystem.INCHES_PER_METER, 149.79/DriveSystem.INCHES_PER_METER),
    //         getBallAutoCommand(149.23/DriveSystem.INCHES_PER_METER, 32.77/DriveSystem.INCHES_PER_METER),
    //         IntakeCommands.intakeStopCommand(),
    //         oldAngularTurnAutoCommand(0.2, 90, Direction.RIGHT),
    //         ShooterCommands.autoShoot(35, 2)
    //     );
    // }

    // public static Command twoBallAutoTopRed() {
    //     return new SequentialCommandGroup(
    //         ShooterCommands.autoShoot(20, 1),
    //         getBallAutoCommand(25.91/DriveSystem.INCHES_PER_METER, 149.79/DriveSystem.INCHES_PER_METER),
    //         IntakeCommands.intakeStopCommand(),
    //         angularTurnAutoCommand(0.2, 180, Direction.LEFT),
    //         ShooterCommands.autoShoot(35, 1)
    //     );
    // }

    // public static Command twoBallAutoBottomBlue() {
    //     return new SequentialCommandGroup(
    //         ShooterCommands.autoShoot(20, 1),
    //         getBallAutoCommand(-25.91/DriveSystem.INCHES_PER_METER, -149.79/DriveSystem.INCHES_PER_METER),
    //         IntakeCommands.intakeStopCommand(),
    //         oldAngularTurnAutoCommand(0.2, 180, Direction.RIGHT),
    //         ShooterCommands.autoShoot(35, 1)
    //     );
    // }

    // public static Command twoBallAutoBottomRed() {
    //     return new SequentialCommandGroup(
    //         ShooterCommands.autoShoot(20, 1),
    //         getBallAutoCommand(88.30/DriveSystem.INCHES_PER_METER, -125.95/DriveSystem.INCHES_PER_METER),
    //         IntakeCommands.intakeStopCommand(),
    //         angularTurnAutoCommand(0.2, 180, Direction.LEFT),
    //         ShooterCommands.autoShoot(35, 1)
    //     );
    // }

    // public static Command twoBallAutoTopBlue() {
    //     return new SequentialCommandGroup(
    //         ShooterCommands.autoShoot(20, 1),
    //         getBallAutoCommand(-129.40/DriveSystem.INCHES_PER_METER, 82.69/DriveSystem.INCHES_PER_METER),
    //         IntakeCommands.intakeStopCommand(),
    //         angularTurnAutoCommand(0.2, 180, Direction.LEFT),
    //         ShooterCommands.autoShoot(35, 1)
    //     );
    // }
}