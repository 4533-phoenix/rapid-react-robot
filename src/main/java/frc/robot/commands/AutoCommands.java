package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSystem;

public class AutoCommands {

    public AutoCommands() {
    }

    // Test Commands:
    public static Command testDriveToPos() {
        return new SequentialCommandGroup(
            driveToPosAutoCommand(1, 2),
            driveToPosAutoCommand(0, 0),
            driveToPosAutoCommand(-1, 2),
            driveToPosAutoCommand(0, 0),
            driveToPosAutoCommand(-1, -2),
            driveToPosAutoCommand(0, 0),
            driveToPosAutoCommand(1, -2),
            driveToPosAutoCommand(0, 0)
        );
    }

    public static Command driveOffTarmac() {
        return oldDriveDistanceAutoCommand(50, Direction.BACKWARD);
    }

    // Normal Commands:    
    public static Command driveDistanceAutoCommand() {
        return new FunctionalCommand(
            () -> Robot.drive.getAngle(),
            () -> Robot.drive.driveDistance(),
            (interrupt) -> Robot.drive.tank(0, 0),
            () -> Robot.drive.reachedPosition(),
            Robot.drive
        );
    }

    public static Command turnCommand() {
        return new FunctionalCommand(
            () -> Robot.drive.getAngle(),
            () -> Robot.drive.turn(0.1),
            (interrupt) -> Robot.drive.tank(0, 0),
            () -> Robot.drive.reachedTurn(),
            Robot.drive
        );
    }

    public static Command driveToPosition(double xMeters, double yMeters) {
        return new InstantCommand(
            () -> Robot.drive.driveToPosition(xMeters, yMeters),
            Robot.drive
        );
    }

    public static Command driveToPosAutoCommand(double xMeters, double yMeters) {
        return new SequentialCommandGroup(
            driveToPosition(xMeters, yMeters),
            turnCommand(),
            driveDistanceAutoCommand(),
            oldAngularTurnAutoCommand(0.1, 0.0, Direction.RIGHT)
        );
    }

    public static Command angularTurnAutoCommand(double speed, double angle, Direction direction) {
        return new FunctionalCommand(
            () -> Robot.drive.resetAngle(),
            () -> Robot.drive.turn(speed),
            (interrupt) -> Robot.drive.tank(0, 0),
            () -> Robot.drive.getAngle() >= angle,
            Robot.drive
        );
    }

    public static Command curveTurnAutoCommand(double left, double right, Direction direction) {
        return new FunctionalCommand(
            () -> Robot.drive.resetPosition(), 
            () -> Robot.drive.driveCurve(left, right, direction), 
            (interrupt) -> Robot.drive.tank(0,0), 
            () -> Robot.drive.reachedCurve(left, right), 
            Robot.drive
        );
    }

    public static Command circleTurnAutoCommand(double speed, double angle, Direction direction, double radius) {
        return new FunctionalCommand(
            () -> Robot.drive.resetPosition(),
            () -> Robot.drive.driveCircle(speed, angle, direction, radius), 
            (interrupted) -> Robot.drive.tank(0,0), 
            () -> Robot.drive.reachedCircle(angle, radius, direction), 
            Robot.drive
        );
    }

    public static Command ballExitCommand(int balls) {
        return new FunctionalCommand(
            () -> Robot.shooter.setFlywheelReset(),
            () -> Robot.shooter.flywheelAndFlywheelIntakeOut(),
            (interrupt) -> Robot.shooter.flywheelAndFlywheelIntakeStop(),
            () -> Robot.shooter.flywheelDoneShootBalls(balls),
            Robot.shooter
        );
    }

    public static Command flywheelWait(int seconds) {
        return new WaitCommand(seconds);
    }

    public static Command activateFlywheel() {
        return new InstantCommand(
            () -> Robot.shooter.flywheelOut(), 
            Robot.shooter
        );
    }

    public static Command shootBallAutoCommand(int balls) {
        return new SequentialCommandGroup(
            activateFlywheel(),
            flywheelWait(3),
            ballExitCommand(balls)
        );
    }

	public static Command oldDriveDistanceAutoCommand(double distance, Direction direction) {
		return new FunctionalCommand(
			() -> Robot.drive.resetPosition(),
			() -> Robot.drive.oldDriveDistance(distance, direction),
			(interrupt) -> Robot.drive.percent(0, 0),
			() -> Robot.drive.oldReachedPosition(),
			Robot.drive
		);
	}

	public static Command oldAngularTurnAutoCommand(double speed, double angle, Direction direction) {
		return new FunctionalCommand(
			() -> Robot.drive.resetAngle(),
			() -> Robot.drive.oldTurn(speed, direction),
			(interrupt) -> Robot.drive.tank(0, 0),
			() -> Robot.drive.oldReachedTurn(angle),
			Robot.drive
		);
	}

    public static Command intakeInAutoCommand() {
        return new InstantCommand(
            () -> Robot.intake.intakeIn(),
            Robot.intake
        );
    }

    public static Command intakeStopAutoCommand() {
        return new InstantCommand(
            () -> Robot.intake.intakeStop(),
            Robot.intake
        );
    }



//Mesurements in Inches and Degrees, all auto needs to be tested. DriveSystem.INCHES_PER_METER is inches to meters

    public static Command threeBallAutoBlue() {
        return new SequentialCommandGroup(
            shootBallAutoCommand(1),
            intakeInAutoCommand(),            
            driveToPosAutoCommand(-25.91/DriveSystem.INCHES_PER_METER,-149.79/DriveSystem.INCHES_PER_METER),
            driveToPosAutoCommand(-124.95/DriveSystem.INCHES_PER_METER,-87.30/DriveSystem.INCHES_PER_METER),
            intakeStopAutoCommand(),
            oldAngularTurnAutoCommand(0.2, 90, Direction.RIGHT),
            shootBallAutoCommand(2)
        );
    }

    public static Command threeBallAutoRed() {
        return new SequentialCommandGroup(
            shootBallAutoCommand(1),
            intakeInAutoCommand(),
            driveToPosAutoCommand(25.91/DriveSystem.INCHES_PER_METER,149.79/DriveSystem.INCHES_PER_METER),
            driveToPosAutoCommand(149.23/DriveSystem.INCHES_PER_METER,32.77/DriveSystem.INCHES_PER_METER),
            intakeStopAutoCommand(),
            oldAngularTurnAutoCommand(0.2, 90, Direction.RIGHT),
            shootBallAutoCommand(2)
        );
    }

    public static Command twoBallAutoTopRed() {
        return new SequentialCommandGroup(
            shootBallAutoCommand(1),
            intakeInAutoCommand(),
            driveToPosAutoCommand(25.91/DriveSystem.INCHES_PER_METER,149.79/DriveSystem.INCHES_PER_METER),
            intakeStopAutoCommand(),
            angularTurnAutoCommand(0.2, 180, Direction.LEFT),
            shootBallAutoCommand(1)
        );
    }

    public static Command twoBallAutoBottomBlue() {
        return new SequentialCommandGroup(
            shootBallAutoCommand(1),
            intakeInAutoCommand(),
            driveToPosAutoCommand(-25.91/DriveSystem.INCHES_PER_METER,-149.79/DriveSystem.INCHES_PER_METER),
            intakeStopAutoCommand(),
            oldAngularTurnAutoCommand(0.2, 180, Direction.RIGHT),
            shootBallAutoCommand(1)
        );
    }

    public static Command twoBallAutoBottomRed() {
        return new SequentialCommandGroup(
            shootBallAutoCommand(1),
            intakeInAutoCommand(),
             driveToPosAutoCommand(88.30/DriveSystem.INCHES_PER_METER,-125.95/DriveSystem.INCHES_PER_METER),
            intakeStopAutoCommand(),
            angularTurnAutoCommand(0.2, 180, Direction.LEFT),
            shootBallAutoCommand(1)
        );
    }

    public static Command twoBallAutoTopBlue() {
        return new SequentialCommandGroup(
            shootBallAutoCommand(1),
            intakeInAutoCommand(),
             driveToPosAutoCommand(-129.40/DriveSystem.INCHES_PER_METER,82.69/DriveSystem.INCHES_PER_METER),
            intakeStopAutoCommand(),
            angularTurnAutoCommand(0.2, 180, Direction.LEFT),
            shootBallAutoCommand(1)
        );
    }
}