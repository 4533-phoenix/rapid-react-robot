package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class AutoCommands {

    public AutoCommands() {
    }

    // Test Commands:
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
			(interrupt) -> Robot.drive.tank(0, 0),
			() -> Robot.drive.reachedPosition(),
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



//Mesurements in Inches and Degrees, all auto needs to be tested.

    public static Command threeBallAutoBlue() {
        return new SequentialCommandGroup(
            // shootBallAutoCommand(1),
            driveToPosAutoCommand(0,-150.76),
            oldAngularTurnAutoCommand(0.2, 90, Direction.LEFT),
            //intakeInAutoCommand(),
            circleTurnAutoCommand(0.1, 45, Direction.RIGHT, 153),
            //intakeStopAutoCommand(),
            oldAngularTurnAutoCommand(0.2, 90, Direction.RIGHT)
            // shootBallAutoCommand(2)
        );
    }

    public static Command threeBallAutoRed() {
        return new SequentialCommandGroup(
            // shootBallAutoCommand(1),
            driveToPosAutoCommand(0,150.76),
            oldAngularTurnAutoCommand(0.2, 90, Direction.LEFT),
            //intakeInAutoCommand(),
            circleTurnAutoCommand(0.1, 45, Direction.RIGHT, 153),
            //intakeStopAutoCommand(),
            oldAngularTurnAutoCommand(0.2, 90, Direction.RIGHT)
            // shootBallAutoCommand(2)
        );
    }

    public static Command twoBallAutoTopRed() {
        return new SequentialCommandGroup(
            //shootBallAutoCommand(1),
            driveToPosAutoCommand(0,150.76),
            oldAngularTurnAutoCommand(0.2, 90, Direction.RIGHT),
            //intakeInAutoCommand(),
            driveToPosAutoCommand(41.42,150.76),
            //intakeStopAutoCommand(),
            angularTurnAutoCommand(0.2, 90, Direction.LEFT)
            // shootBallAutoCommand(1)
        );
    }

    public static Command twoBallAutoBottomBlue() {
        return new SequentialCommandGroup(
            // shootBallAutoCommand(1),
            driveToPosAutoCommand(0,-150.76),
            oldAngularTurnAutoCommand(0.2, 90, Direction.LEFT),
            //intakeInAutoCommand(),
            driveToPosAutoCommand(-41.42,-150.76),
            //intakeStopAutoCommand(),
            oldAngularTurnAutoCommand(0.2, 90, Direction.RIGHT)
            // shootBallAutoCommand(1)
        );
    }
}