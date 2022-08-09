package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    public static Command basicDriveOffTarmac() {
        return new SequentialCommandGroup(
            voltageDriveCommand(-5, -5),
            new WaitCommand(3),
            stopDriveCommand()
        );
    }

    public static Command shootAndDriveOffTarmac() {
        return new SequentialCommandGroup(
            // ShooterCommands.autoFlywheelPos(),
            ShooterCommands.autoShoot(32.0, 1, false),
            oldDriveDistanceAutoCommand(500, Direction.FORWARD).withTimeout(0.1),
            oldAngularTurnAutoCommand(0.20, 150, Direction.LEFT).withTimeout(2.5),
            oldGetBallAutoCommand(20, Direction.FORWARD).withTimeout(2.25),
            oldAngularTurnAutoCommand(0.2, 25, Direction.RIGHT).withTimeout(2.5),
            // ShooterCommands.oldAutoFlywheelPos(),
            ShooterCommands.autoShoot(37.0, 1, false)
        );
    }

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

    // Normal Commands:    
    public static Command driveDistanceAutoCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> Robot.drive.driveDistance(),
            (interrupt) -> Robot.drive.percent(0.0, 0.0),
            () -> Robot.drive.reachedPosition(),
            Robot.drive
        );
    }

    public static Command turnCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> Robot.drive.turn(0.1),
            (interrupt) -> Robot.drive.percent(0.0, 0.0),
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

    public static Command curveTurnAutoCommand(double left, double right, Direction direction) {
        return new FunctionalCommand(
            () -> Robot.drive.resetPosition(), 
            () -> Robot.drive.driveCurve(left, right, direction), 
            (interrupt) -> Robot.drive.tank(0,0), 
            () -> Robot.drive.reachedCurve(left, right), 
            Robot.drive
        );
    }

    public static Command circleTurnAutoCommand(double angle, Direction direction, double radius) {
        return new FunctionalCommand(
            () -> Robot.drive.resetPosition(),
            () -> Robot.drive.driveCircle(angle, direction, radius), 
            (interrupted) -> Robot.drive.tank(0,0), 
            () -> Robot.drive.reachedCircle(angle, radius, direction), 
            Robot.drive
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

	public static Command oldDriveDistanceAutoCommand(double distance, Direction direction) {
		return new FunctionalCommand(
			() -> Robot.drive.resetPosition(),
			() -> Robot.drive.oldDriveDistance(distance, direction),
			(interrupt) -> Robot.drive.percent(0.0, 0.0),
			() -> Robot.drive.oldReachedPosition(),
			Robot.drive
		);
	}

	public static Command oldAngularTurnAutoCommand(double speed, double angle, Direction direction) {
		return new FunctionalCommand(
			() -> {},
			() -> Robot.drive.oldTurn(speed, direction),
			(interrupt) -> Robot.drive.percent(0.0, 0.0),
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

    public static Command voltageDriveCommand(double leftVolt, double rightVolt) {
        return new InstantCommand(
            () -> Robot.drive.voltage(leftVolt, rightVolt),
            Robot.drive
        );
    }

    public static Command stopDriveCommand() {
        return new InstantCommand(
            () -> Robot.drive.tank(0, 0),
            Robot.drive
        );
    }



//Mesurements in Inches and Degrees, all auto needs to be tested. DriveSystem.INCHES_PER_METER is inches to meters

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