package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import java.util.*;
import static java.lang.Math.*;

public class AutoCommands {

    public AutoCommands() {
    }
        
    public static Command driveDistanceAutoCommand(double distance, Direction direction) {
        return new FunctionalCommand(
            () -> Robot.drive.resetPosition(),
            () -> Robot.drive.driveDistance(distance, direction),
            (interrupt) -> Robot.drive.tank(0, 0),
            () -> Robot.drive.reachedPosition(),
            Robot.drive
            );
    }

    public static Command angularTurnAutoCommand(double speed, double angle, Direction direction) {
        return new FunctionalCommand(
            () -> Robot.drive.resetAngle(),
            () -> Robot.drive.turn(speed, direction),
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
            () -> Robot.shooter.resetPosition(),
            () -> Robot.shooter.flywheelAndFlywheelIntakeOut(),
            (interrupt) -> Robot.shooter.flywheelAndFlywheelIntakeStop(),
            () -> Robot.shooter.flywheelDoneShootBalls(balls),
            Robot.shooter
        );
    }

    public static Command flywheelWait() {
        return new WaitCommand(3);
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
            flywheelWait(),
            ballExitCommand(balls)
        );
    }

//Mesurements in Inches and Radians, angle and distance measurements are needed to be tested.

    public static Command meuleDeFromage() {
        return new SequentialCommandGroup(
            // shootBallAutoCommand(1),
            driveDistanceAutoCommand(41, Direction.BACKWARD),
            angularTurnAutoCommand(0.2, (Math.PI/2), Direction.LEFT),
            circleTurnAutoCommand(0.1, (Math.PI/6), Direction.RIGHT, 153),
            angularTurnAutoCommand(0.2, (Math.PI/2), Direction.RIGHT)
            // shootBallAutoCommand(2)
        );
    }

    public static Command trancheDeFromage() {
        return new SequentialCommandGroup(
            shootBallAutoCommand(1),
            driveDistanceAutoCommand(41, Direction.BACKWARD),
            angularTurnAutoCommand(0.2, (Math.PI/2), Direction.RIGHT),
            circleTurnAutoCommand(0.1, (Math.PI/18), Direction.LEFT, 153),
            angularTurnAutoCommand(0.2, (Math.PI/2), Direction.LEFT),
            shootBallAutoCommand(1)
        );
    }


    public static Command meuleDeFromagePourBebe() {
        return new SequentialCommandGroup(
            shootBallAutoCommand(1),
            driveDistanceAutoCommand(41, Direction.BACKWARD),
            angularTurnAutoCommand(0.2, (Math.PI/2), Direction.RIGHT),
            circleTurnAutoCommand(0.1, (Math.PI/18), Direction.LEFT, 153),
            angularTurnAutoCommand(0.2, (Math.PI/2), Direction.LEFT),
            shootBallAutoCommand(1)
        );
    }

}
