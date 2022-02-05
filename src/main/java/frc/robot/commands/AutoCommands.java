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
        
    // public static Command driveDistanceAutoCommand(double distance, Direction direction) {
    //     return new FunctionalCommand(
    //         () -> Robot.drive.resetPosition(),
    //         () -> Robot.drive.driveDistance(distance, direction),
    //         (interrupt) -> Robot.drive.tank(0, 0),
    //         () -> Robot.drive.reachedPosition(),
    //         Robot.drive
    //         );
    //  }

    public static Command angularTurnAutoCommand(double speed, double angle, Direction direction) {
        return new FunctionalCommand(
            () -> Robot.drive.resetAngle(),
            () -> Robot.drive.turn(speed, direction),
            (interrupt) -> Robot.drive.tank(0, 0),
            () -> Robot.drive.getAngle() >= angle,
            Robot.drive
        );
    }

}
