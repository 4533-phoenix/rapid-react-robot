package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class HighClimbCommands {
    public HighClimbCommands() {}

    public static Command highClimberDown() {
        return new InstantCommand(
            () -> Robot.highClimber.highClimberDown(),
            Robot.highClimber
        );
    }

    public static Command highClimberUp() {
        return new InstantCommand(
            () -> Robot.highClimber.highClimberUp(),
            Robot.highClimber
        );
    } 
    
    public static Command highClimberStop() {
        return new InstantCommand(
            () -> Robot.highClimber.highClimberStop(),
            Robot.highClimber
        );
    }
}
