package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class MidClimbCommands {

  public MidClimbCommands() {}

  public static Command climberDown() {
    return new InstantCommand(
      () -> Robot.midClimber.climberDown(),
      Robot.midClimber
    );
  }

  public static Command climberUp() {
    return new InstantCommand(
      () -> Robot.midClimber.climberUp(),
      Robot.midClimber
    );
  }

  public static Command climberStop() {
    return new InstantCommand(
      () -> Robot.midClimber.climberStop(),
      Robot.midClimber
    );
  }
}
