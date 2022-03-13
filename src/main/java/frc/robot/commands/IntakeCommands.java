package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class IntakeCommands {

  public IntakeCommands() {}

  public static Command intakeInCommand() {
    return new InstantCommand(() -> Robot.intake.intakeIn(), Robot.intake);
  }

  public static Command intakeOutCommand() {
    return new InstantCommand(() -> Robot.intake.intakeOut(), Robot.intake);
  }

  public static Command intakeStopCommand() {
    return new InstantCommand(() -> Robot.intake.intakeStop(), Robot.intake);
  }
}
