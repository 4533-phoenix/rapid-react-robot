package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class ShooterCommands {

  public ShooterCommands() {}

  public static Command flywheelOutCommand() {
    return new InstantCommand(() -> Robot.shooter.flywheelOut(), Robot.shooter);
  }

  public static Command flywheelInCommand() {
    return new InstantCommand(() -> Robot.shooter.flywheelIn(), Robot.shooter);
  }

  public static Command flywheelStopCommand() {
    return new InstantCommand(
      () -> Robot.shooter.flywheelStop(),
      Robot.shooter
    );
  }

  public static Command flywheelIntakeInCommand() {
    return new InstantCommand(
      () -> Robot.shooter.flywheelIntakeIn(),
      Robot.shooter
    );
  }

  public static Command flywheelIntakeOutCommand() {
    return new InstantCommand(
      () -> Robot.shooter.flywheelIntakeOut(),
      Robot.shooter
    );
  }

  public static Command flywheelIntakeStopCommand() {
    return new InstantCommand(
      () -> Robot.shooter.flywheelIntakeStop(),
      Robot.shooter
    );
  }

  public static Command hoodUp() {
    return new InstantCommand(() -> Robot.shooter.hoodUp(), Robot.shooter);
  }

  public static Command hoodDown() {
    return new InstantCommand(() -> Robot.shooter.hoodDown(), Robot.shooter);
  }

  public static Command hoodStop() {
    return new InstantCommand(() -> Robot.shooter.hoodStop(), Robot.shooter);
  }

  public static Command autoFlywheelPos() {
    return new FunctionalCommand(
      () -> Robot.shooter.setFlywheelReset(),
      () -> Robot.shooter.setFlywheelPos(),
      interrupt -> Robot.shooter.stopFlywheelPos(),
      () -> Robot.shooter.flywheelReachedPosition(),
      Robot.shooter
    );
  }

  public static Command setHoodAngleCommand(double angle) {
    return new FunctionalCommand(
      () -> Robot.shooter.getHoodAngle(),
      () -> Robot.shooter.setHoodAngle(angle),
      interrupt -> Robot.shooter.hoodStop(),
      () -> Robot.shooter.hoodReachedPosition(),
      Robot.shooter
    );
  }

  public static Command oldAutoFlywheelPos() {
    return new FunctionalCommand(
      () -> Robot.shooter.setFlywheelReset(),
      () -> Robot.shooter.autoTurretSwivel(),
      interrupt -> Robot.shooter.stopFlywheelPos(),
      () -> Robot.shooter.turretReachedPosition(),
      Robot.shooter
    );
  }
}
