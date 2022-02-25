package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

public class RobotContainer {
  // Initialize the driver controls
	private Joystick controller = new Joystick(Constants.DRIVER_CONTROLLER);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
      // Configure the button bindings
      configureButtonBindings();
  
      // Configure the default commands
      configureDefaultCommands();
  }
  // Initialize the drive command
	private final Command defaultDriveCommand = new RunCommand(
		() -> Robot.drive.tank(
			this.controller.getRawAxis(Constants.LEFT_STICK_AXIS),
			this.controller.getRawAxis(Constants.RIGHT_STICK_AXIS)
		),
		Robot.drive
	);

    // Creates a hash map of commands for the robot
    private Map<String, Command> commands = Map.ofEntries(
      Map.entry("testDrivePositionOne", AutoCommands.testDriveToPos()),
      Map.entry("threeBallAutoBlue", AutoCommands.threeBallAutoBlue()),
      Map.entry("threeBallAutoRed", AutoCommands.threeBallAutoRed()),
      Map.entry("twoBallAutoBottomBlue", AutoCommands.twoBallAutoBottomBlue()),
      Map.entry("twoBallAutoTopRed", AutoCommands.twoBallAutoTopRed()),
      Map.entry("twoBallAutoTopBlue",AutoCommands.twoBallAutoTopBlue()),
      Map.entry("twoBallAutoBottomRed", AutoCommands.twoBallAutoBottomRed())
    );
  

  // Defines button bindings for commands
  private void configureButtonBindings() {
    JoystickButton toggleTurbo = new JoystickButton(controller, Constants.BUTTON_RB);
    toggleTurbo.whenPressed(DriveCommands.toggleTurbo());

    JoystickButton flywheelButton = new JoystickButton(controller, Constants.BUTTON_LB);
    flywheelButton.whenPressed(ShooterCommands.flywheelOutCommand());
    flywheelButton.whenReleased(ShooterCommands.flywheelStopCommand());

    JoystickButton hoodUpButton = new JoystickButton(controller, Constants.BUTTON_Y);
    hoodUpButton.whenPressed(ShooterCommands.hoodUp());
    hoodUpButton.whenReleased(ShooterCommands.hoodStop());

    JoystickButton hoodDownButton = new JoystickButton(controller, Constants.BUTTON_A);
    hoodDownButton.whenPressed(ShooterCommands.hoodDown());
    hoodDownButton.whenReleased(ShooterCommands.hoodStop());

    JoystickButton oldAutoFlywheel = new JoystickButton(controller, Constants.BUTTON_B);
    oldAutoFlywheel.whenPressed(ShooterCommands.oldAutoFlywheelPos());
  }

  private void toggleDriveMode() {
    if (controller.getPOV() == 180) {
      Robot.drive.toggleDriveMode();
    }
  }

  private void toggleIntake() {
    if (controller.getRawAxis(Constants.LEFT_TRIGGER_AXIS) > 0.6) {
      Robot.intake.intakeIn();
    }
    else {
      Robot.intake.intakeStop();
    }
  }

  private void toggleFlywheelIntake() {
    if (controller.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) > 0.6) {
      Robot.shooter.flywheelIntakeIn();
    }
    else {
      Robot.shooter.flywheelIntakeStop();
    }
  }

  private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

    scheduler.setDefaultCommand(Robot.drive, defaultDriveCommand);
    scheduler.addButton(
      () -> toggleDriveMode()
    );
    
    scheduler.addButton(
      () -> toggleIntake()
    );

    scheduler.addButton(
      () -> toggleFlywheelIntake()
    );
	}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String key) {
    // An ExampleCommand will run in autonomous
    Command autoCommand = commands.get(key);

    if (autoCommand == null) {
      return defaultDriveCommand;
    }
    return autoCommand;
  }
}
