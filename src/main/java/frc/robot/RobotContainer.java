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
      Map.entry("testDrivePositionOne", AutoCommands.driveToPosAutoCommand(5, 10)),
      Map.entry("threeBallAutoBottom", AutoCommands.threeBallAutoBottom()),
      Map.entry("twoBallAutoBottom", AutoCommands.twoBallAutoBottom()),
      Map.entry("twoBallAutoTop", AutoCommands.twoBallAutoTop())
    );
  

  // Defines button bindings for commands
  private void configureButtonBindings() {
    JoystickButton toggleTurbo = new JoystickButton(controller, Constants.BUTTON_RB);
    toggleTurbo.whenPressed(DriveCommands.toggleTurbo());
  }

  private void toggleDriveMode() {
    if (controller.getPOV() == 180) {
      Robot.drive.toggleDriveMode();
    }
  }

  private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

    scheduler.setDefaultCommand(Robot.drive, defaultDriveCommand);
    scheduler.addButton(
      () -> toggleDriveMode()
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
