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
	private Joystick controllerOne = new Joystick(Constants.DRIVER_CONTROLLER);
  private Joystick controllerTwo = new Joystick(Constants.SECOND_DRIVER_CONTROLLER);

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
			this.controllerOne.getRawAxis(Constants.LEFT_STICK_AXIS),
			this.controllerOne.getRawAxis(Constants.RIGHT_STICK_AXIS)
		),
		Robot.drive
	);

    // Creates a hash map of commands for the robot
    private Map<String, Command> commands = Map.ofEntries(
      Map.entry("basicDriveOffTarmac", AutoCommands.basicDriveOffTarmac()),
      Map.entry("shootAndDriveOffTarmac", AutoCommands.shootAndDriveOffTarmac()),
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
    JoystickButton toggleTurboButton = new JoystickButton(controllerOne, Constants.BUTTON_RB);
    toggleTurboButton.whenPressed(DriveCommands.toggleTurbo());

    JoystickButton toggleOuttakeButton = new JoystickButton(controllerOne, Constants.BUTTON_A);
    toggleOuttakeButton.whenPressed(IntakeCommands.intakeOutCommand());
    toggleOuttakeButton.whenReleased(IntakeCommands.intakeStopCommand());

    JoystickButton quarterVelocityButton = new JoystickButton(controllerOne, Constants.BUTTON_LB);
    quarterVelocityButton.whenPressed(DriveCommands.quarterTrue());
    toggleOuttakeButton.whenReleased(DriveCommands.quarterFalse());

    JoystickButton hoodUpButton = new JoystickButton(controllerTwo, Constants.BUTTON_Y);
    hoodUpButton.whenPressed(ShooterCommands.hoodUp());
    hoodUpButton.whenReleased(ShooterCommands.hoodStop());

    JoystickButton hoodDownButton = new JoystickButton(controllerTwo, Constants.BUTTON_A);
    hoodDownButton.whenPressed(ShooterCommands.hoodDown());
    hoodDownButton.whenReleased(ShooterCommands.hoodStop());

    JoystickButton oldAutoFlywheel = new JoystickButton(controllerTwo, Constants.BUTTON_B);
    oldAutoFlywheel.whenPressed(ShooterCommands.oldAutoFlywheelPos());

    JoystickButton setHoodShootButton = new JoystickButton(controllerTwo, Constants.BUTTON_X);
    setHoodShootButton.whenPressed(ShooterCommands.setHoodAngleCommand(17.9));
    
    JoystickButton climbUpButton = new JoystickButton(controllerOne, Constants.BUTTON_BACK);
    climbUpButton.whenPressed(MidClimbCommands.climberUp());
    climbUpButton.whenReleased(MidClimbCommands.climberStop());

    JoystickButton climbDownButton = new JoystickButton(controllerOne, Constants.BUTTON_START);
    climbDownButton.whenPressed(MidClimbCommands.climberDown());
    climbDownButton.whenReleased(MidClimbCommands.climberStop());
  }

  private void changeHoodAngle() {
    if (controllerTwo.getPOV() == 0) {
      ShooterCommands.setHoodAngleCommand(0);
    } else if (controllerTwo.getPOV() == 90) {
      ShooterCommands.setHoodAngleCommand(15);
    } else if (controllerTwo.getPOV() == 180) {
      ShooterCommands.setHoodAngleCommand(30);
    } else if (controllerTwo.getPOV() == 270) {
      ShooterCommands.setHoodAngleCommand(45);
    }
  }

  private void toggleIntake() {
    if (controllerOne.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) > 0.3) {
      Robot.intake.intakeIn();
    }
    else {
      Robot.intake.intakeStop();
    }
  }

  private void toggleFlywheelIntake() {
    if (controllerTwo.getRawAxis(Constants.LEFT_TRIGGER_AXIS) > 0.3) {
      Robot.shooter.flywheelIntakeIn();
    }
    else {
      Robot.shooter.flywheelIntakeStop();
    }
  }

  private void toggleFlywheel() {
    if (controllerTwo.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) > 0.3) {
      Robot.shooter.flywheelOut();
    }
    else {
      Robot.shooter.flywheelStop();
    }
  }

  private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

    scheduler.setDefaultCommand(Robot.drive, defaultDriveCommand);

    scheduler.addButton(
      () -> changeHoodAngle()
    );
    
    scheduler.addButton(
      () -> toggleIntake()
    );

    scheduler.addButton(
      () -> toggleFlywheelIntake()
    );

    scheduler.addButton(
      () -> toggleFlywheel()
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
