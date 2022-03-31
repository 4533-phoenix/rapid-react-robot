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
      Map.entry("testDrivePositionOne", AutoCommands.testDriveToPos())
      // Map.entry("threeBallAutoBlue", AutoCommands.threeBallAutoBlue()),
      // Map.entry("threeBallAutoRed", AutoCommands.threeBallAutoRed()),
      // Map.entry("twoBallAutoBottomBlue", AutoCommands.twoBallAutoBottomBlue()),
      // Map.entry("twoBallAutoTopRed", AutoCommands.twoBallAutoTopRed()),
      // Map.entry("twoBallAutoTopBlue",AutoCommands.twoBallAutoTopBlue()),
      // Map.entry("twoBallAutoBottomRed", AutoCommands.twoBallAutoBottomRed())
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
    quarterVelocityButton.whenReleased(DriveCommands.quarterFalse());

    JoystickButton flywheelOut = new JoystickButton(controllerTwo, Constants.BUTTON_RB);
    flywheelOut.whenPressed(ShooterCommands.flywheelRunCommand());
    flywheelOut.whenReleased(ShooterCommands.flywheelStopCommand());

    JoystickButton highClimbUp = new JoystickButton(controllerTwo, Constants.BUTTON_LB);
    highClimbUp.whenPressed(HighClimbCommands.highClimberUp());
    highClimbUp.whenReleased(HighClimbCommands.highClimberStop());

    JoystickButton highClimbDown = new JoystickButton(controllerTwo, Constants.BUTTON_BACK);
    highClimbDown.whenPressed(HighClimbCommands.highClimberDown());
    highClimbDown.whenReleased(HighClimbCommands.highClimberStop());

    JoystickButton hoodSetButton = new JoystickButton(controllerTwo, Constants.BUTTON_Y);
    hoodSetButton.whenPressed(ShooterCommands.setHoodAngleCommand(35));
    // hoodSetButton.whenReleased(ShooterCommands.hoodStop());

    JoystickButton hoodUpButton = new JoystickButton(controllerTwo, Constants.BUTTON_X);
    hoodUpButton.whenPressed(ShooterCommands.setHoodAngleCommand(Robot.shooter.getHoodAngle() + 0.2));
    // hoodUpButton.whenReleased(ShooterCommands.hoodStop());

    JoystickButton hoodDownButton = new JoystickButton(controllerTwo, Constants.BUTTON_A);
    hoodDownButton.whenPressed(ShooterCommands.setHoodAngleCommand(Robot.shooter.getHoodAngle() - 0.2));
    // hoodDownButton.whenReleased(ShooterCommands.hoodStop());

    JoystickButton oldAutoFlywheel = new JoystickButton(controllerTwo, Constants.BUTTON_B);
    oldAutoFlywheel.whenPressed(ShooterCommands.oldAutoFlywheelPos());

    JoystickButton setHoodShootButton = new JoystickButton(controllerTwo, Constants.BUTTON_START);
    setHoodShootButton.whenPressed(ShooterCommands.setHoodAngleCommand(Robot.shooter.getShootHoodAngle()));

    // JoystickButton reverseFlywheelIntakeButton = new JoystickButton(controllerTwo, Constants.BUTTON_LB);
    // reverseFlywheelIntakeButton.whenPressed(ShooterCommands.flywheelIntakeOutCommand());
    // reverseFlywheelIntakeButton.whenReleased(ShooterCommands.flywheelIntakeStopCommand());
    
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
    } 
    else if (controllerTwo.getPOV() == 90) {
      ShooterCommands.setHoodAngleCommand(15);
    } 
    else if (controllerTwo.getPOV() == 180) {
      ShooterCommands.setHoodAngleCommand(30);
    } 
    else if (controllerTwo.getPOV() == 270) {
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
    if (controllerTwo.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) > 0.3) {
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
      () -> changeHoodAngle()
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
