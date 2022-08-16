package frc.robot;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HighClimbSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.MidClimbSystem;
import frc.robot.subsystems.ShooterSystem;

import frc.robot.tests.Test;
import frc.robot.tests.DrivePIDTest;
import frc.robot.tests.DriveOdometryTest;
import frc.robot.tests.ShooterHoodTest;
import frc.robot.tests.ShooterVisionTest;
import frc.robot.tests.ShooterAutoShootTest;

public class Robot extends TimedRobot {
  	private Logger stateLogger = LogManager.getLogger("robot_state");
	private Logger robotLogger = LogManager.getLogger("robot");

	private ObjectMapper mapper = new ObjectMapper();

	private Command autoCommand = null;

	private RobotContainer container = null;

	/**
   * Tracks the current state of the robot
   */
  	private RobotState robotState = null;

	/**
	 * Thread pool for handling interval based tasks that are outside of the
	 * typical robot lifecyle. In other words, things that should not be on the
	 * robot's main loop/thread.
	 */
	private ScheduledThreadPoolExecutor executor = new ScheduledThreadPoolExecutor(2);

	/**
	 * The robot's drive train subsystem.
	 */
	public final static DriveSystem drive = new DriveSystem();

	/**
	 * The robot's intake subsystem.
	 */
	public final static IntakeSystem intake = new IntakeSystem();

	/**
	 * The robot's mid rung climber subsystem.
	 */
	public final static MidClimbSystem midClimber = new MidClimbSystem();

  /**
	 * The robot's high/transversal rung climber subsystem.
	 */
	public final static HighClimbSystem highClimber = new HighClimbSystem();

	/**
	 * The robot's shooter subsystem.
	 */
	public final static ShooterSystem shooter = new ShooterSystem();

	/**
	 * The array that stores all of the tests to 
	 * run in Test mode.
	 */
	private static ArrayList<Test> tests = new ArrayList<Test>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button
		// bindings, and put our autonomous chooser on the dashboard.
		this.container = new RobotContainer();

		drive.resetAngle();
		drive.resetPosition();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		drive.setPIDF(drive.getLeftPIDCont(), DriveSystem.Mode.AUTONOMOUS);
		drive.setPIDF(drive.getRightPIDCont(), DriveSystem.Mode.AUTONOMOUS);

		this.autoCommand = this.container.getAutonomousCommand("shootAndDriveOffTarmac");

		// schedule the autonomous command (example)
		if (this.autoCommand != null) {
			this.autoCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		drive.setPIDF(drive.getLeftPIDCont(), DriveSystem.Mode.TELEOP);
		drive.setPIDF(drive.getRightPIDCont(), DriveSystem.Mode.TELEOP);

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (this.autoCommand != null) {
			this.autoCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		// CommandScheduler.getInstance().cancelAll();

		tests.add(new DrivePIDTest(DriveSystem.Mode.AUTONOMOUS));
		tests.add(new DriveOdometryTest());
		tests.add(new ShooterHoodTest());
		tests.add(new ShooterVisionTest());
		tests.add(new ShooterAutoShootTest());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		for (Test t : tests) {
			t.run();
		}
	}

	@Override
	public void startCompetition() {
		super.startCompetition();
	}
}
