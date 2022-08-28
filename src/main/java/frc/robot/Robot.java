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

/**
 * The class for the robot.
 * <p>
 * This class includes all of the subsystems, 
 * the autonomous command to be used, the 
 * robot container, PIDF setting for teleop
 * and autonomous, and test running for
 * test mode.
 */
public class Robot extends TimedRobot {
  	private Command autoCommand = null;

	private RobotContainer container = null;

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
	 * This method is run when the robot is first 
	 * started up and should be used for any 
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		/* 
		 * Instantiate our RobotContainer. This will perform 
		 * all our button bindings, and put our autonomous 
		 * chooser on the dashboard.
		 */
		this.container = new RobotContainer();

		drive.resetAngle();
		drive.resetPosition();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/**
	 * This method is called once when disabled mode
	 * is activated.
	 */
	@Override
	public void disabledInit() {
	}

	/**
	 * This method is called periodically when disabled
	 * mode is activated.
	 */
	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This method is called once when autonomous 
	 * mode is activated.
	 */
	@Override
	public void autonomousInit() {
		/*
		 * Sets the autonomous PIDF values for the
		 * drive system.
		 */
		drive.setPIDF(drive.getLeftPIDCont(), DriveSystem.Mode.AUTONOMOUS);
		drive.setPIDF(drive.getRightPIDCont(), DriveSystem.Mode.AUTONOMOUS);

		/*
		 * Gets the autonomous command selected in
		 * the robot container.
		 */
		this.autoCommand = this.container.getAutonomousCommand("shootAndDriveOffTarmac");

		// Schedules the autonomous command.
		if (this.autoCommand != null) {
			this.autoCommand.schedule();
		}
	}

	/**
	 * This method is called periodically when autonomous
	 * mode is activated.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	/**
	 * This method is called once when teleop mode
	 * is activated.
	 */
	@Override
	public void teleopInit() {
		/*
		 * Sets the teleop PIDF values for the
		 * drive system.
		 */
		drive.setPIDF(drive.getLeftPIDCont(), DriveSystem.Mode.TELEOP);
		drive.setPIDF(drive.getRightPIDCont(), DriveSystem.Mode.TELEOP);

		/* 
		 * This makes sure that the autonomous command 
		 * stops running when teleop mode is 
		 * activated. If you want the autonomous 
		 * command to continue running until it ends, 
		 * remove these lines or comment them out.
		 */
		if (this.autoCommand != null) {
			this.autoCommand.cancel();
		}
	}

	/**
	 * This method is called periodically when teleop 
	 * mode is activated.
	 */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		// CommandScheduler.getInstance().cancelAll();

		/*
		 * Adds the tests that will be run
		 * during test mode.
		 */
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
		// Runs the currently activated tests.
		for (Test t : tests) {
			t.run();
		}
	}

	@Override
	public void startCompetition() {
		super.startCompetition();
	}
}
