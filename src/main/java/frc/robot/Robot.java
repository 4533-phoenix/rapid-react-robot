package frc.robot;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HighClimbSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.MidClimbSystem;
import frc.robot.subsystems.ShooterSystem;

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
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	private PIDController leftPidCont;
	private PIDController rightPidCont;

	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button
		// bindings, and put our autonomous chooser on the dashboard.
		this.container = new RobotContainer();

		leftPidCont = new PIDController(0.0004, 0.0, 0.001);
		rightPidCont = new PIDController(0.0004, 0.0, 0.001);

		SmartDashboard.clearPersistent("PID Controller");

		SendableRegistry.setName(rightPidCont, "DriveSystem", "RightPidController");
		SendableRegistry.setName(leftPidCont, "DriveSystem", "LeftPidController");
	}

	public static void setPIDF(SparkMaxPIDController controller, double p, double i, double d, double iz, double ff, int slotID) {
		controller.setP(p,slotID);
		controller.setI(i,slotID);
		controller.setD(d,slotID);
		controller.setIZone(iz,slotID);
		controller.setFF(ff,slotID);
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
		setPIDF(
			drive.getLeftPIDCont(), 
			DriveSystem.POSITION_P, 
			DriveSystem.POSITION_I, 
			DriveSystem.POSITION_D, 
			DriveSystem.POSITION_I_ZONE, 
			DriveSystem.POSITION_FEED_FORWARD,
			Constants.POSITION_SLOT_ID
		);

		setPIDF(
			drive.getRightPIDCont(), 
			DriveSystem.POSITION_P, 
			DriveSystem.POSITION_I, 
			DriveSystem.POSITION_D, 
			DriveSystem.POSITION_I_ZONE, 
			DriveSystem.POSITION_FEED_FORWARD,
			Constants.POSITION_SLOT_ID
		);

		Robot.drive.resetAngle();
		this.robotLogger.info("reset drive system angle: {}", Robot.drive.getAngle());

		// Robot.drive.resetPosition();

		this.autoCommand = this.container.getAutonomousCommand("driveOffTarmac");

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
		setPIDF(
			drive.getLeftPIDCont(), 
			DriveSystem.VELOCITY_P, 
			DriveSystem.VELOCITY_I, 
			DriveSystem.VELOCITY_D, 
			DriveSystem.VELOCITY_I_ZONE, 
			DriveSystem.VELOCITY_FEED_FORWARD,
			Constants.VELOCITY_SLOT_ID
		);

		setPIDF(
			drive.getRightPIDCont(), 
			DriveSystem.VELOCITY_P, 
			DriveSystem.VELOCITY_I, 
			DriveSystem.VELOCITY_D, 
			DriveSystem.VELOCITY_I_ZONE, 
			DriveSystem.VELOCITY_FEED_FORWARD,
			Constants.VELOCITY_SLOT_ID
		);

		

		Robot.drive.resetAngle();
		// Robot.drive.resetPosition();

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
		CommandScheduler.getInstance().cancelAll();

		// setPIDF(
		// 	drive.getLeftPIDCont(), 
		// 	0.0004, 
		// 	0.0, 
		// 	0.001, 
		// 	DriveSystem.VELOCITY_I_ZONE, 
		// 	DriveSystem.VELOCITY_FEED_FORWARD,
		// 	Constants.VELOCITY_SLOT_ID
		// );

		// setPIDF(
		// 	drive.getRightPIDCont(), 
		// 	0.0004, 
		// 	0.0, 
		// 	0.001, 
		// 	DriveSystem.VELOCITY_I_ZONE, 
		// 	DriveSystem.VELOCITY_FEED_FORWARD,
		// 	Constants.VELOCITY_SLOT_ID
		// );

		// Robot.drive.tank(0.5, 0.5);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		// System.out.println("LeftPidCont:   P: " + leftPidCont.getP() + "   I: " + leftPidCont.getI() + "   D: " + leftPidCont.getD());
		// System.out.println("RightPidCont:   P: " + rightPidCont.getP() + "   I: " + rightPidCont.getI() + "   D: " + rightPidCont.getD());

		// setPIDF(
		// 	drive.getLeftPIDCont(), 
		// 	leftPidCont.getP(), 
		// 	leftPidCont.getI(), 
		// 	leftPidCont.getD(), 
		// 	DriveSystem.VELOCITY_I_ZONE, 
		// 	DriveSystem.VELOCITY_FEED_FORWARD,
		// 	Constants.VELOCITY_SLOT_ID
		// );

		// setPIDF(
		// 	drive.getRightPIDCont(), 
		// 	rightPidCont.getP(), 
		// 	rightPidCont.getI(), 
		// 	rightPidCont.getD(), 
		// 	DriveSystem.VELOCITY_I_ZONE, 
		// 	DriveSystem.VELOCITY_FEED_FORWARD,
		// 	Constants.VELOCITY_SLOT_ID
		// );

		// System.out.println("Velocity: " + drive.getVelocity());
	}

	@Override
	public void startCompetition() {
		super.startCompetition();
	}
}
