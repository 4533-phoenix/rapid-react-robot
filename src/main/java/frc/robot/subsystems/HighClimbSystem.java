package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.commands.Direction;
import frc.robot.commands.Odometry;

/**
 * The class for the high climb system.
 */
public class HighClimbSystem extends SubsystemBase {
  private WPI_TalonSRX highClimbLeader;
  private WPI_TalonSRX highClimbFollower;

  public static final double HIGH_CLIMB_MOTOR_PERCENT = 0.75;

  private DigitalInput leftLimitSwitch;
  private DigitalInput rightLimitSwitch;

  private boolean limitSwitchPressed;

  /**
   * Constructor for the high climb system.
   */
  public HighClimbSystem() {
    this.highClimbLeader = new WPI_TalonSRX(Constants.HIGH_CLIMB_MOTOR);
    this.highClimbFollower = new WPI_TalonSRX(Constants.HIGH_CLIMB_FOLLOWER);

    this.highClimbLeader.setInverted(true);

    this.highClimbFollower.follow(highClimbLeader);

    this.leftLimitSwitch = new DigitalInput(Constants.LEFT_LIMIT_SWITCH);
    this.rightLimitSwitch = new DigitalInput(Constants.RIGHT_LIMIT_SWITCH);

    this.limitSwitchPressed = false;
  }

  /**
   * Sets the high climb motor to lift up the high climb hook.
   * <p>
   * Will only do this if it is not activating the right limit switch
   * ({@link #rightLimitSwitch}).
   */
  public void highClimberUp() {
    if (!this.rightLimitSwitch.get()) {
      highClimbLeader.set(ControlMode.PercentOutput, HIGH_CLIMB_MOTOR_PERCENT);
    }
  }

  /**
   * Sets the high climb motor to lower down the high climb hook.
   * <p>
   * Will only do this if it is not activating the left limit switch
   * ({@link #leftLimitSwitch}).
   */
  public void highClimberDown() {
    if (!this.leftLimitSwitch.get()) {
      highClimbLeader.set(ControlMode.PercentOutput, -HIGH_CLIMB_MOTOR_PERCENT);
    }
  }
  
  /**
   * Stops the high climb motor from running.
   */
  public void highClimberStop() {
    highClimbLeader.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Runs the code within it every 20ms.
   */
  @Override
  public void periodic() {
    /*
     * If either of the limit switches are being pressed
     * and the boolean flag limitSwitchPressed is false,
     * then stop the high climb arm motor and set the 
     * boolean flag limitSwitchPressed to true to make
     * sure that the high climb motor is only commanded
     * to stop once.
     */
    if (!leftLimitSwitch.get() || !rightLimitSwitch.get() && !limitSwitchPressed) {
      this.highClimberStop();

      this.limitSwitchPressed = true;
    }

    /*
     * If neither of the limit switches are being 
     * pressed, then set the boolean flag
     * limitSwitchedPressed to false to reset 
     * the ability to set the high climb arm 
     * motor to stop once either of the limit 
     * switches is pressed.
     */
    if (leftLimitSwitch.get() && rightLimitSwitch.get()) {
      this.limitSwitchPressed = false;
    }
  }

  /**
   * Runs the code within it every 20ms only in simulation mode.
   */
  @Override
  public void simulationPeriodic() {
  }
}
