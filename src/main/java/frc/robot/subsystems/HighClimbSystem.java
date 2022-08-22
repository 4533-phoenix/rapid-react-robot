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
import frc.robot.Constants;
import frc.robot.commands.Direction;
import frc.robot.commands.Odometry;

/**
 * The class for the high climb system.
 */
public class HighClimbSystem extends SubsystemBase {
  private WPI_TalonSRX highClimbLeader;
  private WPI_TalonSRX highClimbFollower;

  public static final double HIGH_CLIMB_MOTOR_PERCENT = 0.25;

  /**
   * Constructor for the high climb system.
   */
  public HighClimbSystem() {
    this.highClimbLeader = new WPI_TalonSRX(Constants.HIGH_CLIMB_MOTOR);
    this.highClimbFollower = new WPI_TalonSRX(Constants.HIGH_CLIMB_FOLLOWER);

    this.highClimbFollower.follow(highClimbLeader);
  }

  /**
   * Sets the high climb motor to lift up the high climb hook.
   */
  public void highClimberUp() {
    highClimbLeader.set(ControlMode.PercentOutput, HIGH_CLIMB_MOTOR_PERCENT);
  }

  /**
   * Sets the high climb motor to lower down the high climb hook.
   */
  public void highClimberDown() {
    highClimbLeader.set(ControlMode.PercentOutput, -HIGH_CLIMB_MOTOR_PERCENT);
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
  }

  /**
   * Runs the code within it every 20ms only in simulation mode.
   */
  @Override
  public void simulationPeriodic() {
  }
}
