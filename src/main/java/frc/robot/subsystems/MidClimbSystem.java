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

public class MidClimbSystem extends SubsystemBase {
  private CANSparkMax climbMotor;

  public static final double CLIMB_MOTOR_PERCENT = 0.75;
  
  /**
   * Constructor for the mid climb system.
   */
  public MidClimbSystem() {
    this.climbMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR,MotorType.kBrushless);
  }

  /**
   * Sets the climb motor to pull robot up (go down).
   */
  public void climberUp() {
    climbMotor.set(CLIMB_MOTOR_PERCENT);
  }

  /**
   * Sets the climb motor to lower robot down (go up).
   */
  public void climberDown() {
    climbMotor.set(-CLIMB_MOTOR_PERCENT);
  }
  
  /**
   * Stops the climb motor from running.
   */
  public void climberStop() {
    climbMotor.set(0.0);
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
