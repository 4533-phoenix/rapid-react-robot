package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Direction;
import frc.robot.commands.Odometry;
import java.lang.Math.*;

public class DriveSystem extends SubsystemBase {
  // Drive train motor controllers
  private CANSparkMax rightMaster;
  private CANSparkMax rightSlave;
  private CANSparkMax leftMaster;
  private CANSparkMax leftSlave;

  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private SparkMaxPIDController leftPIDCont;
  private SparkMaxPIDController rightPIDCont;

  private PIDController dashboardCont;

  // Onboard IMU.
  private AHRS navX;

  private static final double MAX_VELOCITY = 4000;
  private static final double TURBO_VELOCITY = 4500;
  private static final double QUARTER_VELOCITY = 1000;
  private static final double PEAK_OUTPUT = 1.0;
  private boolean turbo = false;
  private boolean quarter = false;

  // Wheel specific constants.
  public static final double TICKS_PER_ROTATION = 42.0;
  private static final double WHEEL_DIAMETER = 6.0;
  private static final double WHEEL_DIAMETER_M = 0.1524;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double WHEEL_CIRCUMFERENCE_M =
    WHEEL_DIAMETER_M * Math.PI;
  private static final double TICKS_PER_INCH =
    TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
  private static final double TICKS_PER_METER =
    TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_M;
  private static final double METERS_PER_TICK =
    WHEEL_CIRCUMFERENCE_M / TICKS_PER_ROTATION;

  // Unit specific constants.
  public static final double INCHES_PER_METER = 39.3701;
  public static final double METERS_PER_INCH = 1 / 39.3701;

  // Velocity PIDF values
  public static final double VELOCITY_P = 0.0003; // 0.3 / 1000 RPM Error
  public static final double VELOCITY_I = 0.0;
  public static final double VELOCITY_D = 0.003; // will tune
  public static final double VELOCITY_I_ZONE = 0.0;
  public static final double VELOCITY_FEED_FORWARD = 1.25E-4; // 0.5 / MAX_VELOCITY

  // Position PIDF values
  public static final double POSITION_P = 0.006;
  public static final double POSITION_I = 0.0;
  public static final double POSITION_D = 0.06; // 0.016677 will tune
  public static final double POSITION_I_ZONE = 0.0;
  public static final double POSITION_FEED_FORWARD = 0.0;

  // Feed Forward Gains
  // kS - the voltage needed to overcome the motor's static friction (V).
  // kV - the voltage needed to maintain a given constant velocity (V * s/m).
  // kA - the voltage needed to induce a given acceleration (V * s^2/m).
  public static final double FEED_FORWARD_KS = 0.16203;
  public static final double FEED_FORWARD_KV = 0.13098;
  public static final double FEED_FORWARD_KA = 0.026747;

  public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
    FEED_FORWARD_KS,
    FEED_FORWARD_KV,
    FEED_FORWARD_KA
  );

  // TODO: These values were calculated as part of the robot characterization
  // process. We need to determine whether or not we want to keep them separate
  // from the above PID and FF gains. Another consideration is whether or not
  // we should track the left and right values separately.
  public static final double kPVelocity = 0.00177;
  public static final double kDVelocity = 0.0;

  public static final double kPPosition = 0.0;
  public static final double kDPosition = 0.0;

  // Kinematic constants.
  public static final double TRACK_WIDTH = 0.537;
  public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
    TRACK_WIDTH
  );

  public static final double kMaxSpeed = 3;
  public static final double kMaxAcceleration = 3;

  /** Default timeout in milliseconds */
  private static final int DEFAULT_TIMEOUT = 30;

  private static Pose2d targetPosition;
  private static Direction targetDirection;
  private static double oldTargetPosition;
  private static double oldTargetAngle;

  private static Direction turnDirection;
  private static Direction driveDirection;

  public enum DriveMode {
    Normal,
    Inverted,
  }

  private DriveMode driveMode = DriveMode.Normal;

  private Odometry odometry;
  private Pose2d robotPos;
  private Rotation2d robotAngle;

  private static int navXCount = 0;

  private double prevLeft, prevRight;

  public DriveSystem() {
    // Initialize all of the drive systems motor controllers.
    this.leftMaster =
      new CANSparkMax(Constants.LEFT_MASTER_MOTOR, MotorType.kBrushless);
    this.leftSlave =
      new CANSparkMax(Constants.LEFT_SLAVE_MOTOR, MotorType.kBrushless);
    this.rightMaster =
      new CANSparkMax(Constants.RIGHT_MASTER_MOTOR, MotorType.kBrushless);
    this.rightSlave =
      new CANSparkMax(Constants.RIGHT_SLAVE_MOTOR, MotorType.kBrushless);

    this.leftEncoder = leftMaster.getEncoder();
    this.rightEncoder = rightMaster.getEncoder();

    this.leftPIDCont = leftMaster.getPIDController();
    leftPIDCont.setFeedbackDevice(leftEncoder);

    this.rightPIDCont = rightMaster.getPIDController();
    rightPIDCont.setFeedbackDevice(rightEncoder);

    this.dashboardCont =
      new PIDController(
        leftPIDCont.getP(1),
        leftPIDCont.getI(1),
        leftPIDCont.getD(1)
      );

    this.leftSlave.follow(leftMaster);
    this.rightSlave.follow(rightMaster);

    this.rightMaster.setIdleMode(IdleMode.kCoast);
    this.rightSlave.setIdleMode(IdleMode.kCoast);
    this.leftMaster.setIdleMode(IdleMode.kCoast);
    this.leftSlave.setIdleMode(IdleMode.kCoast);

    this.rightMaster.setClosedLoopRampRate(0.25);
    this.rightSlave.setClosedLoopRampRate(0.25);
    this.leftMaster.setClosedLoopRampRate(0.25);
    this.leftSlave.setClosedLoopRampRate(0.25);

    // Initialize the NavX IMU sensor.
    this.navX = new AHRS(SPI.Port.kMXP);

    this.robotPos = new Pose2d(5, 5, new Rotation2d());
    this.odometry = new Odometry(navX.getRotation2d(), robotPos);

    this.targetPosition = null;
  }

  public SparkMaxPIDController getLeftPIDCont() {
    return leftPIDCont;
  }

  public SparkMaxPIDController getRightPIDCont() {
    return rightPIDCont;
  }

  public void quarterTrue() {
    quarter = true;
  }

  public void quarterFalse() {
    quarter = false;
  }

  public void setDriveMode(DriveMode mode) {
    this.driveMode = mode;
  }

  public void toggleDriveMode() {
    if (this.driveMode == DriveMode.Normal) {
      this.driveMode = DriveMode.Inverted;
    } else {
      this.driveMode = DriveMode.Normal;
    }
  }

  public boolean reachedPosition() {
    return (
      Math.abs(robotPos.getX()) >= Math.abs(targetPosition.getX()) &&
      Math.abs(robotPos.getY()) >= Math.abs(targetPosition.getY())
    );
  }

  public boolean reachedCurve(double targetL, double targetR) {
    double leftPos = this.leftEncoder.getPosition();
    double rightPos = this.rightEncoder.getPosition();

    if (targetDirection == Direction.FORWARD) {
      return (leftPos >= targetL) && (rightPos >= targetR);
    } else if (targetDirection == Direction.BACKWARD) {
      return (leftPos <= targetL) && (rightPos <= targetR);
    } else {
      return true;
    }
  }

  public Pose2d driveToPosition(double xMeters, double yMeters) {
    Translation2d translation = new Translation2d(xMeters, yMeters);
    Rotation2d angle = Rotation2d.fromDegrees(
      Math.atan(yMeters / xMeters) * (180 / Math.PI)
    );

    if (angle.getDegrees() < 0) {
      angle = Rotation2d.fromDegrees(360 + angle.getDegrees());
    }

    System.out.println("Angle: " + angle.getDegrees());

    targetPosition = new Pose2d(translation, angle);

    if (
      targetPosition.getRotation().getDegrees() - robotAngle.getDegrees() > 180
    ) {
      turnDirection = Direction.RIGHT;
    } else if (
      targetPosition.getRotation().getDegrees() - robotAngle.getDegrees() <= 180
    ) {
      turnDirection = Direction.LEFT;
    } else if (
      targetPosition.getRotation().getDegrees() - robotAngle.getDegrees() < -180
    ) {
      turnDirection = Direction.RIGHT;
    } else if (
      targetPosition.getRotation().getDegrees() -
      robotAngle.getDegrees() >=
      -180
    ) {
      turnDirection = Direction.LEFT;
    } else {
      turnDirection = Direction.LEFT;
    }

    System.out.println(turnDirection);

    // if (targetPosition.getY() >= 0) {
    // 	driveDirection = Direction.FORWARD;
    // }
    // else if (targetPosition.getY() < 0) {
    // 	driveDirection = Direction.BACKWARD;
    // }
    // else {
    // 	driveDirection = Direction.FORWARD;
    // }

    driveDirection = Direction.FORWARD;

    return targetPosition;
  }

  // TODONE: Add back old drive distance code in a different method for use of non-odometry autonomous commands
  public void driveDistance() {
    double targetDist =
      targetPosition.getY() /
      Math.sin(targetPosition.getRotation().getRadians());

    if (driveDirection == Direction.FORWARD) {
      targetDist *= INCHES_PER_METER / WHEEL_CIRCUMFERENCE;
    } else if (driveDirection == Direction.BACKWARD) {
      targetDist *= -1 * INCHES_PER_METER / WHEEL_CIRCUMFERENCE;
    } else {
      targetDist = 0;
    }

    this.leftPIDCont.setReference(
        targetDist,
        CANSparkMax.ControlType.kPosition,
        Constants.POSITION_SLOT_ID
      );
    this.rightPIDCont.setReference(
        targetDist,
        CANSparkMax.ControlType.kPosition,
        Constants.POSITION_SLOT_ID
      );
  }

  public void oldDriveDistance(double inches, Direction direction) {
    targetDirection = direction;

    if (targetDirection == Direction.FORWARD) {
      oldTargetPosition = (inches / WHEEL_CIRCUMFERENCE) * 42;
    } else if (targetDirection == Direction.BACKWARD) {
      oldTargetPosition = -1 * (inches / WHEEL_CIRCUMFERENCE) * 42;
    } else {
      oldTargetPosition = 0;
    }

    System.out.println("Left: " + leftEncoder.getPosition());
    System.out.println("Right: " + rightEncoder.getPosition());

    this.leftPIDCont.setReference(
        oldTargetPosition,
        CANSparkMax.ControlType.kPosition,
        Constants.POSITION_SLOT_ID
      );
    this.rightPIDCont.setReference(
        -oldTargetPosition,
        CANSparkMax.ControlType.kPosition,
        Constants.POSITION_SLOT_ID
      );
  }

  public boolean oldReachedPosition() {
    double leftPos = leftEncoder.getPosition();
    double rightPos = rightEncoder.getPosition();

    if (targetDirection == Direction.FORWARD) {
      return (leftPos >= oldTargetPosition) && (rightPos >= oldTargetPosition);
    } else if (targetDirection == Direction.BACKWARD) {
      return (leftPos <= oldTargetPosition) && (rightPos <= oldTargetPosition);
    } else {
      return true;
    }
  }

  public void driveCurve(
    double leftDist,
    double rightDist,
    Direction direction
  ) {
    targetDirection = direction;
    if (direction == Direction.FORWARD) {
      leftDist = -1 * leftDist / WHEEL_CIRCUMFERENCE;
      rightDist = -1 * rightDist / WHEEL_CIRCUMFERENCE;
    } else if (direction == Direction.BACKWARD) {
      leftDist = leftDist / WHEEL_CIRCUMFERENCE;
      rightDist = rightDist / WHEEL_CIRCUMFERENCE;
    } else {
      leftDist = 0;
      rightDist = 0;
    }
    this.leftPIDCont.setReference(
        leftDist,
        CANSparkMax.ControlType.kPosition,
        Constants.POSITION_SLOT_ID
      );
    this.rightPIDCont.setReference(
        rightDist,
        CANSparkMax.ControlType.kPosition,
        Constants.POSITION_SLOT_ID
      );
  }

  public void driveCircle(
    double speed,
    double angle,
    Direction direction,
    double radius
  ) {
    double innerCircumference = radius * 2 * Math.PI * (angle / 360);
    double outerCircumference = (radius + 24) * 2 * Math.PI * (angle / 360);

    double leftDist, rightDist;

    if (direction == Direction.LEFT) {
      leftDist = innerCircumference / WHEEL_CIRCUMFERENCE;
      rightDist = outerCircumference / WHEEL_CIRCUMFERENCE;
    } else if (direction == Direction.RIGHT) {
      leftDist = outerCircumference / WHEEL_CIRCUMFERENCE;
      rightDist = innerCircumference / WHEEL_CIRCUMFERENCE;
    } else {
      leftDist = 0;
      rightDist = 0;
    }

    this.leftPIDCont.setReference(
        leftDist,
        CANSparkMax.ControlType.kPosition,
        Constants.POSITION_SLOT_ID
      );
    this.rightPIDCont.setReference(
        rightDist,
        CANSparkMax.ControlType.kPosition,
        Constants.POSITION_SLOT_ID
      );
  }

  public boolean reachedCircle(
    double angle,
    double radius,
    Direction direction
  ) {
    double leftPos = this.leftEncoder.getPosition();
    double rightPos = this.rightEncoder.getPosition();

    double rightTarget, leftTarget;

    if (direction == Direction.LEFT) {
      leftTarget = radius * 2 * Math.PI * (angle / 360) / WHEEL_CIRCUMFERENCE;
      rightTarget =
        (radius + 24) * 2 * Math.PI * (angle / 360) / WHEEL_CIRCUMFERENCE;

      return (leftPos >= leftTarget) && (rightPos >= rightTarget);
    } else if (direction == Direction.RIGHT) {
      rightTarget = radius * 2 * Math.PI * (angle / 360) / WHEEL_CIRCUMFERENCE;
      leftTarget =
        (radius + 24) * 2 * Math.PI * (angle / 360) / WHEEL_CIRCUMFERENCE;

      return (leftPos >= leftTarget) && (rightPos >= rightTarget);
    } else {
      rightTarget = 0;
      leftTarget = 0;

      return true;
    }
    // System.out.println("Position: " + leftPos + "    " + rightPos);
    // System.out.println("Target: " + leftTarget + "    " + rightTarget);
  }

  public double getPosition() {
    return this.leftEncoder.getPosition() / TICKS_PER_INCH;
  }

  public double getLeftDistance() {
    return this.leftEncoder.getPosition() / TICKS_PER_METER;
  }

  public double getRightDistance() {
    return this.rightEncoder.getPosition() / TICKS_PER_METER;
  }

  public void toggleTurbo() {
    this.turbo = !this.turbo;
  }

  public void setTurbo(boolean isTriggerPressed) {
    if (isTriggerPressed) {
      this.turbo = true;
    } else if (!isTriggerPressed) {
      this.turbo = false;
    } else {
      this.turbo = false;
    }
  }

  public void tank(double left, double right) {
    if (left > -0.05 && left < 0.05) {
      left = 0.0;
    }

    if (right > -0.05 && right < 0.05) {
      right = 0.0;
    }

    double targetLeft;
    double targetRight;

    double targetVelocity = MAX_VELOCITY;

    if (this.turbo) {
      targetVelocity = TURBO_VELOCITY;
    }

    if (this.quarter) {
      targetVelocity = QUARTER_VELOCITY;
    }

    targetLeft = left * targetVelocity;
    targetRight = right * targetVelocity;

    if (this.driveMode == DriveMode.Inverted) {
      double temp = targetLeft;
      targetLeft = -targetRight;
      targetRight = -temp;
    }

    this.leftPIDCont.setReference(
        -targetLeft,
        CANSparkMax.ControlType.kVelocity,
        Constants.VELOCITY_SLOT_ID
      );
    this.rightPIDCont.setReference(
        targetRight,
        CANSparkMax.ControlType.kVelocity,
        Constants.VELOCITY_SLOT_ID
      );
  }

  public void voltage(double left, double right) {
    leftMaster.setVoltage(left * 12.0);
    rightMaster.setVoltage(right * 12.0);
  }

  public void percent(double left, double right) {
    this.leftMaster.set(left);
    this.rightMaster.set(right);
  }

  public double getAngle() {
    double angle = Math.abs(navX.getAngle());
    return angle;
  }

  public void resetAngle() {
    navX.reset();
  }

  public void resetPosition() {
    this.leftEncoder.setPosition(0);
    this.rightEncoder.setPosition(0);
  }

  // TODONE: Add back old turn code in a different method for use of non-odometry autonomous commands
  public void turn(double speed) {
    switch (turnDirection) {
      case LEFT:
        this.voltage(-speed, -speed);
        break;
      case RIGHT:
        this.voltage(speed, speed);
        break;
      default:
        this.voltage(0, 0);
    }
  }

  public void oldTurn(double speed, Direction direction) {
    switch (direction) {
      case LEFT:
        this.voltage(speed, speed);
        break;
      case RIGHT:
        this.voltage(-speed, -speed);
        break;
      default:
        this.voltage(0, 0);
    }
  }

  public boolean oldReachedTurn(double angle) {
    if (angle + 1 > 360) {
      return (
        360 - robotAngle.getDegrees() <= (angle + 1) % 360 &&
        robotAngle.getDegrees() >= angle - 1
      );
    } else if (angle - 1 < 0) {
      return (
        robotAngle.getDegrees() <= angle + 1 &&
        -(360 - robotAngle.getDegrees()) >= angle - 1
      );
    } else {
      return (
        robotAngle.getDegrees() <= angle + 1 &&
        robotAngle.getDegrees() >= angle - 1
      );
    }
  }

  public boolean reachedTurn() {
    double angle = targetPosition.getRotation().getDegrees();

    if (angle + 1 > 360) {
      return (
        360 - robotAngle.getDegrees() <= (angle + 1) % 360 &&
        robotAngle.getDegrees() >= angle - 1
      );
    } else if (angle - 1 < 0) {
      return (
        robotAngle.getDegrees() <= angle + 1 &&
        -(360 - robotAngle.getDegrees()) >= angle - 1
      );
    } else {
      return (
        robotAngle.getDegrees() <= angle + 1 &&
        robotAngle.getDegrees() >= angle - 1
      );
    }
  }

  @Override
  public void periodic() {
    double leftDist =
      leftEncoder.getPosition() * TICKS_PER_ROTATION / TICKS_PER_METER;
    double rightDist =
      rightEncoder.getPosition() * TICKS_PER_ROTATION / TICKS_PER_METER;

    robotPos = odometry.update(navX.getRotation2d(), leftDist, rightDist);

    // ensures there is never a negative angle value
    // this is useful for turning, as we don't have to worry about the robot making unecessarily long turns
    if (-navX.getAngle() < 0) {
      robotAngle = Rotation2d.fromDegrees(360 - (navX.getAngle() % 360));
    } else {
      robotAngle = Rotation2d.fromDegrees(-navX.getAngle() % 360);
    }
  }
}
