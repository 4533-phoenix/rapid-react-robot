package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class ShooterSystem extends SubsystemBase {
  private WPI_TalonFX leftFlywheelMotor;
  private WPI_TalonFX rightFlywheelMotor;

  private WPI_TalonSRX flywheelIntakeMotor;
  private CANSparkMax hoodMotor;
  private SparkMaxPIDController hoodPIDCont;
  private RelativeEncoder hoodEncoder;

  double initialAngle;
  double targetAngle;

  private static final double FLYWHEEL_MOTOR_PERCENT = 0.90;
  private static final double FLYWHEEL_INTAKE_MOTOR_PERCENT = 0.25;
  private static final double HOOD_MOTOR_PERCENT = 0.05;
  private static final double DEGREES_PER_TICK =
    360 / DriveSystem.TICKS_PER_ROTATION;
  private static final double HOOD_DEGREES_PER_TICK = DEGREES_PER_TICK / 80;
  private static final double HOOD_DEGREES_PER_ROTATION =
    HOOD_DEGREES_PER_TICK * DriveSystem.TICKS_PER_ROTATION;

  private AHRS shooterNAVX;

  public double hoodAngle;

  private NetworkTable limelight;

  private double targetOffsetAngle_Horizontal;
  private double targetOffsetAngle_Vertical;
  private double targetArea;
  private double targetSkew;

  private double targetDistanceFromShooter;
  private double targetDistanceFromRobot;

  private DigitalInput limitSwitch;

  // PhotonCamera visionCam;
  // PhotonPipelineResult result;
  // PhotonTrackedTarget target;

  // private double yaw;
  // private double pitch;
  // private double skew;
  // private double area;

  // Transform2d pos;
  // List<TargetCorner> corners;

  public ShooterSystem() {
    leftFlywheelMotor = new WPI_TalonFX(MotorConstants.FLYWHEEL_MOTOR_LEFT);
    rightFlywheelMotor = new WPI_TalonFX(MotorConstants.FLYWHEEL_MOTOR_RIGHT);

    flywheelIntakeMotor = new WPI_TalonSRX(MotorConstants.TURRET_WHEEL_MOTOR);

    hoodMotor = new CANSparkMax(MotorConstants.HOOD_MOTOR, MotorType.kBrushless);

    leftFlywheelMotor.setNeutralMode(NeutralMode.Brake);
    rightFlywheelMotor.setNeutralMode(NeutralMode.Brake);

    hoodMotor.setIdleMode(IdleMode.kBrake);

    flywheelIntakeMotor.setNeutralMode(NeutralMode.Brake);

    leftFlywheelMotor.setInverted(true);

    flywheelIntakeMotor.setInverted(true);

    hoodEncoder = hoodMotor.getEncoder();
    hoodPIDCont = hoodMotor.getPIDController();

    shooterNAVX = new AHRS(SPI.Port.kMXP);

    hoodAngle = 0;

    limitSwitch = new DigitalInput(0);

    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // visionCam = new PhotonCamera("WARCam");
    // result = null;
    // target = null;

    // yaw = 0.0;
    // pitch = 0.0;
    // skew = 0.0;

    // pos = null;
    // corners = null;
  }

  public void flywheelIn() {
    leftFlywheelMotor.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
    rightFlywheelMotor.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
  }

  public void flywheelOut() {
    this.leftFlywheelMotor.set(
        ControlMode.PercentOutput,
        FLYWHEEL_MOTOR_PERCENT
      );
    this.rightFlywheelMotor.set(
        ControlMode.PercentOutput,
        FLYWHEEL_MOTOR_PERCENT
      );
  }

  public void flywheelStop() {
    leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
    rightFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void flywheelIntakeIn() {
    flywheelIntakeMotor.set(
      ControlMode.PercentOutput,
      FLYWHEEL_INTAKE_MOTOR_PERCENT
    );
  }

  public void flywheelIntakeOut() {
    flywheelIntakeMotor.set(
      ControlMode.PercentOutput,
      -FLYWHEEL_INTAKE_MOTOR_PERCENT
    );
  }

  public void flywheelIntakeStop() {
    flywheelIntakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void flywheelAndFlywheelIntakeIn() {
    leftFlywheelMotor.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
    rightFlywheelMotor.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
    flywheelIntakeMotor.set(
      ControlMode.PercentOutput,
      FLYWHEEL_INTAKE_MOTOR_PERCENT
    );
  }

  public void flywheelAndFlywheelIntakeOut() {
    leftFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
    rightFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
    flywheelIntakeMotor.set(
      ControlMode.PercentOutput,
      -FLYWHEEL_INTAKE_MOTOR_PERCENT
    );
  }

  public void flywheelAndFlywheelIntakeStop() {
    leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
    rightFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
    flywheelIntakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetPosition() {
    leftFlywheelMotor.setSelectedSensorPosition(0);
    rightFlywheelMotor.setSelectedSensorPosition(0);
    flywheelIntakeMotor.setSelectedSensorPosition(0);
  }

  public void setFlywheelReset() {
    Robot.drive.resetPosition();
  }

  public double setHoodAngle(double angle) {
    initialAngle = getHoodAngle();
    targetAngle = angle;

    if (targetAngle < initialAngle) {
      while (hoodEncoder.getVelocity() == 0) {
        this.hoodPIDCont.setReference(
            -HOOD_MOTOR_PERCENT,
            CANSparkMax.ControlType.kDutyCycle
          );
      }
    } else {
      while (hoodEncoder.getVelocity() == 0) {
        this.hoodPIDCont.setReference(
            HOOD_MOTOR_PERCENT,
            CANSparkMax.ControlType.kDutyCycle
          );
      }
    }

    return getHoodAngle();
  }

  public boolean hoodReachedPosition() {
    if (hoodEncoder.getVelocity() > 0) {
      if (initialAngle > targetAngle) {
        return true;
      }
      return false;
    } else {
      if (initialAngle < targetAngle + 2.8) {
        return true;
      }
      return false;
    }
  }

  public double getHoodAngle() {
    hoodAngle =
      (this.hoodEncoder.getPosition() * HOOD_DEGREES_PER_ROTATION) % 360;
    return hoodAngle;
  }

  public void resetHoodAngle(double angle) {
    hoodAngle = angle;
    hoodEncoder.setPosition(hoodAngle % 360);
  }

  public void hoodUp() {
    this.hoodPIDCont.setReference(
        HOOD_MOTOR_PERCENT,
        CANSparkMax.ControlType.kDutyCycle
      );
  }

  public void hoodDown() {
    this.hoodPIDCont.setReference(
        -HOOD_MOTOR_PERCENT,
        CANSparkMax.ControlType.kDutyCycle
      );
  }

  public void hoodStop() {
    this.hoodPIDCont.setReference(0.0, CANSparkMax.ControlType.kDutyCycle);
  }

  public void setFlywheelPos() {
    // if (yaw >= 1) {
    //     Robot.drive.percent(0.5, -0.5);
    // }
    // else if (yaw <= -1) {
    //     Robot.drive.percent(-0.5, 0.5);
    // }
    // else {
    //     Robot.drive.percent(0.0, 0.0);
    // }
  }

  public void stopFlywheelPos() {
    Robot.drive.percent(0.0, 0.0);
  }

  public boolean flywheelReachedPosition() {
    // return yaw <= 1 && yaw >= -1;
    return true;
  }

  public boolean flywheelDoneShootBalls(int balls) {
    double targetPosition = (balls * 30) * DriveSystem.TICKS_PER_ROTATION;
    return rightFlywheelMotor.getSelectedSensorPosition() >= targetPosition;
  }

  public void autoTurretSwivel() {
    // System.out.println("test");
    if (targetOffsetAngle_Horizontal > 1) {
      Robot.drive.percent(0.05, 0.05);
    } else if (targetOffsetAngle_Horizontal < -1) {
      Robot.drive.percent(-0.05, -0.05);
    } else {
      Robot.drive.percent(0.0, 0.0);
    }
  }

  public boolean turretReachedPosition() {
    return (
      targetOffsetAngle_Horizontal > -1.2 && targetOffsetAngle_Horizontal < 1.2
    );
  }

  public void updateLimelight() {
    this.targetOffsetAngle_Horizontal = this.limelight.getEntry("tx").getDouble(0);
    this.targetOffsetAngle_Vertical = this.limelight.getEntry("ty").getDouble(0);
    this.targetArea = this.limelight.getEntry("ta").getDouble(0);
    this.targetSkew = this.limelight.getEntry("ts").getDouble(0);

    double angleToGoalDegrees = LimelightConstants.LIMELIGHT_MOUNT_ROTATION_DEGREES + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    double distanceFromLimelightToGoalInches = (FieldConstants.TARGET_HEIGHT_INCHES - LimelightConstants.LIMELIGHT_MOUNT_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

    this.targetDistanceFromShooter = 0;
    this.targetDistanceFromRobot = 0;
  }

  @Override
  public void periodic() {
    updateLimelight();

    // System.out.println("Hood angle: " + getHoodAngle());

    if (!limitSwitch.get()) {
      hoodEncoder.setPosition(0.0);
    }
    // result = visionCam.getLatestResult();

    // if (result.hasTargets()) {
    //     target = result.getBestTarget();
    // }

    // yaw = target.getYaw();
    // pitch = target.getPitch();
    // skew = target.getSkew();
    // area = target.getArea();

    // pos = target.getCameraToTarget();
    // corners = target.getCorners();
  }
}
