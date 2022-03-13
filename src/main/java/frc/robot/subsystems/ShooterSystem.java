package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.*;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI ;

import frc.robot.Constants;
import frc.robot.Robot;

import java.util.List;
import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.Direction;
public class ShooterSystem extends SubsystemBase {
    private WPI_TalonFX leftFlywheelMotor;
    private WPI_TalonFX rightFlywheelMotor;

    private WPI_TalonSRX flywheelIntakeMotor;
    private CANSparkMax hoodMotor;
    private SparkMaxPIDController hoodPIDCont;
    private RelativeEncoder hoodEncoder;

    private double initialAngle;
    private double targetAngle;
    private Direction targetDirection;

    private static final double FLYWHEEL_MOTOR_PERCENT = 0.85;
    private static final double FLYWHEEL_INTAKE_MOTOR_PERCENT = 0.40;
    private static final double HOOD_MOTOR_PERCENT = 0.05;
    private static final double DEGREES_PER_TICK = 360/DriveSystem.TICKS_PER_ROTATION;
    private static final double HOOD_DEGREES_PER_TICK = DEGREES_PER_TICK/80;
    private static final double HOOD_DEGREES_PER_ROTATION = HOOD_DEGREES_PER_TICK * DriveSystem.TICKS_PER_ROTATION;

    private AHRS shooterNAVX;

    public double hoodAngle;

    private NetworkTable limelight;

    private double targetOffsetAngle_Horizontal;
    private double targetOffsetAngle_Vertical;
    private double targetArea;
    private double targetSkew;

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
        leftFlywheelMotor = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_LEFT);
        rightFlywheelMotor = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_RIGHT);

        flywheelIntakeMotor = new WPI_TalonSRX(Constants.TURRET_WHEEL_MOTOR);

        hoodMotor = new CANSparkMax(Constants.HOOD_MOTOR, MotorType.kBrushless);

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
        this.leftFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
        this.rightFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
    }

    public void flywheelStop() {
        leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
        rightFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void flywheelIntakeIn() {
        flywheelIntakeMotor.set(ControlMode.PercentOutput, FLYWHEEL_INTAKE_MOTOR_PERCENT);
    }

    public void flywheelIntakeOut() {
        flywheelIntakeMotor.set(ControlMode.PercentOutput, -FLYWHEEL_INTAKE_MOTOR_PERCENT);
    }

    public void flywheelIntakeStop() {
        flywheelIntakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void flywheelAndFlywheelIntakeIn() {
        leftFlywheelMotor.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
        rightFlywheelMotor.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
        flywheelIntakeMotor.set(ControlMode.PercentOutput, FLYWHEEL_INTAKE_MOTOR_PERCENT);
    }

    public void flywheelAndFlywheelIntakeOut() {
        leftFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
        rightFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
        flywheelIntakeMotor.set(ControlMode.PercentOutput, -FLYWHEEL_INTAKE_MOTOR_PERCENT);
    }

    public void flywheelAndFlywheelIntakeStop() {
        leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
        rightFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
        flywheelIntakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void flywheelAndFlywheelIntakeRun(double hoodAngle, int balls, boolean flywheelRunning) {
        Timer timer = new Timer();
        timer.reset();
        timer.start();

        int time, flywheelIntakeTime;

        if (flywheelRunning) {
            time = 1 + (2 * balls);
            flywheelIntakeTime = 0;
        }
        else {
            time = 5 + (2 * balls);
            flywheelIntakeTime = 4;
        }
        
        initialAngle = getHoodAngle();
        targetAngle = hoodAngle;
        targetDirection = Direction.FORWARD;

        while (timer.get() < time) {
            if (!flywheelRunning) {
                leftFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
                rightFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
            }
            
            if (timer.get() > flywheelIntakeTime && timer.get() < time) {
                flywheelIntakeMotor.set(ControlMode.PercentOutput, FLYWHEEL_INTAKE_MOTOR_PERCENT);
            }

            // if (!hoodReachedPosition()) {
            //     setHoodAngle(hoodAngle);
            // }
        }

        timer.stop();
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
            targetDirection = Direction.BACKWARD;

            do {
                this.hoodPIDCont.setReference(-HOOD_MOTOR_PERCENT, ControlType.kDutyCycle);
            }
            while (hoodEncoder.getVelocity() <= 0);
        }
        else {
            targetDirection = Direction.FORWARD;

            do {
                this.hoodPIDCont.setReference(HOOD_MOTOR_PERCENT, ControlType.kDutyCycle);

            }
            while (hoodEncoder.getVelocity() <= 0);
        }

        return getHoodAngle();
    }

    public boolean hoodReachedPosition() {
        if (targetDirection == Direction.FORWARD) {
            if (initialAngle > targetAngle) { 
                return true;
            }
            return false;
        } 
        else if (targetDirection == Direction.BACKWARD) {
            if (initialAngle < targetAngle + 2.8) { 
                return true;
            }
            return false;
        }
        else {
            return true;
        }
    }

    public double getHoodAngle() {
        hoodAngle = (this.hoodEncoder.getPosition()*HOOD_DEGREES_PER_ROTATION) % 360;
        return hoodAngle;
    }

    public void resetHoodAngle(double angle) {
        hoodAngle = angle;
        hoodEncoder.setPosition(hoodAngle % 360);
    }

    public void hoodUp() {
        this.hoodPIDCont.setReference(HOOD_MOTOR_PERCENT, ControlType.kDutyCycle);
    }

    public void hoodDown() {
        this.hoodPIDCont.setReference(-HOOD_MOTOR_PERCENT, ControlType.kDutyCycle);
    }

    public void hoodStop() {
        this.hoodPIDCont.setReference(0.0, ControlType.kDutyCycle);
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
		} 
		else if (targetOffsetAngle_Horizontal < -1) {
			Robot.drive.percent(-0.05, -0.05);
		}
        else {
            Robot.drive.percent(0.0, 0.0);
        }
	}

	public boolean turretReachedPosition() {
		return targetOffsetAngle_Horizontal > -1.2 && targetOffsetAngle_Horizontal < 1.2;
	}

    @Override
    public void periodic() {
        targetOffsetAngle_Horizontal = limelight.getEntry("tx").getDouble(0);
		targetOffsetAngle_Vertical = limelight.getEntry("ty").getDouble(0);
		targetArea = limelight.getEntry("ta").getDouble(0);
		targetSkew = limelight.getEntry("ts").getDouble(0);

        // System.out.println("Hood angle: " + getHoodAngle());

        getHoodAngle();

        if (!limitSwitch.get()) {
            // System.out.println("Limit Switch Pressed!");
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
