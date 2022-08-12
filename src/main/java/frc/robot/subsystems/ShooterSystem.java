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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.List;

import javax.swing.GroupLayout;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.Direction;

/**
 * The class for the shooter system.
 */
public class ShooterSystem extends SubsystemBase {
    private WPI_TalonFX leftFlywheelMotor;
    private WPI_TalonFX rightFlywheelMotor;

    private WPI_TalonSRX flywheelIntakeMotor;
    private Servo hoodMotor;

    private double initialAngle;
    private double targetAngle;
    private Direction targetDirection;

    // Motor Constants
    private static final double FLYWHEEL_MOTOR_PERCENT = 0.7;
    private static final double FLYWHEEL_INTAKE_MOTOR_PERCENT = 0.40;
    private static final double HOOD_MOTOR_PERCENT = 0.05;
    private static final double DEGREES_PER_TICK = 360/DriveSystem.TICKS_PER_ROTATION;
    private static final double HOOD_DEGREES_PER_TICK = DEGREES_PER_TICK/80;
    private static final double HOOD_DEGREES_PER_ROTATION = HOOD_DEGREES_PER_TICK * DriveSystem.TICKS_PER_ROTATION;
    private static double servoOffset = 179;

    // Projectile Constants and Variables
    private static final double MAX_FLYWHEEL_RPM = 3065.0; // RPM
    private static final double FLYWHEEL_RADIUS = 1.0 / 6.0; // feet
    private static final double TIME_TO_SHOOT = 4; // seconds 
    private static final double GOAL_HEIGHT = 8.67; // feet
    private static final double CAMERA_HEIGHT = 24.8 / 12.0; // feet
    private static final double CAMERA_MOUNTING_ANGLE = 37.0; // degrees
    private static double cameraTargetAngle = 0.0;
    private static double horizontalDistance = 0.0;
    private static final double GRAVITY_ACCELERATION = -32.1741; // feet/s^2
    private static final double INITIAL_Y_VELOCITY = ((GOAL_HEIGHT - CAMERA_HEIGHT) - (0.5 * GRAVITY_ACCELERATION * (TIME_TO_SHOOT * TIME_TO_SHOOT))) / TIME_TO_SHOOT; // feet/s
    private static double initialXVelocity = 0.0;
    private static double initialVelocity = 0.0;
    private static double rotationalVelocity = 0.0;
    private static double shootHoodAngle = 0.0;
    private static double shootHoodPercent = 0.0;
    private static double trajectoryConstant = 2.0; // changes location of max height of trajectory

    private AHRS shooterNAVX;
    
    private double currHoodAngle;

    private NetworkTable limelight;

    private double targetOffsetAngle_Horizontal;
    private double targetOffsetAngle_Vertical;
    private double targetArea;
    private double targetSkew;

    private DigitalInput limitSwitch;

    // TODO: Decide soon when we're going to activate PhotonVision
    // PhotonCamera visionCam;
    // PhotonPipelineResult result;
    // PhotonTrackedTarget target;

    // private double yaw;
    // private double pitch;
    // private double skew;
    // private double area;

    // Transform2d pos;
    // List<TargetCorner> corners;

    /**
     * Constructor for the shooter system.
     */
    public ShooterSystem() {
        leftFlywheelMotor = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_LEFT);
        rightFlywheelMotor = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_RIGHT);

        flywheelIntakeMotor = new WPI_TalonSRX(Constants.TURRET_WHEEL_MOTOR);

        hoodMotor = new Servo(Constants.HOOD_SERVO_MOTOR);

        hoodMotor.setAngle(servoOffset);

        leftFlywheelMotor.setNeutralMode(NeutralMode.Brake);
        rightFlywheelMotor.setNeutralMode(NeutralMode.Brake);

        flywheelIntakeMotor.setNeutralMode(NeutralMode.Brake);

        leftFlywheelMotor.setInverted(true);

        flywheelIntakeMotor.setInverted(true);

        shooterNAVX = new AHRS(SPI.Port.kMXP);

        currHoodAngle = 0;

        limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH);

        limelight = NetworkTableInstance.getDefault().getTable("limelight");

        shootHoodPercent = FLYWHEEL_MOTOR_PERCENT;

        // visionCam = new PhotonCamera("WARCam");
        // result = null;
        // target = null;
        
        // yaw = 0.0;
        // pitch = 0.0;
        // skew = 0.0;

        // pos = null;
        // corners = null;
    }

    /**
     * Runs the flywheel motors backwards at a constant motor percentage
     * to get balls out of the main shooter. 
     */
    public void flywheelIn() {
        leftFlywheelMotor.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
        rightFlywheelMotor.set(ControlMode.PercentOutput, -FLYWHEEL_MOTOR_PERCENT);
    }

    /**
     * Runs the flywheel motors forwards at a constant motor percentage to 
     * shoot balls out of the main shooter.
     */
    public void flywheelOut() {
        this.leftFlywheelMotor.set(ControlMode.PercentOutput, shootHoodPercent);
        this.rightFlywheelMotor.set(ControlMode.PercentOutput, shootHoodPercent);
    }

    /**
     * Runs the flywheel motors forwards at the percentage calculated
     * by the auto shoot code in {@link #periodic()} to shoot balls out 
     * of the main shooter. 
     */
    public void flywheelAutoShoot() {
        this.leftFlywheelMotor.set(ControlMode.PercentOutput, shootHoodPercent);
        this.rightFlywheelMotor.set(ControlMode.PercentOutput, shootHoodPercent);
    }

    /**
     * Stops the flywheel motors.
     */
    public void flywheelStop() {
        leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
        rightFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Runs the flywheel intake motor forwards to intake balls
     * into the main shooter.
     */
    public void flywheelIntakeIn() {
        flywheelIntakeMotor.set(ControlMode.PercentOutput, FLYWHEEL_INTAKE_MOTOR_PERCENT);
    }

    /**
     * Runs the flywheel intake motor backwards to outtake balls
     * into the ball hopper.
     */
    public void flywheelIntakeOut() {
        flywheelIntakeMotor.set(ControlMode.PercentOutput, -FLYWHEEL_INTAKE_MOTOR_PERCENT);
    }

    /**
     * Stops the flywheel intake motor.
     */
    public void flywheelIntakeStop() {
        flywheelIntakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Runs the flywheel motors and then the flywheel intake motor to shoot
     * balls, with timing based on the number of balls and whether the 
     * flywheel is currently running.
     * 
     * @param balls The number of balls to shoot.
     * @param flywheelRunning Whether or not the flywheel is running.
     */
    public void flywheelAndFlywheelIntakeRun(int balls, boolean flywheelRunning) {
        Timer timer = new Timer();
        timer.reset();
        timer.start();

        double time, flywheelIntakeTime;

        if (flywheelRunning) {
            time = 1 + (2 * balls);
            flywheelIntakeTime = 0;
        }
        else {
            time = 2.5 + (2 * balls);
            flywheelIntakeTime = 1.5;
        }
        
        initialAngle = currHoodAngle;
        targetAngle = currHoodAngle;
        targetDirection = Direction.FORWARD;

        while (timer.get() < time) {
            if (!flywheelRunning) {
                this.flywheelOut();
            }
            
            if (timer.get() > flywheelIntakeTime) {
                this.flywheelIntakeIn();
            }
        }

        timer.stop();
    }

    /**
     * Resets the encoder positions of the flywheel motors and 
     * the flywheel intake motor to zero.
     */
    public void resetPosition() {
        leftFlywheelMotor.setSelectedSensorPosition(0);
        rightFlywheelMotor.setSelectedSensorPosition(0);
        flywheelIntakeMotor.setSelectedSensorPosition(0);
    }

    /**
     * Sets the servo motor to the specified angle by subtracting
     * said angle from the servo offset, which is the real position 
     * of the servo.
     * 
     * @param angle The angle to set the servo motor to.
     */
    public void setHoodAngle(double angle) {
        this.hoodMotor.setAngle(servoOffset - angle);
    }

    /**
     * Returns the hood motor's current commanded position.
     * 
     * @return The hood motor's current commanded position.
     */
    public double getHoodAngle() {
        return this.hoodMotor.getAngle();
    }

    /**
     * Sets the hood motor's servo offset to the 
     * passed in offset.
     * 
     * @param offset Offset to set servo offset to (degrees).
     */
    public void setServoOffset(double offset) {
        this.servoOffset = offset;
    }
    /**
     * Returns the hood motor's current servo offset.
     * 
     * @return The hood motor's current servo offset.
     */
    public double getServoOffset() {
        return this.servoOffset;
    }

    /**
     * Sets the auto shoot math's trajectory constant
     * to the passed in constant.
     * 
     * @param constant Constant to set trajectory constant to.
     */
    public void setTrajectoryConstant(double constant) {
        this.trajectoryConstant = constant;
    }

    /**
     * Returns the auto shoot math's current 
     * trajectory constant.
     * 
     * @return The auto shoot math's current trajectory constant.
     */
    public double getTrajectoryConstant() {
        return this.trajectoryConstant;
    }

    /**
     * Moves the hood up by a value of 0.5 degrees using
     * {@link #setHoodAngle(double)}.
     */
    public void hoodUp() {
        setHoodAngle((servoOffset - currHoodAngle) + 0.5);
    }

    /** 
     * Moves the hood down by a value of 0.5 degrees using
     * {@link #setHoodAngle(double)}.
     */
    public void hoodDown() {
        setHoodAngle((servoOffset - currHoodAngle) - 0.5);
    }

    /** 
     * Sets the hood to its current angle using 
     * {@link #setHoodAngle(double)}.
     */
    public void hoodStop() {
        setHoodAngle(servoOffset - currHoodAngle);
    }

    /**
     * Turns the robot to be facing towards the hub using
     * {@link DriveSystem#percent(double, double)}.
     */
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

    /**
     * Stops the robot from turning to face towards the hub using
     * {@link DriveSystem#percent(double, double)}.
     */
    public void stopFlywheelPos() {
        Robot.drive.percent(0.0, 0.0);
    }

    /**
     * Returns whether the robot is facing the hub.
     * 
     * @return Whether or not the robot is facing the hub.
     */
    public boolean flywheelReachedPosition() {
        // return yaw <= 1 && yaw >= -1;
        return true;
    }

    /**
     * @deprecated Old auto turret swivel code, replaced by 
     * {@link #setFlywheelPos()}.
     * <p>
     * Turns the robot to face the hub.
     */
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

    /**
     * @deprecated Old turret reached position code, replaced by 
     * {@link #flywheelReachedPosition()}.
     * <p>
     * Returns whether the robot is facing the hub.
     * 
     * @return Whether or not the robot is facing the hub.
     */
	public boolean turretReachedPosition() {
		return targetOffsetAngle_Horizontal > -1.2 && targetOffsetAngle_Horizontal < 1.2;
	}

    /**
     * Returns the rotational velocity currently calculated
     * by the auto shoot code.
     * 
     * @return The currently calculated rotation velocity (RPM).
     */
    public double getFlywheelRPM() {
        return this.rotationalVelocity;
    }

    /**
     * Returns the hood angle currently calculated by the 
     * auto shoot code.
     * 
     * @return The currently calculated hood angle
     */
    public double getShootHoodAngle() {
        return this.shootHoodAngle;
    }

    /**
     * Runs the code within it every 20ms.
     * <p>
     * Used for calculating limelight data, hood angle data, and 
     * auto shoot data.
     */
    @Override
    public void periodic() {
        targetOffsetAngle_Horizontal = limelight.getEntry("tx").getDouble(0);
		targetOffsetAngle_Vertical = limelight.getEntry("ty").getDouble(0);
		targetArea = limelight.getEntry("ta").getDouble(0);
		targetSkew = limelight.getEntry("ts").getDouble(0);

        currHoodAngle = hoodMotor.getAngle();

        System.out.println("Hood Angle: " + currHoodAngle);

        // System.out.println(currHoodAngle);

        /*
         * Kinematic Equations:
         * vf = v0 + at
         * vf^2 = v0^2 + 2ad
         * d = (1/2(v0 + vf)t), or vagt
         * d = v0t + 1/2at^2
         */

        /*
         * Kinematics Calculation Notes:
         * v0x will equal our distance from the goal divided by our time to shoot, which is 2 seconds
         * NOTE: this is technically calculating average velocity, but x velocity is constant
         * since we now know v0x, we will calculate v0y by saying that it is:
         * v0y = (dy - 1/2at^2) / t
         * ... which is derived from d = v0t + 1/2at^2
         * now we can use v0x and v0y to calculate v and 0 (theta), which gives us 
         * the flywheel velocity and the hood angle
         * v0x will vary, however v0y will not, but this will still lead to varying v's and 0's
         */
        cameraTargetAngle = CAMERA_MOUNTING_ANGLE + targetOffsetAngle_Vertical;

        /* 
         * NOTE: change the trajectory constant at the end to edit the arc
         * Ex.: changing the trajectory constant to 1.9 should make it to where
         * the ball ends slightly lower than the goal height
         * when it goes into the goal
         * TODO: Consider using a different trajectory constant for different distances from the goal?
         */
        horizontalDistance = ((GOAL_HEIGHT - CAMERA_HEIGHT) / Math.tan(cameraTargetAngle * (Math.PI / 180))) * trajectoryConstant;

        initialXVelocity = horizontalDistance / TIME_TO_SHOOT;
        initialVelocity = Math.sqrt(Math.pow(initialXVelocity, 2) + Math.pow(INITIAL_Y_VELOCITY, 2));

        /* 
         * Multiplying by 60 and dividing by 2pi  
         * changes from radians per second to 
         * revolutions per minute (RPM) 
         */
        rotationalVelocity = ((initialVelocity / FLYWHEEL_RADIUS) * 60) / (2 * Math.PI); 
        
        shootHoodAngle = Math.atan(INITIAL_Y_VELOCITY / initialXVelocity) * (180 / Math.PI);
        
        // shootHoodPercent = rotationalVelocity * (1 / MAX_FLYWHEEL_RPM);
        shootHoodPercent = FLYWHEEL_MOTOR_PERCENT;

        initialAngle = getHoodAngle();

        if (!limitSwitch.get()) {
            System.out.println("True");
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
