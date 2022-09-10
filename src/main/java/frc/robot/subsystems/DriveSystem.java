package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Direction;
import frc.robot.commands.Odometry;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import java.lang.Math.*;

/**
 * The class for the drive system.
 */
public class DriveSystem extends SubsystemBase {
	// Drive train motor controllers.
	private CANSparkMax rightMaster;
	private CANSparkMax rightSlave;
	private CANSparkMax leftMaster;
	private CANSparkMax leftSlave;

	// Drive train motor encoders.
	private RelativeEncoder rightEncoder;
	private RelativeEncoder leftEncoder;

	// Drive train motor PID controllers.
	private SparkMaxPIDController leftPIDCont;
	private SparkMaxPIDController rightPIDCont;

    public enum Mode {
        AUTONOMOUS, TELEOP
    }

	// Onboard IMU.
	private AHRS navX;

	private static final double MAX_VELOCITY = 5500;
	private static final double TURBO_VELOCITY = 8000;
	private static final double QUARTER_VELOCITY = 1000;
	private static final double PEAK_OUTPUT = 1.0;
	private boolean turbo = false;
	private boolean quarter = false;

	// Wheel specific constants.
	public static final double TICKS_PER_ROTATION = 42.0;
	public static final double WHEEL_DIAMETER = 6.0;
	public static final double WHEEL_DIAMETER_M = 0.1524;
	public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	public static final double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI;
	public static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
	public static final double TICKS_PER_METER = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_M;
	public static final double METERS_PER_TICK = WHEEL_CIRCUMFERENCE_M / TICKS_PER_ROTATION;

	// Unit specific constants.
	public static final double INCHES_PER_METER = 39.3701;
	public static final double METERS_PER_INCH = 1 / 39.3701;

	// Velocity PIDF values *Don't mess with, pretty much perfect
	public static final double VELOCITY_P = 0.0003; // 0.3 / 1000 RPM Error
	public static final double VELOCITY_I = 0.0;
	public static final double VELOCITY_D = 0.003; // will tune
	public static final double VELOCITY_I_ZONE = 0.0;
	public static final double VELOCITY_FEED_FORWARD = 8.75E-5; // 0.35 / MAX_VELOCITY

	// Position PIDF values
	public static final double POSITION_P = 0.3 / 50; // 0.3 / 50 Rotation Error
	public static final double POSITION_I = 0.0;
	public static final double POSITION_D = POSITION_P * 10; // will tune
	public static final double POSITION_I_ZONE = 0.0;
	public static final double POSITION_FEED_FORWARD = 0.0;

	// Feed Forward Gains
	// kS - the voltage needed to overcome the motor's static friction (V).
	// kV - the voltage needed to maintain a given constant velocity (V * s/m).
	// kA - the voltage needed to induce a given acceleration (V * s^2/m).
	public static final double FEED_FORWARD_KS = 0.16203;
	public static final double FEED_FORWARD_KV = 0.13098;
	public static final double FEED_FORWARD_KA = 0.026747;

	public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(FEED_FORWARD_KS,
			FEED_FORWARD_KV, FEED_FORWARD_KA);

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
	public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

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
		Normal, Inverted
	}

	private DriveMode driveMode = DriveMode.Normal;

	private Odometry odometry;
	private Pose2d robotPos;
	private Rotation2d robotAngle;

	private double target;

	private static int navXCount = 0;

	private double prevLeft, prevRight;

	/**
	 * Constructor for the drive system.
	 */
	public DriveSystem() {
		// Initialize all of the drive system's motor controllers.
		this.leftMaster = new CANSparkMax(Constants.LEFT_MASTER_MOTOR,MotorType.kBrushless);
		this.leftSlave = new CANSparkMax(Constants.LEFT_SLAVE_MOTOR,MotorType.kBrushless);
		this.rightMaster = new CANSparkMax(Constants.RIGHT_MASTER_MOTOR,MotorType.kBrushless);
		this.rightSlave = new CANSparkMax(Constants.RIGHT_SLAVE_MOTOR,MotorType.kBrushless);

		this.leftEncoder = leftMaster.getEncoder();
		this.rightEncoder = rightMaster.getEncoder();

		this.leftPIDCont = leftMaster.getPIDController();
		leftPIDCont.setFeedbackDevice(leftEncoder);

		this.rightPIDCont = rightMaster.getPIDController();
		rightPIDCont.setFeedbackDevice(rightEncoder);

		this.leftSlave.follow(leftMaster);
		this.rightSlave.follow(rightMaster);
		
		this.rightMaster.setIdleMode(IdleMode.kBrake);
		this.rightSlave.setIdleMode(IdleMode.kBrake);
		this.leftMaster.setIdleMode(IdleMode.kBrake);
		this.leftSlave.setIdleMode(IdleMode.kBrake);

		this.rightMaster.setClosedLoopRampRate(0.75);
		this.rightSlave.setClosedLoopRampRate(0.75);
		this.leftMaster.setClosedLoopRampRate(0.75);
		this.leftSlave.setClosedLoopRampRate(0.75);

		// Initialize the NavX IMU sensor.
		this.navX = new AHRS(SPI.Port.kMXP);

		this.robotPos = new Pose2d(0, 0, new Rotation2d());
		this.odometry = new Odometry(Rotation2d.fromDegrees(-navX.getAngle()), robotPos);

		targetPosition = null;
	}

	/**
	 * Returns the PID controller object for our left motor ({@link #leftPidCont}).
	 * 
	 * @return Left motor PID controller object.
	 */
	public SparkMaxPIDController getLeftPIDCont() {
		return leftPIDCont;
	}

	/**
	 * Return the PID controller object for our right motor ({@link #rightPIDCont}).
	 * 
	 * @return Right motor PID controller object.
	 */
	public SparkMaxPIDController getRightPIDCont() {
		return rightPIDCont;
	}

	/**
	 * Returns the motor object for our left follower motor ({@link #leftSlave}).
	 * 
	 * @return Left follower motor object.
	 */
	public CANSparkMax getLeftSlave() {
		return this.leftSlave;
	}

	/**
	 * Returns the motor object for our right follower motor ({@link #rightSlave}).
	 * 
	 * @return Right follower motor object.
	 */
	public CANSparkMax getRightSlave() {
		return this.rightSlave;
	}

	/**
	 * Returns the motor object for our left main motor ({@link #leftMaster}).
	 * 
	 * @return Left main motor object.
	 */
	public CANSparkMax getLeftMaster() {
		return this.leftMaster;
	}

	/**
	 * Returns the motor object for our right main motor ({@link #rightMaster}).
	 * 
	 * @return Right main motor object.
	 */
	public CANSparkMax getRightMaster() {
		return this.rightMaster;
	}

	/**
	 * Returns the object for our NavX gyroscope.
	 * 
	 * @return NavX gyroscope.
	 */
	public AHRS getNavX() {
		return navX;
	}

	/**
	 * Returns the distance traveled by the robot.
	 * 
	 * @return The distance traveled (rotations).
	 */
	public double getPosition() {
		return this.leftEncoder.getPosition();
	}

	/**
	 * Returns the velocity of the robot.
	 * 
	 * @return The velocity of the robot (RPM). 
	 */
	public double getVelocity() {
		return this.leftEncoder.getVelocity();
	}

	/**
	 * Sets quarter velocity ({@link #quarter}) for our drivetrain state to true.
	 */
	public void quarterTrue() {
		quarter = true;
	}

	/**
	 * Sets quarter velocity ({@link #quarter}) for our drivetrain state to false.
	 */
	public void quarterFalse() {
		quarter = false;
	}

	/**
	 * Toggles drive mode ({@link #driveMode}) of our drive train between normal and inverted state.
	 */
	public void toggleDriveMode() {
		if (this.driveMode == DriveMode.Normal) {
			this.driveMode = DriveMode.Inverted;
		} else {
			this.driveMode = DriveMode.Normal;
		}
	}

	/**
	 * Sets the PIDF values of the passed in PID 
	 * controller depending on the drive mode ({@link Mode}).
	 * 
	 * @param controller The controller to have its PIDF values set.
	 * @param mode The drive mode.
	 */
	public void setPIDF(SparkMaxPIDController controller, Mode mode) {
		if (mode == Mode.AUTONOMOUS) {
			controller.setP(POSITION_P, Constants.POSITION_SLOT_ID);
			controller.setI(POSITION_I, Constants.POSITION_SLOT_ID);
			controller.setD(POSITION_D, Constants.POSITION_SLOT_ID);
			controller.setIZone(POSITION_I_ZONE, Constants.POSITION_SLOT_ID);
			controller.setFF(POSITION_FEED_FORWARD, Constants.POSITION_SLOT_ID);

			controller.setP(POSITION_P);
			controller.setI(POSITION_I);
			controller.setD(POSITION_D);
			controller.setIZone(POSITION_I_ZONE);
			controller.setFF(POSITION_FEED_FORWARD);
		}
		else if (mode == Mode.TELEOP) {
			controller.setP(VELOCITY_P, Constants.VELOCITY_SLOT_ID);
			controller.setI(VELOCITY_I, Constants.VELOCITY_SLOT_ID);
			controller.setD(VELOCITY_D, Constants.VELOCITY_SLOT_ID);
			controller.setIZone(VELOCITY_I_ZONE, Constants.VELOCITY_SLOT_ID);
			controller.setFF(VELOCITY_FEED_FORWARD, Constants.VELOCITY_SLOT_ID);

			controller.setP(VELOCITY_P);
			controller.setI(VELOCITY_I);
			controller.setD(VELOCITY_D);
			controller.setIZone(VELOCITY_I_ZONE);
			controller.setFF(VELOCITY_FEED_FORWARD);
		}
	}

	/**
	 * Sets the PIDF values of the passed in PID controller
	 * depending on the passed in values.
	 * 
	 * @param controller The controller to have its PIDF values set.
	 * @param p The p value.
	 * @param i The i value.
	 * @param d The d value.
	 * @param iz The i zone value.
	 * @param ff The feed forward value.
	 * @param slotID The id for the slot that these values will be stored at.
	 */
	public void setPIDF(SparkMaxPIDController controller, double p, double i, double d, double iz, double ff, int slotID) {
		controller.setP(p, slotID);
		controller.setI(i, slotID);
		controller.setD(d, slotID);
		controller.setIZone(iz, slotID);
		controller.setFF(ff, slotID);

		controller.setP(p);
		controller.setI(i);
		controller.setD(d);
		controller.setIZone(iz);
		controller.setFF(ff);
	}

	/**
	 * Sets the robot to reach the passed in setpoint corresponding
	 * with the passed in type to read values from (velocity or 
	 * position) with the PID values from the slot specified
	 * by the passed in slot ID.
	 * 
	 * @param setpoint The setpoint to set the PID control loop to.
	 * @param type The type of value for the PID control loop to read (velocity or position).
	 * @param slotID The slot ID for the desired PID slot values.
	 */
	public void pidDrive(double setpoint, ControlType type, int slotID) {
		this.leftPIDCont.setReference(setpoint, type, slotID);
		this.rightPIDCont.setReference(setpoint, type, slotID);
	}

	/**
	 * Returns the robot's current position.
	 * 
	 * @return Robot's position.
	 */
	public Pose2d getRobotPos() {
		return robotPos;
	}

	/**
	 * Returns the robot's current angle.
	 * 
	 * @return Robot's angle.
	 */
	public double getRobotAngle() {
		return robotAngle.getDegrees();
	}

	/**
	 * Returns the current target position.
	 * 
	 * @return Target position.
	 */
	public Pose2d getTargetPos() {
		return targetPosition;
	}

	/**
	 * Returns whether the robot has reached its current set position.
	 * 
	 * @return Whether or not the robot has reached the target.
	 */
	public boolean reachedPosition() {
		double robotX = robotPos.getX();
		double robotY = robotPos.getY();

		System.out.println("X: " + robotX + "\n" + "Y: " + robotY);

		double targetX = targetPosition.getX();
		double targetY = targetPosition.getY();

		System.out.println("Target Distance: " + target);

		return
		(robotX >= (targetX - 0.25) && robotX <= (targetX + 0.25)) 
		&& 
		(robotY >= (targetY - 0.25) && robotY <= (targetY + 0.25));
	}

	/**
	 * @deprecated Old reached curve code, to be replaced by odometry.
	 * <p>
	 * Returns whether the robot has reached a set curve based on its targets.
	 * 
	 * @param targetL Target for the left side of the drive train (inches).
	 * @param targetR Target for the right side of the drive train (inches).
	 * @return Whether or not the robot has reached the curve.
	 */
	public boolean reachedCurve(double targetL, double targetR) {
		double leftPos = this.leftEncoder.getPosition();
		double rightPos = this.rightEncoder.getPosition();

		if (targetDirection == Direction.FORWARD) {
			return (leftPos >= targetL) && (rightPos >= targetR);
		} else if (targetDirection == Direction.BACKWARD) {
			return (leftPos <= targetL) && (rightPos <= targetR);
		}
		return true;
	}

	/**
	 * Calculates the distance and angle at the robot has to drive to reach
	 * the passed in Cartesian coordinates.
	 * 
	 * @param xMeters Horizontal distance to travel in meters.
	 * @param yMeters Vertical distance to travel in meters.
	 * @return The target position ({@link #targetPosition}).
	 */
	public Pose2d driveToPosition(double xMeters, double yMeters) {
		// Creating translation for target position based on x and y params.
		Translation2d translation = new Translation2d(xMeters, yMeters);

		// Variables to calculate distance from robot to target and angle from robot to target.
		double targetX = translation.getX();
		double targetY = translation.getY();
		double initRobotX = robotPos.getX();
		double initRobotY = robotPos.getY();
		double offsetX = targetX - initRobotX;
		double offsetY = targetY - initRobotY;

		/* 
		 * Creating angle for target position based on the target position 
		 * and the robot's current position by calculating the angle from 
		 * the robot's current position to the robot's target position.
		 */
		Rotation2d angle = Rotation2d.fromDegrees(Math.atan(offsetY / offsetX) * (180 / Math.PI));

		// Used to change angle due to atan()'s limited range.
		if ((offsetX < 0 && offsetY > 0) || (offsetX < 0 && offsetY < 0)) {
			angle = Rotation2d.fromDegrees(angle.getDegrees() + 180);
		}
		else if (offsetX > 0 && offsetY < 0) {
			angle = Rotation2d.fromDegrees(angle.getDegrees() + 360);
		}

		// Target position stored for later use.
		targetPosition = new Pose2d(translation, angle);

		// Distance from robot to target.
		target = Math.sqrt(Math.pow(offsetX, 2) + Math.pow(offsetY, 2));

		/* 
		 * Determines what direction is the shortest turn direction
		 * for the robot during turn commands based on the target angle
		 * and the robot's current angle.
		 */
		if (targetPosition.getRotation().getDegrees() > robotAngle.getDegrees()) {
			if (targetPosition.getRotation().getDegrees() - robotAngle.getDegrees() > 180) {
				turnDirection = Direction.RIGHT;
			}
			else if (targetPosition.getRotation().getDegrees() - robotAngle.getDegrees() <= 180) {
				turnDirection = Direction.LEFT;
			}
		}
		else if (targetPosition.getRotation().getDegrees() <= robotAngle.getDegrees()) {
			if (targetPosition.getRotation().getDegrees() - robotAngle.getDegrees() > -180) {
				turnDirection = Direction.LEFT;
			}
			else if (targetPosition.getRotation().getDegrees() - robotAngle.getDegrees() <= -180) {
				turnDirection = Direction.RIGHT;
			}
		}	
		else {
			turnDirection = Direction.LEFT;
		}

		// Sets drive direction to forward, as robot will have already turned to the set angle.
		driveDirection = Direction.FORWARD;

		return targetPosition;
	}
	
	/**
	 * Makes the robot drive the distance calculated in 
	 * {@link #driveToPosition(double, double)} ({@link #target})
	 */
	public void driveDistance() {
		double targetDist = target;

		System.out.println("Target Distance: " + targetDist);
		
		if (driveDirection == Direction.FORWARD) {
			targetDist = (targetDist * INCHES_PER_METER) / WHEEL_CIRCUMFERENCE;
		} else if (driveDirection == Direction.BACKWARD) {
			targetDist = (-1 * targetDist * INCHES_PER_METER) / WHEEL_CIRCUMFERENCE;
		} else {
			targetDist = 0;
		}

		this.leftPIDCont.setReference(targetDist, ControlType.kPosition, Constants.POSITION_SLOT_ID);
		this.rightPIDCont.setReference(-targetDist, ControlType.kPosition, Constants.POSITION_SLOT_ID);
	}

	/**
	 * @deprecated Old drive distance code, replaced by {@link #driveDistance()}.
	 * <p>
	 * Makes the robot drive the specified distance in inches in the 
	 * specified direction.
	 * 
	 * @param inches The number of inches for the robot to drive.
	 * @param direction The direction for the robot to drive in.
	 */
	public void oldDriveDistance(double inches, Direction direction) {
		targetDirection = direction;

		if (targetDirection == Direction.FORWARD) {
			oldTargetPosition = (inches / WHEEL_CIRCUMFERENCE);
		} else if (targetDirection == Direction.BACKWARD) {
			oldTargetPosition = -1 * (inches / WHEEL_CIRCUMFERENCE);
		} else {
			oldTargetPosition = 0;
		}

		this.leftPIDCont.setReference(oldTargetPosition, ControlType.kPosition, Constants.POSITION_SLOT_ID);
		this.rightPIDCont.setReference(-oldTargetPosition, ControlType.kPosition, Constants.POSITION_SLOT_ID);
	}

	/**
	 * @deprecated Old reached position code, replaced by {@link #reachedPosition()}.
	 * <p>
	 * Returns whether the robot has reached the target position.
	 * 
	 * @return Whether or not the robot has reached the target.
	 */
	public boolean oldReachedPosition() {
		double leftPos = leftEncoder.getPosition();
		double rightPos = rightEncoder.getPosition();

		if (targetDirection == Direction.FORWARD) {
			return (leftPos >= oldTargetPosition) && (rightPos >= oldTargetPosition);
		} else if (targetDirection == Direction.BACKWARD) {
			return (leftPos <= oldTargetPosition) && (rightPos <= oldTargetPosition);
		} 
		return true;
	}

	/**
	 * @deprecated Old drive curve code, to be replaced by odometry.
	 * <p>
	 * Makes the robot drive in a curve in the specified direction by setting the 
	 * left and right sides of the drive train to drive different distances.
	 * 
	 * @param leftDist Target for the left side of the drive train (inches).
	 * @param rightDist Target for the right side of the drive train (inches).
	 * @param direction The direction for the robot to drive in.
	 */
	public void driveCurve(double leftDist, double rightDist, Direction direction) {
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

		this.leftPIDCont.setReference(leftDist, ControlType.kPosition, Constants.POSITION_SLOT_ID);
		this.rightPIDCont.setReference(rightDist, ControlType.kPosition, Constants.POSITION_SLOT_ID);
	}

	/**
	 * @deprecated Old drive circle code, to be replaced by odometry.
	 * <p>
	 * Makes the robot drive in a circle that has a radius of the radius passed in
	 * for the amount of degrees specified by the passed in angle that goes in 
	 * the passed in direction. 
	 * 
	 * @param angle The degrees to be traveled.
	 * @param direction The direction to travel in.
	 * @param radius The radius of the circle.
	 */
	public void driveCircle(double angle, Direction direction, double radius) {
		double innerCircumference = radius * 2 * Math.PI * (angle / 360);
		double outerCircumference = (radius + 24) * 2 * Math.PI * (angle / 360);

		double leftDist, rightDist;
		
		if (direction == Direction.LEFT) {
			leftDist = innerCircumference / WHEEL_CIRCUMFERENCE;
			rightDist = outerCircumference / WHEEL_CIRCUMFERENCE;
		}
		else if (direction == Direction.RIGHT) {
			leftDist = outerCircumference / WHEEL_CIRCUMFERENCE;
			rightDist = innerCircumference / WHEEL_CIRCUMFERENCE;
		}
		else {
			leftDist = 0;
			rightDist = 0;
		}

		this.leftPIDCont.setReference(leftDist, ControlType.kPosition, Constants.POSITION_SLOT_ID);
		this.rightPIDCont.setReference(rightDist, ControlType.kPosition, Constants.POSITION_SLOT_ID);
	}

	/**
	 * @deprecated Old reached circle code, to be replaced by odometry.
	 * <p>
	 * Checks whether the robot has reached the end of the target circle based
	 * on the degrees to be traveled from the passed in angle, the radius of
	 * the circle, and the direction to be traveled in.
	 * 
	 * @param angle The degrees to be traveled.
	 * @param radius The radius of the circle.
	 * @param direction The direction to travel in.
	 * @return Whether or not the robot has reached the end of the target circle.
	 */
	public boolean reachedCircle(double angle, double radius, Direction direction) {
		double leftPos = this.leftEncoder.getPosition();
		double rightPos = this.rightEncoder.getPosition();

		double rightTarget, leftTarget;

		if (direction == Direction.LEFT) {
			leftTarget = radius * 2 * Math.PI * (angle / 360) / WHEEL_CIRCUMFERENCE;
			rightTarget = (radius + 24) * 2 * Math.PI * (angle / 360) / WHEEL_CIRCUMFERENCE;

			return (leftPos >= leftTarget) && (rightPos >= rightTarget);
		}
		else if (direction == Direction.RIGHT) {
			rightTarget = radius * 2 * Math.PI * (angle / 360) / WHEEL_CIRCUMFERENCE;
			leftTarget = (radius + 24) * 2 * Math.PI * (angle / 360) / WHEEL_CIRCUMFERENCE;

			return (leftPos >= leftTarget) && (rightPos >= rightTarget);
		}
		return true;
	}

	/**
	 * Sets the turbo drive mode state to true.
	 */
	public void setTurboTrue() {
		this.turbo = true;
	}

	/**
	 * Sets the turbo drive mode state to false.
	 */
	public void setTurboFalse() {
		this.turbo = false;
	}
	
	/**
	 * The main drive function for our tele-op drive code. The velocity for
	 * the left and right sides of the drive train is a percentage of 
	 * the max velocity defined, with the percentages being the values 
	 * passed in.
	 * 
	 * @param left The percentage of max velocity for the left side.
	 * @param right The percentage of max velocity for the right side.
	 */
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
		
		this.leftPIDCont.setReference(-targetLeft, ControlType.kVelocity, Constants.VELOCITY_SLOT_ID);
		this.rightPIDCont.setReference(targetRight, ControlType.kVelocity, Constants.VELOCITY_SLOT_ID);
	}

	/**
	 * Makes the robot's left and right sides of the drive train
	 * drive percentages of the max voltage defined.
	 * 
	 * @param left The percentage of max voltage for the left side.
	 * @param right The percentage of max voltage for the right side.
	 */
	public void voltage(double left, double right) {
		leftMaster.setVoltage(left * 12.0);
		rightMaster.setVoltage(right * 12.0);
	}

	/**
	 * Makes the robot's left and right sides of the drive train
	 * drive at their respective percentages of max throttle 
	 * passed in.
	 * 
	 * @param left The percentage of max throttle for the left side.
	 * @param right The percentage of max throttle for the right side.
	 */
	public void percent(double left, double right) {
		this.leftPIDCont.setReference(left, ControlType.kDutyCycle);
		this.rightPIDCont.setReference(right, ControlType.kDutyCycle);
	}

	/**
	 * Resets the angle of the NavX to zero.
	 */
	public void resetAngle() {
		navX.reset();
	}

	/**
	 * Resets the position of the left and right motor encoders
	 * to zero.
	 */
	public void resetPosition() {
		this.leftEncoder.setPosition(0);
		this.rightEncoder.setPosition(0);
	}

	/**
	 * Turns the robot in the set direction ({@link #turnDirection})
	 * calculated via {@link #driveToPosition(double, double)} at the 
	 * passed in speed.
	 * 
	 * @param speed Percentage of voltage to set the robot to using {@link #percent(double, double)}.
	 */
	public void turn(double speed) {
		switch (turnDirection) {
		case LEFT:
			this.percent(-speed, -speed);
			break;
		case RIGHT:
 
		this.percent(speed, speed);
			break;
		default:
			this.percent(0, 0);
		}
	}

	/**
	 * Turns the robot in the passed in direction at the passed in speed.
	 * 
	 * @param speed Percentage of voltage to set the robot to using {@link #percent(double, double)}.
	 * @param direction The direction to turn in.
	 */
	public void oldTurn(double speed, Direction direction) {
		switch (direction) {
		case LEFT:
			this.percent(-speed, -speed);
			break;
		case RIGHT:
			this.percent(speed, speed);
			break;
		default:
			this.percent(0, 0);
		}
	}

	/**
	 * Returns whether the robot has reached the passed in angle.
	 * This method is for use with the {@link #oldTurn(double, Direction)} method.
	 * 
	 * @param angle The angle to check.
	 * @return Whether or not the robot has reached the angle.
	 */
	public boolean oldReachedTurn(double angle) {
		return robotAngle.getDegrees() <= angle + 2 && robotAngle.getDegrees() >= angle - 2;
	}

	/**
	 * Returns whether the robot has reached the target angle calculated in
	 * {@link #driveToPosition(double, double)}.
	 * This method is for use with the {@link #turn(double)} method.
	 * 
	 * @return Whether or not the robot has reached the angle.
	 */
	public boolean reachedTurn() {
		double angle = targetPosition.getRotation().getDegrees();
		
		return robotAngle.getDegrees() <= angle + 2 && robotAngle.getDegrees() >= angle - 2;
	}

	/**
	 * Runs the code within it every 20ms.
	 * <p>
	 * Used for calculating the robot's position via odometry.
	 */
	@Override
	public void periodic() {
		double leftDist = leftEncoder.getPosition() * WHEEL_CIRCUMFERENCE_M;
		double rightDist = -(rightEncoder.getPosition() * WHEEL_CIRCUMFERENCE_M);

		Rotation2d navXAngle = Rotation2d.fromDegrees(-navX.getAngle());

		robotPos = odometry.update(navXAngle, leftDist, rightDist);

		robotAngle = Rotation2d.fromDegrees(-navX.getAngle() % 360);
	}
}
