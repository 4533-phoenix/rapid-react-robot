package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Direction;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveSystem extends SubsystemBase {
    
	// Drive train motor controllers
	private WPI_TalonSRX rightMaster;
	private WPI_TalonSRX rightSlave;
	private WPI_TalonSRX leftMaster;
	private WPI_TalonSRX leftSlave;

	// Onboard IMU.
	private AHRS navX;

	private static final double MAX_VELOCITY = 400;
	private static final double TURBO_VELOCITY = 475;
	private static final double PEAK_OUTPUT = 1.0;
	private boolean turbo = false;

	// Wheel specific constants.
	public static final double TICKS_PER_ROTATION = 4096.0;
	private static final double WHEEL_DIAMETER = 6.0;
	private static final double WHEEL_DIAMETER_M = 0.1524;
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	private static final double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI;
	private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
	private static final double TICKS_PER_METER = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_M;
	private static final double METERS_PER_TICK = WHEEL_CIRCUMFERENCE_M / TICKS_PER_ROTATION;

	// Velocity PID Gains and Feed Forward values.
	//
	// The following values should be used when driving the robot in "Velocity"
	// mode.
	public static final double VELOCITY_P = 0.000213;
	public static final double VELOCITY_I = 0.0;
	public static final double VELOCITY_D = 0.0;
	public static final double VELOCITY_FEED_FORWARD = 0.243;

	// Position PID Gains and Feed Forward values.
	//
	// The following values should be used when driving the robot in "Position"
	// mode.
	public static final double POSITION_P = 0.06;
	public static final double POSITION_I = 0.0;
	public static final double POSITION_D = 0.0;
	public static final double POSITION_FEED_FORWARD = 0.0;

	// Feed Forward Gains
	// kS - the voltage needed to overcome the motor's static friction (V).
	// kV - the voltage needed to maintain a given constant velocity (V * s/m).
	// kA - the voltage needed to induce a given acceleration (V * s^2/m).
	public static final double FEED_FORWARD_KS = 0.456;
	public static final double FEED_FORWARD_KV = 0.145;
	public static final double FEED_FORWARD_KA = 0.0227;

	public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(FEED_FORWARD_KV,
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

	private static double targetPosition = 0;
	private static Direction targetDirection;

	public enum DriveMode {
		Normal, Inverted
	}

	private DriveMode driveMode = DriveMode.Normal;

	public DriveSystem() {
		// Initialize all of the drive systems motor controllers.
		this.leftMaster = new WPI_TalonSRX(Constants.LEFT_MASTER_MOTOR);
		this.leftSlave = new WPI_TalonSRX(Constants.LEFT_SLAVE_MOTOR);
		this.rightMaster = new WPI_TalonSRX(Constants.RIGHT_MASTER_MOTOR);
		this.rightSlave = new WPI_TalonSRX(Constants.RIGHT_SLAVE_MOTOR);

		this.leftSlave.follow(leftMaster, FollowerType.AuxOutput1);
		this.rightSlave.follow(rightMaster, FollowerType.AuxOutput1);

		this.leftMaster.setInverted(true);
		this.leftSlave.setInverted(true);

		this.leftMaster.setSensorPhase(true);
		this.rightMaster.setSensorPhase(true);

		this.leftMaster.configPeakOutputForward(PEAK_OUTPUT);
		this.leftMaster.configPeakOutputReverse(-PEAK_OUTPUT);
		this.leftSlave.configPeakOutputForward(PEAK_OUTPUT);
		this.leftSlave.configPeakOutputReverse(-PEAK_OUTPUT);

		this.rightMaster.configPeakOutputForward(PEAK_OUTPUT);
		this.rightMaster.configPeakOutputReverse(-PEAK_OUTPUT);
		this.rightSlave.configPeakOutputForward(PEAK_OUTPUT);
		this.rightSlave.configPeakOutputReverse(-PEAK_OUTPUT);

		this.leftMaster.configNominalOutputForward(0, DEFAULT_TIMEOUT);
		this.leftMaster.configNominalOutputReverse(0, DEFAULT_TIMEOUT);
		this.leftSlave.configNominalOutputForward(0, DEFAULT_TIMEOUT);
		this.leftSlave.configNominalOutputReverse(0, DEFAULT_TIMEOUT);

		this.rightMaster.configNominalOutputForward(0, 30);
		this.rightMaster.configNominalOutputReverse(0, 30);
		this.rightSlave.configNominalOutputForward(0, 30);
		this.rightSlave.configNominalOutputReverse(0, 30);

		this.leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, DEFAULT_TIMEOUT);
		this.rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, DEFAULT_TIMEOUT);
		
		this.rightMaster.setNeutralMode(NeutralMode.Brake);
		this.rightSlave.setNeutralMode(NeutralMode.Brake);
		this.leftMaster.setNeutralMode(NeutralMode.Brake);
		this.leftSlave.setNeutralMode(NeutralMode.Brake);

		// Initialize the NavX IMU sensor.
		this.navX = new AHRS(SPI.Port.kMXP);
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

	public void setPIDF(double p, double i, double d, double f) {
		// Set PIDF for Left Master controller.
		this.leftMaster.config_kP(0, p, 100);
		this.leftMaster.config_kI(0, i, 100);
		this.leftMaster.config_kD(0, d, 100);
		this.leftMaster.config_kF(0, f, 100);

		// Set PIDF for Right Master controller.
		this.rightMaster.config_kP(0, p, 100);
		this.rightMaster.config_kI(0, i, 100);
		this.rightMaster.config_kD(0, d, 100);
		this.rightMaster.config_kF(0, f, 100);
	}

	public SlotConfiguration[] getPID() {
		SlotConfiguration[] slots = new SlotConfiguration[]{
			new SlotConfiguration(),
			new SlotConfiguration()
		};

		this.leftMaster.getSlotConfigs(slots[0], 0, 30);
		this.rightMaster.getSlotConfigs(slots[1], 0, 30);

		return slots;
	}

	public void resetPosition() {
		this.rightMaster.setSelectedSensorPosition(0);
		this.leftMaster.setSelectedSensorPosition(0);
	}

	public boolean reachedPosition() {
		double leftPos = this.leftMaster.getSelectedSensorPosition();
		double rightPos = this.rightMaster.getSelectedSensorPosition();

		System.out.println("Left: " + leftPos + " Right: " + rightPos);
		if (targetDirection == Direction.FORWARD) {
			System.out.printf("%f : %f : %f\n", leftPos, rightPos, targetPosition);
			return (leftPos <= targetPosition) && (rightPos <= targetPosition);
		} else if (targetDirection == Direction.BACKWARD) {
			return (leftPos >= targetPosition) && (rightPos >= targetPosition);
		} else {
			return true;
		}
	}

	public boolean reachedCurve(double targetL, double targetR) {
		double leftPos = this.leftMaster.getSelectedSensorPosition();
		double rightPos = this.rightMaster.getSelectedSensorPosition();

		if (targetDirection == Direction.FORWARD) {
			return (leftPos >= targetL) && (rightPos >= targetR);
		} else if (targetDirection == Direction.BACKWARD) {
			return (leftPos <= targetL) && (rightPos <= targetR);
		} else {
			return true;
		}
	}

	public void driveDistance(double inches, Direction direction) {
		targetDirection = direction;
		if (direction == Direction.FORWARD) {
			targetPosition = -1 * inches * TICKS_PER_INCH;
		} else if (direction == Direction.BACKWARD) {
			targetPosition = inches * TICKS_PER_INCH;
		} else {
			targetPosition = 0;
		}

		this.leftMaster.set(ControlMode.Position, targetPosition);
		this.rightMaster.set(ControlMode.Position, targetPosition);
	}

	public void driveCurve(double leftDist, double rightDist, Direction direction) {
		targetDirection = direction;
		if (direction == Direction.FORWARD) {
			leftDist = -1 * (leftDist * TICKS_PER_INCH);
			rightDist = -1 * (rightDist * TICKS_PER_INCH);
		} else if (direction == Direction.BACKWARD) {
			leftDist = leftDist * TICKS_PER_INCH;
			rightDist = rightDist * TICKS_PER_INCH;
		} else {
			leftDist = 0;
			rightDist = 0;
		}
		this.leftMaster.set(ControlMode.Position, leftDist);
		this.rightMaster.set(ControlMode.Position, rightDist);
	}

	public void driveCircle(double speed, double angle, Direction direction, double radius) {
		double innerCircumference = radius * 2 * Math.PI * (angle / 360);
		double outerCircumference = (radius + 24) * 2 * Math.PI * (angle / 360);

		/*double innerVelocity = -1 * (speed * (innerCircumference / outerCircumference)) * MAX_VELOCITY * 4096 / 600.0;
		double outerVelocity = -1 * speed * MAX_VELOCITY * 4096 / 600.0;*/

		//double outerVelocity = -1 * (speed * ((innerCircumference + outerCircumference) / innerCircumference)) * MAX_VELOCITY * 4096 / 600.0;
		// double innerVelocity = -1 * speed * MAX_VELOCITY * 4096 / 600.0;
		// double outerVelocity = (innerVelocity * outerCircumference) / innerCircumference;

		
		double leftDist, rightDist;
		//double leftVelocity, rightVelocity;
		
		if (direction == Direction.LEFT) {
			leftDist = innerCircumference;
			rightDist = outerCircumference;
			
			// leftVelocity = innerVelocity;
			// rightVelocity = outerVelocity;
		}
		else if (direction == Direction.RIGHT) {
			leftDist = outerCircumference;
			rightDist = innerCircumference;
			
			// leftVelocity = outerVelocity;
			// rightVelocity = innerVelocity;
		}
		else {
			leftDist = 0;
			rightDist = 0;
			
			// leftVelocity = 0;
			// rightVelocity = 0;
		}
		
		// System.out.println("Left Velocity: " + leftVelocity);
		// System.out.println("Right Velocity: " + rightVelocity);
		this.leftMaster.set(ControlMode.Position, leftDist);
		this.rightMaster.set(ControlMode.Position, rightDist);
	}

	public boolean reachedCircle(double angle, double radius, Direction direction) {
		double leftPos = this.leftMaster.getSelectedSensorPosition();
		double rightPos = this.rightMaster.getSelectedSensorPosition();

		double rightTarget, leftTarget;

		if (direction == Direction.LEFT) {
			leftTarget = radius * 2 * Math.PI * (angle / 360) * TICKS_PER_INCH;
			rightTarget = (radius + 24) * 2 * Math.PI * (angle / 360) * TICKS_PER_INCH;
		}
		else if (direction == Direction.RIGHT) {
			rightTarget = radius * 2 * Math.PI * (angle / 360) * TICKS_PER_INCH;
			leftTarget = (radius + 24) * 2 * Math.PI * (angle / 360) * TICKS_PER_INCH;
		}
		else {
			rightTarget = 0;
			leftTarget = 0;
		}

		System.out.println("Position: " + leftPos + "    " + rightPos);
		System.out.println("Target: " + leftTarget + "    " + rightTarget);


		return rightPos >= rightTarget;
	}

	public double getPosition() {
		return this.leftMaster.getSelectedSensorPosition() / TICKS_PER_INCH;
	}

	public double getLeftDistance() {
		return this.leftMaster.getSelectedSensorPosition() / TICKS_PER_METER;
	}

	public double getRightDistance() {
		return this.rightMaster.getSelectedSensorPosition() / TICKS_PER_METER;
	}

	public void toggleTurbo() {
		this.turbo = !this.turbo;
	}

	public void setTurbo(boolean isTriggerPressed) {
		if (isTriggerPressed) {
			this.turbo = true;
		}
		else if (!isTriggerPressed) {
			this.turbo = false;
		}
		else {
			this.turbo = false;
		}
	}
	
	public void tank(double left, double right) {
		double targetLeft;
		double targetRight;

		double targetVelocity = MAX_VELOCITY;

		if (this.turbo) {
			targetVelocity = TURBO_VELOCITY;
		}

		targetLeft = left * targetVelocity * 4096 / 600.0;
		targetRight = right * targetVelocity * 4096 / 600.0;

		if (this.driveMode == DriveMode.Inverted) {
			double temp = targetLeft;
			targetLeft = -targetRight;
			targetRight = -temp;
		}

		this.leftMaster.set(ControlMode.Velocity, targetLeft);
		this.rightMaster.set(ControlMode.Velocity, targetRight);
	}

	public void voltage(double left, double right) {
		leftMaster.setVoltage(left * 12.0);
		rightMaster.setVoltage(right * 12.0);
		leftSlave.set(ControlMode.Follower, Constants.LEFT_MASTER_MOTOR);
		rightSlave.set(ControlMode.Follower, Constants.RIGHT_MASTER_MOTOR);
	}

	public void percent(double left, double right) {
		this.leftMaster.set(ControlMode.PercentOutput, left);
		this.rightMaster.set(ControlMode.PercentOutput, right);
		this.leftSlave.set(ControlMode.Follower, Constants.LEFT_MASTER_MOTOR);
		this.rightSlave.set(ControlMode.Follower, Constants.RIGHT_MASTER_MOTOR);
	}

	public double getAngle() {
		double angle = Math.abs(navX.getAngle());
		System.out.println(angle);
		return angle;
	}

	public void resetAngle() {
		navX.reset();
	}

	public void turn(double speed, Direction direction) {
		switch (direction) {
		case LEFT:
			this.voltage(speed, -speed);
			break;
		case RIGHT:
			this.voltage(-speed, speed);
			break;
		default:
			this.voltage(0, 0);
		}
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		// Convert the measured rate of velocity to meters per second.
		//
		// The function 'getSelectedSensorVelocity' returns values that are raw
		// units (pulses or ticks) per 100ms. We need to convert this value into
		// meters per second.
		//
		// First we must convert the raw units in to meters. This is done by
		// multiplying measured value by the value of METERS_PER_TICK. The result
		// of this will be the measured value in meters per 100ms. Since 1 s =
		// 1000ms, we know that we need to multiply the value by 10, which will give
		// us the meters per second value.
		double left = this.leftMaster.getSelectedSensorVelocity() * METERS_PER_TICK * 1000;
		double right = this.rightMaster.getSelectedSensorVelocity() * METERS_PER_TICK * 1000;
		return new DifferentialDriveWheelSpeeds(left, right);
	}

	@Override
	public void periodic() {
		
	}
}
