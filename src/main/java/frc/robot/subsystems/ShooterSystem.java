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
public class ShooterSystem extends SubsystemBase {
    private WPI_TalonFX leftFlywheelMotor;
    private WPI_TalonFX rightFlywheelMotor;

    private WPI_TalonSRX flywheelIntakeMotor;
    private Servo hoodMotor;

    private double initialAngle;
    private double targetAngle;
    private Direction targetDirection;

    // Motor Constants
    private static final double FLYWHEEL_MOTOR_PERCENT = 0.6;
    private static final double FLYWHEEL_INTAKE_MOTOR_PERCENT = 0.40;
    private static final double HOOD_MOTOR_PERCENT = 0.05;
    private static final double DEGREES_PER_TICK = 360/DriveSystem.TICKS_PER_ROTATION;
    private static final double HOOD_DEGREES_PER_TICK = DEGREES_PER_TICK/80;
    private static final double HOOD_DEGREES_PER_ROTATION = HOOD_DEGREES_PER_TICK * DriveSystem.TICKS_PER_ROTATION;
    private static final double SERVO_OFFSET = 128.7;

    // Projectile Constants and Variables
    private static final double MAX_FLYWHEEL_RPM = 3065.0; // TODO: Change this to the right value (RPM)
    private static final double FLYWHEEL_RADIUS = 1.0 / 6.0; // TODO: Change this to the right value (feet)
    private static final double TIME_TO_SHOOT = 2; // seconds 
    private static final double GOAL_HEIGHT = 8.67; // feet
    private static final double CAMERA_HEIGHT = 24.8 / 12.0; // TODO: Change this to the right value (feet)
    private static final double CAMERA_MOUNTING_ANGLE = 37.0; // TODO: Change this to the right value (degrees)
    private static double cameraTargetAngle = 0.0;
    private static double horizontalDistance = 0.0;
    private static final double GRAVITY_ACCELERATION = -32.1741; // feet/s^2
    private static final double INITIAL_Y_VELOCITY = (GOAL_HEIGHT - CAMERA_HEIGHT) - (0.5 * GRAVITY_ACCELERATION * (TIME_TO_SHOOT * TIME_TO_SHOOT)); // feet/s
    private static double initialXVelocity = 0.0;
    private static double initialVelocity = 0.0;
    private static double rotationalVelocity = 0.0;
    private static double shootHoodAngle = 0.0;
    private static double shootHoodPercent = 0.0;

    private AHRS shooterNAVX;

    public double hoodAngle;
    
    private double currHoodAngle;

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

        hoodMotor = new Servo(9);

        hoodMotor.setAngle(SERVO_OFFSET);

        leftFlywheelMotor.setNeutralMode(NeutralMode.Brake);
        rightFlywheelMotor.setNeutralMode(NeutralMode.Brake);

        flywheelIntakeMotor.setNeutralMode(NeutralMode.Brake);

        leftFlywheelMotor.setInverted(true);

        flywheelIntakeMotor.setInverted(true);

        shooterNAVX = new AHRS(SPI.Port.kMXP);

        hoodAngle = 0;

        currHoodAngle = 0;

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

    public void flywheelAutoShoot() {
        this.leftFlywheelMotor.set(ControlMode.PercentOutput, shootHoodPercent);
        this.rightFlywheelMotor.set(ControlMode.PercentOutput, shootHoodPercent);
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

    public double getShootHoodAngle() {
        return shootHoodAngle;
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

        double time, flywheelIntakeTime;

        if (flywheelRunning) {
            time = 1 + (2 * balls);
            flywheelIntakeTime = 0;
        }
        else {
            time = 2 + (2 * balls);
            flywheelIntakeTime = 1.5;
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

    public void setHoodAngle(double angle) {
        this.hoodMotor.setAngle(SERVO_OFFSET - angle);
    }

    public double getHoodAngle() {
        hoodAngle = this.hoodMotor.getAngle();
        return hoodAngle;
    }

    public void hoodUp() {
        setHoodAngle((SERVO_OFFSET - currHoodAngle) + 0.5);
    }

    public void hoodDown() {
        setHoodAngle((SERVO_OFFSET - currHoodAngle) - 0.5);
    }

    public void hoodStop() {
        setHoodAngle(SERVO_OFFSET - currHoodAngle);
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

        currHoodAngle = hoodMotor.getAngle();

        System.out.println("Hood Angle: " + currHoodAngle);

        // System.out.println(currHoodAngle);

        // Kinematic Equations:
        // vf = v0 + at
        // vf^2 = v0^2 + 2ad
        // d = vagt (1/2(v0 + vf)t)
        // d = v0t + 1/2at^2

        // v0x will equal our distance from the goal divided by our time to shoot, which is 2 
        // *this is technically calculating average velocity, but x velocity is constant
        // since we now know v0x, we will calculate v0y by saying that it is:
        // v0y = dy - 1/2at^2
        // now we can use v0x and v0y to calculate v and 0 (theta), which gives us 
        // the flywheel velocity and the hood angle
        // v0x will vary, however v0y will not, but this will still lead to varying v's and 0's
        cameraTargetAngle = CAMERA_MOUNTING_ANGLE + targetOffsetAngle_Vertical;
        horizontalDistance = (GOAL_HEIGHT - CAMERA_HEIGHT) / Math.tan(cameraTargetAngle * (Math.PI / 180));

        initialXVelocity = horizontalDistance / TIME_TO_SHOOT;
        initialVelocity = Math.sqrt(Math.pow(initialXVelocity, 2) + Math.pow(INITIAL_Y_VELOCITY, 2));
        rotationalVelocity = ((initialVelocity / FLYWHEEL_RADIUS) * 60) / (2 * Math.PI); /* multiplying by 60 and dividing by 2pi  
                                                                                            changes from radians per second to 
                                                                                            revolutions per minute (RPM) */
        
        shootHoodAngle = Math.atan(INITIAL_Y_VELOCITY / initialXVelocity) * (180 / Math.PI);
        shootHoodPercent = rotationalVelocity * (1 / MAX_FLYWHEEL_RPM);

        initialAngle = getHoodAngle();

        if (!limitSwitch.get()) {
            // hoodMotor.setPosition(0.0);
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
