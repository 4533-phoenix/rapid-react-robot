package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.*;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants;
import frc.robot.Robot;

import java.util.List;
import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

public class ShooterSystem extends SubsystemBase {
    private WPI_TalonFX leftFlywheelMotor;
    private WPI_TalonFX rightFlywheelMotor;

    private WPI_TalonSRX flywheelIntakeMotor;
    private CANSparkMax hoodMotor;
    private RelativeEncoder hoodEncoder;

    private static final double FLYWHEEL_MOTOR_PERCENT = 0.75;
    private static final double FLYWHEEL_INTAKE_MOTOR_PERCENT = 0.5;
    private static final double HOOD_MOTOR_PERCENT = 0.25;
    private static final double DEGREES_PER_TICK = 360/DriveSystem.TICKS_PER_ROTATION;
    private static final double HOOD_DEGREES_PER_TICK = DEGREES_PER_TICK/2;

    private AHRS shooterNAVX;

    public double hoodAngle;

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

        hoodMotor.setInverted(true);
        
        flywheelIntakeMotor.setInverted(true);

        shooterNAVX = new AHRS(SPI.Port.kMXP);

        hoodAngle = 0;

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
        leftFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
        rightFlywheelMotor.set(ControlMode.PercentOutput, FLYWHEEL_MOTOR_PERCENT);
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

    public void resetPosition() {
        leftFlywheelMotor.setSelectedSensorPosition(0);
        rightFlywheelMotor.setSelectedSensorPosition(0);
        flywheelIntakeMotor.setSelectedSensorPosition(0);
    }

    public void setFlywheelReset() {
        Robot.drive.resetPosition();
    }

    public double setHoodAngle(double angle) {
        double initialAngle = getHoodAngle();
        double targetAngle = angle;
        
        while (Math.abs(targetAngle - getHoodAngle()) > 1) {
            if (initialAngle > targetAngle) { 
                hoodMotor.set(HOOD_MOTOR_PERCENT);
            }
            else { 
                hoodMotor.set(-HOOD_MOTOR_PERCENT);
            }
        }
        
        hoodAngle = this.hoodEncoder.getPosition()*(HOOD_DEGREES_PER_TICK * 4096);
        return getHoodAngle();
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public void resetHoodAngle(double angle) {
        hoodAngle = angle;
        hoodEncoder.setPosition(hoodAngle/360);
    }

    public void hoodUp() {
        this.hoodMotor.set(HOOD_MOTOR_PERCENT);
    }

    public void hoodDown() {
        this.hoodMotor.set(-HOOD_MOTOR_PERCENT);
    }

    public void hoodStop() {
        this.hoodMotor.set(0.0);
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

    @Override
    public void periodic() {
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

        hoodAngle = this.hoodEncoder.getPosition()*(HOOD_DEGREES_PER_TICK * 4096);
    }
}
