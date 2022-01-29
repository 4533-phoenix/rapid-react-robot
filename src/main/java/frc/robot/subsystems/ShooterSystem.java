package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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

    private WPI_VictorSPX flywheelIntakeMotor;

    private static final double FLYWHEEL_MOTOR_PERCENT = 0.75;
    private static final double FLYWHEEL_INTAKE_MOTOR_PERCENT = 0.5;

    private AHRS shooterNAVX;
    
    private NetworkTable WARCam;

    PhotonCamera visionCam;

    private double yaw;
    private double pitch;
    private double skew;
    private double area;

    Transform2d pos;
    List<TargetCorner> corners;

    public ShooterSystem() {
        leftFlywheelMotor = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_LEFT);
        rightFlywheelMotor = new WPI_TalonFX(Constants.FLYWHEEL_MOTOR_RIGHT);

        flywheelIntakeMotor = new WPI_VictorSPX(Constants.TURRET_WHEEL_MOTOR);

        leftFlywheelMotor.setNeutralMode(NeutralMode.Brake);
        rightFlywheelMotor.setNeutralMode(NeutralMode.Brake);

        flywheelIntakeMotor.setNeutralMode(NeutralMode.Brake);

        leftFlywheelMotor.setInverted(true);

        shooterNAVX = new AHRS(SPI.Port.kMXP);

        WARCam = NetworkTableInstance.getDefault().getTable("hi");
        visionCam = new PhotonCamera("WARCam");

        yaw = 0.0;
        pitch = 0.0;
        skew = 0.0;

        pos = new Transform2d();
        corners = new ArrayList<TargetCorner>();
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

    public void resetPosition() {
        leftFlywheelMotor.setSelectedSensorPosition(0);
        rightFlywheelMotor.setSelectedSensorPosition(0);

        flywheelIntakeMotor.setSelectedSensorPosition(0);
    }

    public void setFlywheelReset() {
        Robot.drive.resetPosition();
    }

    public void setFlywheelPos() {
        if (yaw >= 1) {
            Robot.drive.percent(0.5, -0.5);
        }
        else if (yaw <= -1) {
            Robot.drive.percent(-0.5, 0.5);
        }
        else {
            Robot.drive.percent(0.0, 0.0);
        }
    }

    public void stopFlywheelPos() {
        Robot.drive.percent(0.0, 0.0);
    }

    public boolean flywheelReachedPosition() {
        return yaw <= 1 && yaw >= -1;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = visionCam.getLatestResult();

        PhotonTrackedTarget target = null;

        if (result.hasTargets()) {
            target = result.getBestTarget();
        }
        
        yaw = target.getYaw();
        pitch = target.getPitch();
        skew = target.getSkew();
        area = target.getArea();

        pos = target.getCameraToTarget();
        corners = target.getCorners();
    }
}
