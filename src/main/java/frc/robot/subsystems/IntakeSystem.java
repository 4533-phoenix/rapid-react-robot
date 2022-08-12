package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The class for the intake system.
 */
public class IntakeSystem extends SubsystemBase {
  private final double INTAKE_MOTOR_PERCENT = 0.65;
  private final double INTAKE_LIFT_MOTOR_PERCENT = 0.4;

	private WPI_TalonSRX intakeMotor;
	private CANSparkMax intakeLiftMotor;

	/**
	 * Constructor for the intake system.
	 */
	public IntakeSystem() {
		this.intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
		this.intakeLiftMotor = new CANSparkMax(Constants.INTAKE_LIFT_MOTOR, MotorType.kBrushless);

		this.intakeMotor.setNeutralMode(NeutralMode.Brake);
		this.intakeLiftMotor.setIdleMode(IdleMode.kBrake);

		this.intakeMotor.setInverted(true);
	}

	/**
	 * Sets the intake motor to intake balls.
	 */
	public void intakeIn() {
		this.intakeMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_PERCENT);
	}

	/**
	 * Sets the intake motor to outtake balls.
	 */
	public void intakeOut() {
		this.intakeMotor.set(ControlMode.PercentOutput, -INTAKE_MOTOR_PERCENT);
	}

	/**
	 * Stops the intake motor from running.
	 */
	public void intakeStop(){
		this.intakeMotor.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * Sets the intake lift motor to lift the intake up.
	 */
	public void intakeLiftUp() {
		this.intakeLiftMotor.set(INTAKE_LIFT_MOTOR_PERCENT);
	}

	/**
	 * Sets the intake lift motor to lower the intake down.
	 */
	public void intakeLiftDown() {
		this.intakeLiftMotor.set(-INTAKE_LIFT_MOTOR_PERCENT);
	}

	/**
	 * Stops the intake lift motor from running.
	 */
	public void intakeLiftStop() {
		this.intakeLiftMotor.set(0);
	}

	/**
	 * Runs the code within it every 20ms.
	 */
    @Override
    public void periodic() {
    }

	/**
	 * Runs the code within it every 20ms only during simulation mode.
	 */
	@Override
	public void simulationPeriodic() {
	}
}
