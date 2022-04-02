package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class IntakeSystem extends SubsystemBase {
  private final double INTAKE_MOTOR_PERCENT = 0.65;
  private final double INTAKE_LIFT_MOTOR_PERCENT = 0.4;

	private WPI_TalonSRX intakeMotor;
	private CANSparkMax intakeLiftMotor;

	public IntakeSystem() {
		this.intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
		this.intakeLiftMotor = new CANSparkMax(Constants.INTAKE_LIFT_MOTOR, MotorType.kBrushless);

		this.intakeMotor.setNeutralMode(NeutralMode.Brake);
		this.intakeLiftMotor.setIdleMode(IdleMode.kBrake);

		this.intakeMotor.setInverted(true);
  }

  public void intakeIn() {
		this.intakeMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_PERCENT);
	}

	public void intakeOut() {
		this.intakeMotor.set(ControlMode.PercentOutput, -INTAKE_MOTOR_PERCENT);
	}

	public void intakeStop(){
		this.intakeMotor.set(ControlMode.PercentOutput, 0);
	}

	public void intakeLiftUp() {
		this.intakeLiftMotor.set(INTAKE_LIFT_MOTOR_PERCENT);
	}

	public void intakeLiftDown() {
		this.intakeLiftMotor.set(-INTAKE_LIFT_MOTOR_PERCENT);
	}

	public void intakeLiftStop() {
		this.intakeLiftMotor.set(0);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
