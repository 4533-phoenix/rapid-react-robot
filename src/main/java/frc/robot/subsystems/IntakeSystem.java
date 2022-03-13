package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;

public class IntakeSystem extends SubsystemBase {
  private final double INTAKE_MOTOR_PERCENT = 0.65;

  private WPI_TalonSRX intakeMotor;

  public IntakeSystem() {
    this.intakeMotor = new WPI_TalonSRX(MotorConstants.INTAKE_MOTOR);

    this.intakeMotor.setNeutralMode(NeutralMode.Brake);

    this.intakeMotor.setInverted(true);
  }

  public void intakeIn() {
    this.intakeMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_PERCENT);
  }

  public void intakeOut() {
    this.intakeMotor.set(ControlMode.PercentOutput, -INTAKE_MOTOR_PERCENT);
  }

  public void intakeStop() {
    this.intakeMotor.set(ControlMode.PercentOutput, 0);
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
