package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;

public class MidClimbSystem extends SubsystemBase {
  private CANSparkMax climbMotor;

  public static final double CLIMB_MOTOR_PERCENT = 0.5;

  public MidClimbSystem() {
    this.climbMotor =
      new CANSparkMax(MotorConstants.ELEVATOR_MOTOR, MotorType.kBrushless);
  }

  public void climberUp() {
    climbMotor.set(CLIMB_MOTOR_PERCENT);
  }

  public void climberDown() {
    climbMotor.set(-CLIMB_MOTOR_PERCENT);
  }

  public void climberStop() {
    climbMotor.set(0.0);
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
