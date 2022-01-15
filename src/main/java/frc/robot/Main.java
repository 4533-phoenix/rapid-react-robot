package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// Don't change any of this
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
