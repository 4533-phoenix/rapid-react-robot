package frc.robot.constants;

public class DriveConstants {
  public static final double MAX_VELOCITY = 4000;
  public static final double TURBO_VELOCITY = 4500;
  public static final double QUARTER_VELOCITY = 1000;
  public static final double PEAK_OUTPUT = 1.0;

  // Wheel specific constants.
  public static final double WHEEL_DIAMETER = 6.0;
  public static final double WHEEL_DIAMETER_M = 0.1524;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  public static final double WHEEL_CIRCUMFERENCE_M =
    WHEEL_DIAMETER_M * Math.PI;
  public static final double TICKS_PER_INCH =
  EncoderConstants.TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
  public static final double TICKS_PER_METER =
  EncoderConstants.TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_M;
  public static final double METERS_PER_TICK =
    WHEEL_CIRCUMFERENCE_M / EncoderConstants.TICKS_PER_ROTATION;

  // Unit specific constants.
  public static final double INCHES_PER_METER = 39.3701;
  public static final double METERS_PER_INCH = 1 / 39.3701;

  // Velocity PIDF values
  public static final double VELOCITY_P = 0.0003; // 0.3 / 1000 RPM Error
  public static final double VELOCITY_I = 0.0;
  public static final double VELOCITY_D = 0.003; // will tune
  public static final double VELOCITY_I_ZONE = 0.0;
  public static final double VELOCITY_FEED_FORWARD = 1.25E-4; // 0.5 / MAX_VELOCITY

  // Position PIDF values
  public static final double POSITION_P = 0.006;
  public static final double POSITION_I = 0.0;
  public static final double POSITION_D = 0.06; // 0.016677 will tune
  public static final double POSITION_I_ZONE = 0.0;
  public static final double POSITION_FEED_FORWARD = 0.0;

  // Feed Forward Gains
  // kS - the voltage needed to overcome the motor's static friction (V).
  // kV - the voltage needed to maintain a given constant velocity (V * s/m).
  // kA - the voltage needed to induce a given acceleration (V * s^2/m).
  public static final double FEED_FORWARD_KS = 0.16203;
  public static final double FEED_FORWARD_KV = 0.13098;
  public static final double FEED_FORWARD_KA = 0.026747;
}
