package frc.robot.constants;

public class ShooterConstants {
    public static final double RPM_TO_HOOD_RATIO = 0.25;
    public static final double FLYWHEEL_MOTOR_PERCENT = 0.90;
    public static final double FLYWHEEL_INTAKE_MOTOR_PERCENT = 0.25;
    public static final double HOOD_MOTOR_PERCENT = 0.05;
    public static final double DEGREES_PER_TICK = 360 / EncoderConstants.TICKS_PER_ROTATION;
    public static final double HOOD_DEGREES_PER_TICK = DEGREES_PER_TICK / 80;
    public static final double HOOD_DEGREES_PER_ROTATION = HOOD_DEGREES_PER_TICK * EncoderConstants.TICKS_PER_ROTATION;
}
