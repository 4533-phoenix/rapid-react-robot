package frc.robot;

public final class Constants {
	// Drive train motor IDs.
    public static final int RIGHT_MASTER_MOTOR = 1;
	public static final int LEFT_MASTER_MOTOR = 3;
	public static final int RIGHT_SLAVE_MOTOR = 2;
	public static final int LEFT_SLAVE_MOTOR = 4;

	// Non-drive-train motor IDs.
	public static final int INTAKE_MOTOR = 11;
	public static final int FLYWHEEL_MOTOR_RIGHT = 14;
	public static final int FLYWHEEL_MOTOR_LEFT = 10;
	public static final int TURRET_WHEEL_MOTOR = 6;
	public static final int TURRET_SWIVEL_MOTOR = 12;
	public static final int WINCH_MOTOR = 7;
	public static final int ELEVATOR_MOTOR = 9;
	public static final int HIGH_CLIMB_MOTOR = 15;
	public static final int INTAKE_LIFT_MOTOR = 16;
	public static final int HIGH_CLIMB_FOLLOWER = 17;

	/*
	 * Miscellaneous IDs. 
	 *
	 * Correspond to the DIO and PWM channels on the RoboRio.
	 */
	public static final int HOOD_SERVO_MOTOR = 9;
	public static final int LEFT_LIMIT_SWITCH = 1;
	public static final int RIGHT_LIMIT_SWITCH = 0;

	// Drive controller IDs.
	public static final int DRIVER_CONTROLLER = 0;
	public static final int SECOND_DRIVER_CONTROLLER = 1;

	// Drive controller button IDs.
	public static final int BUTTON_A = 1;
	public static final int BUTTON_B = 2;
	public static final int BUTTON_X = 3;
	public static final int BUTTON_Y = 4;
	public static final int BUTTON_LB = 5;
	public static final int BUTTON_RB = 6;
	public static final int BUTTON_BACK = 7;
	public static final int BUTTON_START = 8;
	public static final int LEFT_STICK_PRESS_DOWN = 9;
	public static final int RIGHT_STICK_PRESS_DOWN = 10;

	// Drive controller axis IDs.
	public static final int LEFT_STICK_AXIS = 1;
	public static final int RIGHT_STICK_AXIS = 5;
	public static final int LEFT_TRIGGER_AXIS = 2;
	public static final int RIGHT_TRIGGER_AXIS = 3;

	// Drive system PID Slot IDs.
	public static final int POSITION_SLOT_ID = 0;
	public static final int VELOCITY_SLOT_ID = 1;
}
