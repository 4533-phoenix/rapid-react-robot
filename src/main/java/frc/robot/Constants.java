package frc.robot;

public final class Constants {
    public static final int RIGHT_MASTER_MOTOR = 1;
	public static final int LEFT_MASTER_MOTOR = 3;
	public static final int RIGHT_SLAVE_MOTOR = 2;
	public static final int LEFT_SLAVE_MOTOR = 4;

	public static final int DRIVER_JOYSTICK_LEFT = 0;
	public static final int DRIVER_JOYSTICK_RIGHT = 1;
	public static final int SECOND_DRIVER_JOYSTICK = 2;

	public static final int PDH_CHANNEL = 5;

	public static final int INTAKE_MOTOR = 11;
	public static final int FLYWHEEL_MOTOR_RIGHT = 14;
	public static final int FLYWHEEL_MOTOR_LEFT = 10;
	public static final int TURRET_WHEEL_MOTOR = 6;
	public static final int TURRET_SWIVEL_MOTOR = 12;
	//TODO Winch needs new motor id
	public static final int WINCH_MOTOR = 7;
	public static final int ELEVATOR_MOTOR = 9;
	public static final int HOOD_MOTOR = 8;

	// logitech controller buttons and ports
	public static final int DRIVER_CONTROLLER = 0;
	public static final int SECOND_DRIVER_CONTROLLER = 1;

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

	// the logitech controller's triggers are analog axis, not digital
	public static final int LEFT_STICK_AXIS = 1;
	public static final int RIGHT_STICK_AXIS = 5;
	public static final int LEFT_TRIGGER_AXIS = 2;
	public static final int RIGHT_TRIGGER_AXIS = 3;

	// SlotIDs for PID control
	public static final int POSITION_SLOT_ID = 0;
	public static final int VELOCITY_SLOT_ID = 1;
}
