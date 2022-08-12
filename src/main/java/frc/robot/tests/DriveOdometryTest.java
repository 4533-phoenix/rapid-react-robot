package frc.robot.tests;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.util.sendable.Sendable.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.tests.Test;
import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.AutoCommands;
import frc.robot.Robot;
import frc.robot.Constants;

/**
 * The class for testing drive odometry in Test mode.
 */
public class DriveOdometryTest extends Test {
    /**
     * The Shuffleboard tab that the odometry
     * testing will be displayed on.
     */
    private ShuffleboardTab tab;

    /**
     * The Shuffleboard list that the odometry
     * testing will be displayed in.
     */
    private ShuffleboardLayout layout;

    /**
     * The robot position for odometry testing.
     */
    private NetworkTableEntry robotPosition;

    /**
     * The robot angle for odometry testing.
     */
    private NetworkTableEntry robotAngle;

    /**
     * The target position for odometry testing.
     */
    private NetworkTableEntry targetPosition;

    /**
     * The value for whether or not odometry testing is enabled.
     */
    private NetworkTableEntry enableOdometry;

    /**
     * Stores the drive to position command 
     * ({@link AutoCommands#driveToPosAutoCommand(double, double)}).
     */
    private Command command;

    /**
     * The constructor for our drive odometry test.
     */
    public DriveOdometryTest() {
        this.tab = Shuffleboard.getTab("Test");
        this.layout = tab.getLayout("Drive Odometry Testing").withPosition(2, 0).withSize(2, 4);

        Pose2d robotPos = Robot.drive.getRobotPos();

        this.robotPosition = layout.add("Robot Pos", new double[]{robotPos.getX(), robotPos.getY()}).getEntry();
        this.robotAngle = layout.add("Robot Angle", Robot.drive.getRobotAngle()).getEntry();
        this.targetPosition = layout.add("Target Pos", new double[]{0.0, 0.0}).getEntry();

        this.enableOdometry = layout.add("Enable Odometry Test", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }
    
    /**
     * This method is run every loop of the Test periodic function in the 
     * {@link Robot Robot class}.
     * <p>
     * It updates the current robot position and angle, it
     * checks what the current target position is, and it 
     * schedules for the robot to drive to that target position 
     * if a command for the robot to drive to that target position 
     * hasn't already been scheduled.
     */
    @Override
    public void run() {
        // Checks if odometry testing is enabled.
        if (this.enableOdometry.getBoolean(false)) {
            // Gets the current robot position.
            Pose2d robotPos = Robot.drive.getRobotPos();

            // Sets the current robot position in the Shuffleboard.
            this.robotPosition.setDoubleArray(new double[]{robotPos.getX(), robotPos.getY()});

            // Sets the current robot angle in the Shuffleboard.
            this.robotAngle.setDouble(Robot.drive.getRobotAngle());

            // Gets the current target coords from the Shuffleboard.
            double targetX = targetPosition.getDoubleArray(new double[]{0.0, 0.0})[0];
            double targetY = targetPosition.getDoubleArray(new double[]{0.0, 0.0})[1];

            // Gets the command to run for the test.
            this.command = AutoCommands.driveToPosAutoCommand(targetX, targetY);

            // Schedules the command if not already scheduled.
            if (!CommandScheduler.getInstance().isScheduled(this.command)) {
                CommandScheduler.getInstance().schedule(this.command);
            }
        }
    }
}
