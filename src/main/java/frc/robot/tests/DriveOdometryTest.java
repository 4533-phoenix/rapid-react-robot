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
     * The robot's x position for odometry testing.
     */
    private NetworkTableEntry robotX;

    /**
     * The robot's y position for odometry testing.
     */
    private NetworkTableEntry robotY;

    /**
     * The robot angle for odometry testing.
     */
    private NetworkTableEntry robotAngle;

    /**
     * The target's x position for odometry testing.
     */
    private NetworkTableEntry targetX;

    /**
     * The target's y position for odometry testing.
     */
    private NetworkTableEntry targetY;

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
        this.layout = tab.getLayout("Drive Odometry Testing", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);

        Pose2d robotPos = Robot.drive.getRobotPos();

        this.robotX = layout.add("Robot X", robotPos.getX()).getEntry();
        this.robotY = layout.add("Robot Y", robotPos.getY()).getEntry();

        this.robotAngle = layout.add("Robot Angle", Robot.drive.getRobotAngle()).getEntry();

        /*
         * Set the initial target position of the robot to the 
         * robot's current position so that when the test is 
         * enabled, it doesn't immediately drive to another
         * location.
         */
        this.targetX = layout.add("Target X", robotPos.getX()).getEntry();
        this.targetY = layout.add("Target Y", robotPos.getY()).getEntry();

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
            this.robotX.forceSetDouble(robotPos.getX());
            this.robotY.forceSetDouble(robotPos.getY());

            // Sets the current robot angle in the Shuffleboard.
            this.robotAngle.forceSetDouble(Robot.drive.getRobotAngle());

            // Gets the current target coords from the Shuffleboard.
            double xPos = this.targetX.getDouble(robotPos.getX());
            double yPos = this.targetY.getDouble(robotPos.getY());

            // Gets the command to run for the test.
            this.command = AutoCommands.driveToPosAutoCommand(xPos, yPos);

            // Schedules the command if not already scheduled.
            if (!CommandScheduler.getInstance().isScheduled(this.command)) {
                CommandScheduler.getInstance().schedule(this.command);
            }
        }
        // If not enabled, cancel the current test.
        else {
            CommandScheduler.getInstance().cancel(this.command);
        }
    }
}
