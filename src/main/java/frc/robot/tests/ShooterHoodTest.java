package frc.robot.tests;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.util.sendable.Sendable.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.tests.Test;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.commands.ShooterCommands;
import frc.robot.Robot;
import frc.robot.Constants;

/**
 * The class for testing the shooter hood in Test mode.
 */
public class ShooterHoodTest extends Test {
    /**
     * The Shuffleboard tab that the hood 
     * testing will be displayed on.
     */
    private ShuffleboardTab tab;

    /**
     * The Shuffleboard list that the hood 
     * testing will be displayed in.
     */
    private ShuffleboardLayout layout;
    
    /**
     * The hood angle that the hood is 
     * theoretically set to (degrees).
     */
    private NetworkTableEntry setHoodAngle;

    /**
     * The hood angle that the hood is
     * actually set to (degrees).
     */
    private NetworkTableEntry actualHoodAngle;

    /**
     * The current servo offset (degrees).
     */
    private NetworkTableEntry currServoOffset;

    /**
     * The value for whether or not hood
     * testing is enabled.
     */
    private NetworkTableEntry enableHood;

    /**
     * Stores the set hood command
     * ({@link ShooterCommands#setHoodAngleCommand(double)}).
     */
    private Command command;
    
    /**
     * The constructor for our shooter hood test.
     */
    public ShooterHoodTest() {
        this.tab = Shuffleboard.getTab("Test");
        this.layout = tab.getLayout("Shooter Hood Testing", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4);

        /*
         * Set the commanded hood angle to the current commanded
         * hood angle by extracting said commanded hood angle
         * from the servo offset via subtracting the actual
         * hood angle from the servo offset.
         * 
         * This is done so that when the test is started, the
         * hood doesn't move immediately to another angle
         * other than the one it's already at.
         */
        this.setHoodAngle = layout.add("Commanded Hood Angle", Robot.shooter.getServoOffset() - Robot.shooter.getHoodAngle()).getEntry();
        this.actualHoodAngle = layout.add("Actual Hood Angle", Robot.shooter.getHoodAngle()).getEntry();
        this.currServoOffset = layout.add("Current Servo Offset", Robot.shooter.getServoOffset()).getEntry();

        this.enableHood = layout.add("Enable Hood Test", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }
    
    /**
     * This method is run every loop of the Test periodic function in the
     * {@link Robot Robot class}.
     * <p>
     * It sets the current set servo offset, it theoretically sets the 
     * hood to the set hood angle, and it sets the actual hood angle
     * to the actual hood angle the hood motor was set to.
     */
    @Override
    public void run() {
        // Checks if hood testing is enabled.
        if (this.enableHood.getBoolean(false)) {
            /* 
             * Sets the servo offset to the servo offset 
             * in the Shuffleboard.
             */
            Robot.shooter.setServoOffset(this.currServoOffset.getDouble(90.0));

            /* 
             * Sets the hood angle to the commanded hood 
             * angle in the Shuffleboard.
             * 
             * Use Robot.shooter.getServoOffset(), as servo 
             * offset might be different in Shuffleboard.
             */
            this.command = ShooterCommands.setHoodAngleCommand(setHoodAngle.getDouble(Robot.shooter.getServoOffset()));

            /*
             * Schedules the set hood angle command if the 
             * command is not already scheduled.
             */
            if (!CommandScheduler.getInstance().isScheduled(this.command)) {
                CommandScheduler.getInstance().schedule(this.command);
            }

            /* 
             * Sets the actual hood angle in the Shuffleboard 
             * to the actual hood angle commanded.
             */
            this.actualHoodAngle.forceSetDouble(Robot.shooter.getHoodAngle());
        }
        // If not enabled, cancel the current test.
        else {
            CommandScheduler.getInstance().cancel(this.command);
        }
    }
}
