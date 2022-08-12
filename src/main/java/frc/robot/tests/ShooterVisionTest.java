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
 * The class for testing the shooter vision in Test mode.
 */
public class ShooterVisionTest extends Test {
    /**
     * The Shuffleboard tab that the vision
     * testing will be displayed on.
     */
    private ShuffleboardTab tab;

    /**
     * The Shuffleboard list that the vision
     * testing will be displayed in.
     */
    private ShuffleboardLayout layout;

    /**
     * The value for whether or not vision
     * testing is enabled.
     */
    private NetworkTableEntry enableVision;

    /**
     * Stores the old auto flywheel position command
     * ({@link ShooterCommands#oldAutoFlywheelPos()}).
     */
    private Command command;

    /**
     * The constructor for our shooter vision test.
     */
    public ShooterVisionTest() {
        this.tab = Shuffleboard.getTab("Test");
        this.layout = tab.getLayout("Shooter Vision Testing").withPosition(6, 0).withSize(2, 2);

        this.enableVision = layout.add("Test/Enable Vision Test", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }
    
    /**
     * This method is run every loop of the Test periodic function in the
     * {@link Robot Robot class}.
     * <p>
     * If vision testing is enabled via {@link #enableVision}, then the
     * automatic vision aiming command will be scheduled if not already
     * scheduled.
     */
    @Override
    public void run() {
        // Checks if vision testing is enabled.
        if (this.enableVision.getBoolean(false)) {
            this.command = ShooterCommands.oldAutoFlywheelPos();

            if (!CommandScheduler.getInstance().isScheduled((this.command))) {
                CommandScheduler.getInstance().schedule(this.command);
            }
        }
    }
}
