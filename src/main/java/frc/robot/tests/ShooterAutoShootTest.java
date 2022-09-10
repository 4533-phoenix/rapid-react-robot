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
 * The class for testing shooter auto shoot in Test mode.
 */
public class ShooterAutoShootTest extends Test {
    /**
     * The Shuffleboard tab that the auto shoot
     * testing will be displayed on.
     */
    private ShuffleboardTab tab;

    /**
     * The Shuffleboard list that the auto shoot
     * testing will be displayed in.
     */
    private ShuffleboardLayout layout;

    /**
     * The percent that the robot will drive at.
     * <p>
     * The robot cannot shoot if the robot 
     * is moving.
     */
    private NetworkTableEntry drivePercent;

    /**
     * The flywheel RPM currently calculated
     * by the auto shoot math.
     */
    private NetworkTableEntry flywheelRPM;

    /**
     * The hood angle currently calculated 
     * by the auto shoot math.
     */
    private NetworkTableEntry shootHoodAngle;

    /**
     * The current trajectory constant that will
     * be used by the auto shoot math.
     */
    private NetworkTableEntry trajectoryConstant;

    /**
     * The value for whether or not auto shoot
     * testing is enabled.
     */
    private NetworkTableEntry enableAutoShoot;

    /**
     * Stores the auto shoot command
     * ({@link ShooterCommands#autoShoot(double, int, boolean)}).
     */
    private Command command;

    /**
     * The constructor for our shooter auto shoot test.
     */
    public ShooterAutoShootTest() {
        this.tab = Shuffleboard.getTab("Test");
        this.layout = tab.getLayout("Shooter Auto Shoot Testing", BuiltInLayouts.kList).withPosition(8, 0).withSize(2, 4);

        this.drivePercent = layout.add("Drive Percent Slider", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

        this.flywheelRPM = layout.add("Calculated Flywheel RPM", Robot.shooter.getFlywheelRPM()).getEntry();
        this.shootHoodAngle = layout.add("Calculated Hood Angle", Robot.shooter.getShootHoodAngle()).getEntry();
        this.trajectoryConstant = layout.add("Trajectory Constant", Robot.shooter.getTrajectoryConstant()).getEntry();

        this.enableAutoShoot = layout.add("Enable Auto Shoot Test", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

        this.command = ShooterCommands.autoShoot(Robot.shooter.getServoOffset() - Robot.shooter.getHoodAngle(), 0, false);
    }

    /**
     * This method is run every loop of the Test periodic function in the
     * {@link Robot Robot class}.
     * <p>
     * It sets the robot to drive at the specified percent, it sets
     * the flywheel RPM and hood angle currently calculated by the 
     * auto shoot math, it sets the specified trajectory constant,
     * and it schedules for the robot to auto shoot two balls if
     * the command isn't already scheduled and if the robot is
     * not driving.
     */
    @Override
    public void run() {
        // Checks if auto shoot testing is enabled.
        if (this.enableAutoShoot.getBoolean(false)) {
            /* 
             * Gets the specified drive percent and halves it
             * for slower driving.
             */
            double percent = 0.5 * this.drivePercent.getDouble(0.0);

            /* 
             * Sets the robot to drive at the calculated 
             * drive percent.
             */
            Robot.drive.percent(percent, percent);

            /*
             * Sets the flywheel RPM and hood angle to 
             * their currently calculated values from
             * the auto shoot math.
             */
            this.flywheelRPM.forceSetDouble(Robot.shooter.getFlywheelRPM());
            this.shootHoodAngle.forceSetDouble(Robot.shooter.getShootHoodAngle());

            // Sets the currently specified trajectory constant.
            Robot.shooter.setTrajectoryConstant(this.trajectoryConstant.getDouble(2.0));

            /*
             * Schedules the auto shoot command if the auto shoot
             * command is not already scheduled.
             */
            if (!this.command.isScheduled()) {
                /* 
                 * Sets the auto shoot command.
                 * 
                 * Use Robot.shooter.getShootHoodAngle(), as 
                 * the shoot hood angle might not be set in
                 * the Shuffleboard.
                 */
                this.command = ShooterCommands.autoShoot(Robot.shooter.getShootHoodAngle(), 2, false);

                this.command.schedule(false);
            }

            /*
             * If the robot is driving, then the auto shoot command
             * is canceled.
             */
            if (percent != 0.0) {
                this.command.cancel();
            }
        }
        // If not enabled, cancel the current test.
        else {
            this.command.cancel();
        }
    }
}
