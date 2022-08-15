package frc.robot.tests;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.*;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.networktables.*;

import frc.robot.tests.Test;
import frc.robot.subsystems.DriveSystem;
import frc.robot.Robot;
import frc.robot.Constants;

/**
 * The class for testing drive PID in Test mode.
 */
public class DrivePIDTest extends Test {
    /**
     * The Shuffleboard tab that the PID testing will 
     * be displayed on.
     */
    private ShuffleboardTab tab;

    /**
     * The Shuffleboard list that the PID testing will 
     * be displayed in.
     */
    private ShuffleboardLayout layout;

    /**
     * The p value for PID testing.
     */
    private NetworkTableEntry pValue;
    
    /**
     * The i value for PID testing.
     */
    private NetworkTableEntry iValue;

    /**
     * The d value for PID testing.
     */
    private NetworkTableEntry dValue;

    /**
     * The f value for PID testing.
     */
    private NetworkTableEntry fValue;

    /**
     * The setpoint value for PID testing.
     */
    private NetworkTableEntry setpoint;

    /**
     * The actual value of the robot (Position/Velocity).
     */
    private NetworkTableEntry actualValue;
    
    /**
     * The value for whether or not PID testing is enabled.
     */
    private NetworkTableEntry enablePID;

    /**
     * The value for auto or teleop drive mode. 
     * False for auto, true for teleop.
     */
    private NetworkTableEntry driveMode;

    /**
     * The previously set drive mode for PID testing.
     */
    private DriveSystem.Mode prevMode;

    /**
     * The currently set drive mode for PID testing.
     * <p>
     * Compared with {@link #prevMode} to check whether
     * or not the mode has been toggled.
     */
    private DriveSystem.Mode currMode;

    /**
     * The constructor for our drive PID test.
     * 
     * @param mode The initial drive mode that will be PID tested.
     */
    public DrivePIDTest(DriveSystem.Mode mode) {
        this.tab = Shuffleboard.getTab("Test");
        this.layout = tab.getLayout("Drive PID Testing", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);

        this.pValue = layout.add("P", 0.0).getEntry();
        this.iValue = layout.add("I", 0.0).getEntry();
        this.dValue = layout.add("D", 0.0).getEntry();
        this.fValue = layout.add("F", 0.0).getEntry();
        this.setpoint = layout.add("Setpoint", 0.0).getEntry();

        this.enablePID = layout.add("Enable PID Test", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

        if (mode == DriveSystem.Mode.AUTONOMOUS) {
            this.actualValue = layout.add("Actual Value", Robot.drive.getPosition()).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible time", 15)).getEntry();

            this.driveMode = layout.add("Autonomous <---> Teleop", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        }
        else if (mode == DriveSystem.Mode.TELEOP) {
            this.actualValue = layout.add("Actual Value", Robot.drive.getVelocity()).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible time", 15)).getEntry();

            this.driveMode = layout.add("Autonomous <---> Teleop", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        }

        this.prevMode = mode;
        this.currMode = mode;

        this.setPIDFMode(mode);
    }

    /**
     * Sets the PID values for the PID controllers
     * and the Shuffleboard values to presets based
     * on the passed in drive mode.
     * 
     * @param mode The drive mode.
     */
    private void setPIDFMode(DriveSystem.Mode mode) {
        Robot.drive.setPIDF(Robot.drive.getLeftPIDCont(), mode);
        Robot.drive.setPIDF(Robot.drive.getRightPIDCont(), mode);

        this.pValue.setDouble(Robot.drive.getLeftPIDCont().getP());
        this.iValue.setDouble(Robot.drive.getLeftPIDCont().getI());
        this.dValue.setDouble(Robot.drive.getLeftPIDCont().getD());
        this.fValue.setDouble(Robot.drive.getLeftPIDCont().getFF());
    }

    /**
     * Sets the PID values for the PID controllers and sets them to the slot 
     * at the passed in slot ID.
     * <p>
     * Used mainly to set the PID values entered on the Shuffleboard, which is 
     * why the Shuffleboard PID values are not set like in {@link #setPIDFMode}.
     * 
     * @param p The p value to set.
     * @param i The i value to set.
     * @param d The d value to set.
     * @param iz The i zone value to set.
     * @param ff The feed forward value to set.
     * @param slotID The slot to set the values to.
     */
    private void setPIDFVals(double p, double i, double d, double iz, double ff, int slotID) {
        Robot.drive.setPIDF(Robot.drive.getLeftPIDCont(), p, i, d, iz, ff, slotID);
        Robot.drive.setPIDF(Robot.drive.getRightPIDCont(), p, i, d, iz, ff, slotID);
    } 
    
    /**
     * This method is run every loop of the Test periodic function in the 
     * {@link Robot Robot class}.
     * <p>
     * It checks whether or not PID testing is enabled, what the current 
     * drive mode is, whether or not the drive mode to PID test has been 
     * switched, and if PID testing has been enabled, it starts PID testing 
     * for the specified drive mode at the specified setpoint.
     */
    @Override
    public void run() {
        // Checks if PID testing is enabled.
        if (this.enablePID.getBoolean(false)) {
            // Sets the current drive mode based on the selected drive mode.
            if (!this.driveMode.getBoolean(false)) {
                currMode = DriveSystem.Mode.AUTONOMOUS;
            }
            else {
                currMode = DriveSystem.Mode.TELEOP;
            }

            /*
             * Sets PID preset of current drive mode 
             * if drive modes have just been switched.
             */
            if (prevMode != currMode) {
                setPIDFMode(currMode);

                prevMode = currMode;
            }

            // Starts PID testing for the selected drive mode.
            if (currMode == DriveSystem.Mode.AUTONOMOUS) {
                // Update current robot position (rotations).
                this.actualValue.setDouble(Robot.drive.getPosition());

                setPIDFVals(pValue.getDouble(0.0), iValue.getDouble(0.0), dValue.getDouble(0.0), 0.0, fValue.getDouble(0.0), Constants.POSITION_SLOT_ID);

                // Divide by wheel circumference (in.) to go from inches to rotations
                Robot.drive.getLeftPIDCont().setReference(setpoint.getDouble(0.0) / DriveSystem.WHEEL_CIRCUMFERENCE, ControlType.kPosition, Constants.POSITION_SLOT_ID);
                Robot.drive.getRightPIDCont().setReference(setpoint.getDouble(0.0) / DriveSystem.WHEEL_CIRCUMFERENCE, ControlType.kPosition, Constants.POSITION_SLOT_ID);
            }
            else if (currMode == DriveSystem.Mode.TELEOP) {
                // Update current robot velocity (RPM).
                this.actualValue.setDouble(Robot.drive.getVelocity());
                
                setPIDFVals(pValue.getDouble(0.0), iValue.getDouble(0.0), dValue.getDouble(0.0), 0.0, fValue.getDouble(0.0), Constants.VELOCITY_SLOT_ID);

                Robot.drive.getLeftPIDCont().setReference(setpoint.getDouble(0.0), ControlType.kVelocity, Constants.VELOCITY_SLOT_ID);
                Robot.drive.getRightPIDCont().setReference(setpoint.getDouble(0.0), ControlType.kVelocity, Constants.VELOCITY_SLOT_ID);
            }
        }
        // If not enabled, cancel the current test.
        else {
            Robot.drive.percent(0.0, 0.0);
        }
    }
}
