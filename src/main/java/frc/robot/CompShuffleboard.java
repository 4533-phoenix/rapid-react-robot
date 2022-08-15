package frc.robot;

import java.text.FieldPosition;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.networktables.*;
 

public class CompShuffleboard {
    private ShuffleboardTab compTab;

    private ComplexWidget field;
    private NetworkTableEntry gyro; 

    public CompShuffleboard() {
        this.compTab = Shuffleboard.getTab("Competition");
        this.field = compTab.add("Field", Robot.drive.getField()).withWidget(BuiltInWidgets.kField);
        this.gyro = compTab.add("Gyro", Robot.drive.getRobotAngle()).withWidget(BuiltInWidgets.kGyro).getEntry();
    }
}