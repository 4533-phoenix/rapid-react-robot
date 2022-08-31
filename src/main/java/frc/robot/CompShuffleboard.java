package frc.robot;

import java.security.KeyStore.Entry;
import java.text.FieldPosition;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.MjpegServer;
 

public class CompShuffleboard {
    ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

    //Shuffleboard Elements
    private ComplexWidget field;
    private NetworkTableEntry gyroEntry;


    //Cameras
    private CameraServer cameraTest;

    //Shuffleboard Elements being constructed.
    public CompShuffleboard() {
        this.compTab = Shuffleboard.getTab("Competition");
        this.field = compTab.add("Field", Robot.drive.getField()).withWidget(BuiltInWidgets.kField);
        this.gyroEntry = compTab.add("Gyro", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
    }

    public void periodic() {
        double gyro = Robot.drive.getRobotAngle();
        gyroEntry.setDouble(gyro);

        //Cameras for Shuffleboard
        public static MjpegServer addServer​(cameraTest,1181);
    }
}