package frc.robot.commands;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Odometry {
    /* All distances are in meters */
    private Pose2d position;

    private Rotation2d gyroOffset;
    private Rotation2d previousAngle;

    private double prevLeftDistance;
    private double prevRightDistance;

    public Odometry(Rotation2d gyroAngle, Pose2d initialPosition) {
        position = initialPosition;
        gyroOffset = position.getRotation().minus(gyroAngle);
        previousAngle = initialPosition.getRotation();
    }

    // Overloaded constructor that gives robot position (0,0)
    public Odometry(Rotation2d gyroAngle) {
        this(gyroAngle, new Pose2d());
    }

    public void resetPosition(Pose2d pos, Rotation2d gyroAngle) {
        position = pos;
        previousAngle = pos.getRotation();
        gyroOffset = previousAngle.minus(gyroAngle);

        prevLeftDistance = 0;
        prevRightDistance = 0;
    }

    public Pose2d getPosition() {
        return position;
    }

    public Pose2d update(Rotation2d gyroAngle, double leftDist, double rightDist) {
        double deltaLeft = leftDist - prevLeftDistance;
        double deltaRight = rightDist - prevRightDistance;

        prevLeftDistance = leftDist;
        prevRightDistance = rightDist;

        double averageDeltaDist = (deltaLeft + deltaRight) / 2.0;
        var angle = gyroAngle.plus(gyroOffset);

        var newPosition = position.exp(new Twist2d(averageDeltaDist, 0.0, angle.minus(previousAngle).getRadians()));

        previousAngle = angle;

        position = new Pose2d(newPosition.getTranslation(), angle);

        return position;
    }

    public Translation2d getTransformation(Rotation2d updateAngle, double leftDist, double rightDist) {
        double deltaLeft = leftDist - prevLeftDistance;
        double deltaRight = rightDist - prevRightDistance;

        prevLeftDistance = leftDist;
        prevRightDistance = rightDist;

        double averageDeltaDist = (deltaLeft + deltaRight) / 2.0;
        var angle = updateAngle.plus(gyroOffset);

        var newPosition = position.exp(new Twist2d(averageDeltaDist, 0.0, angle.minus(previousAngle).getRadians()));

        previousAngle = angle;

        return newPosition.getTranslation();
    }
}
