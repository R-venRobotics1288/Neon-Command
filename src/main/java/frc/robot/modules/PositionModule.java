package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Utilises AprilTag pose estimation, Odometry, and the Pigeon IMU to report a best-guess robot pose.
 * @author Team 1288
 * @version 0.1.1
 * @since 17-JAN-2025
 */
public class PositionModule extends SubsystemBase {
    private Pose3d robotPosition;
    private DriveModule driveModule;
    private VisionModule visionModule;
    private GyroscopeModule gyroscopeModule;
    private Field2d dashboardField;

    // PositionModule Initialization
    public PositionModule(DriveModule driveModule, VisionModule visionModule, GyroscopeModule gyroscopeModule) {
        this.driveModule = driveModule;
        this.visionModule = visionModule;
        this.gyroscopeModule = gyroscopeModule;
        this.robotPosition = new Pose3d();
        
        dashboardField = new Field2d();
        SmartDashboard.putData("Estimated Robot Position", dashboardField);
    }

    /**
     * Gets the current best-estimate position of the robot, relative to the robot's origin.
     * @return Current robot position as a {@link Pose3d}.
     */
    public Pose3d getRobotPosition() {
        return robotPosition;
    }

    /**
     * Sets current position and rotation to (0, 0, 0), with rotational heading of 0 degrees.
     */
    public void setPositionOriginToCurrentPosition() {
        gyroscopeModule.resetGyroscope();
        robotPosition = new Pose3d();
        System.out.println("INFO: PositionModule: Reset Gyroscope and Position Estimate! Current position is now the origin!");
    }
    
    @Override
    public void periodic() {
        double gyroYaw = gyroscopeModule.getGyroscopeYawRadians();
        Pose2d odometryPoseEstimate = driveModule.getPose();
        Pose3d visionPoseEstimate = visionModule.getVisionPose();

        int visionConfidence = visionModule.getVisionConfidence();
        if (visionConfidence == 1) { // Moderate Vision Accuracy - blend it with other sensors and report if any disagreements arise
            double degreeError = Math.toDegrees(Math.abs(gyroYaw - visionPoseEstimate.getRotation().getZ()));
            if (degreeError > 10) {
                System.out.println("WARNING: PositionModule: " + degreeError + " degrees of heading error! Gyroscope reported heading: "
                                   + gyroscopeModule.getGyroscopeYawDegrees() + " while Vision reported heading: " + visionPoseEstimate.getRotation().getZ());
            }

            Pose2d positionErrorPose = odometryPoseEstimate.relativeTo(visionPoseEstimate.toPose2d());
            double positionError = Math.sqrt(Math.pow(positionErrorPose.getX(), 2) + Math.pow(positionErrorPose.getY(), 2));
            if (positionError > 0.5) {
                System.out.println("WARNING: PositionModule: " + positionError + " metres of position error! Odometry reported field position  ("
                                   + odometryPoseEstimate.getX() + ", " + odometryPoseEstimate.getY() + "), while Vision reported field position ("
                                   + visionPoseEstimate.getX() + ", " + visionPoseEstimate.getY() + ")");
            }

            /**
             * Current Fusion Weights
             * 40% Vision 60% Gyroscope
             * 60% Vision 40% Odometry
             */
            double weightedYaw = ((gyroYaw * 6) + (visionPoseEstimate.getRotation().getZ() * 4)) / 10;
            double weightedX = ((odometryPoseEstimate.getX() * 4) + (visionPoseEstimate.getX() * 6)) / 10;
            double weightedY = ((odometryPoseEstimate.getY() * 4) + (visionPoseEstimate.getY() * 6)) / 10;
            robotPosition = new Pose3d(
                weightedX,
                weightedY,
                visionPoseEstimate.getZ(),
                new Rotation3d(
                    visionPoseEstimate.getRotation().getX(),
                    visionPoseEstimate.getRotation().getY(),
                    weightedYaw
                )
            );
        } else if (visionConfidence > 1) { // High Vision Accuracy - use it as a source of truth for other sensors position detection
            gyroscopeModule.resetGyroscope(Math.toDegrees(visionPoseEstimate.getRotation().getZ()));
            driveModule.resetOdometry(
                new Pose2d(visionPoseEstimate.getX(),
                visionPoseEstimate.getY(),
                visionPoseEstimate.getRotation().toRotation2d())
            );
            robotPosition = visionPoseEstimate;
        } else { // Vision Unavailable - use exculsively odometry and gyro until we get more apriltags available
            robotPosition = new Pose3d(
                odometryPoseEstimate.getX(),
                odometryPoseEstimate.getY(),
                0, new Rotation3d(0, 0, gyroYaw)
            );
        }

        dashboardField.setRobotPose(robotPosition.getX(), robotPosition.getY(), new Rotation2d(robotPosition.getRotation().getZ()));
    }
}
