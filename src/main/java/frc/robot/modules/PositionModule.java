package frc.robot.modules;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Utilises AprilTag pose estimation, Odometry, and the Pigeon IMU to report a best-guess robot pose.
 * @author Joel Machens and Sean Ryan
 * @version 0.2.0
 * @since 17-JAN-2025
 */
public class PositionModule extends SubsystemBase {
    private DriveModule driveModule;
    private VisionModule visionModule;
    private GyroscopeModule gyroscopeModule;

    private Pose2d robotPose; // pose of the robot relative to its origin
    
    private final Field2d dashboardField;
    private final SwerveDrivePoseEstimator estimator;
    

    // PositionModule Initialization
    public PositionModule(DriveModule driveModule, VisionModule visionModule, GyroscopeModule gyroscopeModule) {
        this.driveModule = driveModule;
        this.visionModule = visionModule;
        this.gyroscopeModule = gyroscopeModule;
        this.robotPose = new Pose2d();

        this.estimator = new SwerveDrivePoseEstimator(
            DRIVE_KINEMATICS,
            new Rotation2d(gyroscopeModule.getGyroscopeYawRadians()),
            driveModule.getModulePositions(),
            new Pose2d()
        ); 
        dashboardField = new Field2d();
        SmartDashboard.putData("Estimated Robot Position", dashboardField);
    }

    /**
     * Gets the current best-estimate position of the robot, relative to the robot's origin point.
     * @return Current robot position as a {@link Pose2d}.
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    /**
     * Sets current position and rotation to (0, 0, 0), with rotational heading of 0 degrees.
     */
    public void resetPosition() {
        estimator.resetPose(Pose2d.kZero);
        robotPose = new Pose2d();
        System.out.println("INFO: PositionModule: Reset relative position, current position is now the origin!");
    }
    
    @Override
    public void periodic() {
        // update odometry and gyro
        estimator.update(new Rotation2d(gyroscopeModule.getGyroscopeYawRadians()), driveModule.getModulePositions());

        // apply vision if, and only if, we're sure its constructive
        boolean rotationConfidence = driveModule.getTurnRate() < 540;
        int targetConfidence = visionModule.getVisionConfidence();
        double ta = visionModule.getVisionTargetArea();
        if (rotationConfidence && ((targetConfidence >= 2 && ta > 0.04) || (targetConfidence == 1 && ta > 0.08))) { // slow enough rotation, multiple targets, or solid target definition
            estimator.addVisionMeasurement(visionModule.getVisionPose(), Timer.getFPGATimestamp());
        }

        // grab the estimated position and put it on the map
        robotPose = estimator.getEstimatedPosition();
        dashboardField.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
    }
}
