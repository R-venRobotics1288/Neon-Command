package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ModuleConstants;

/**
 * Implements Limelight to detect AprilTags and map them to field layout.
 * Currently only used for dead reckoning.
 * @author Joel Machens
 * @version 0.1.0
 * @since 17-JAN-2025
 */
public class VisionModule extends SubsystemBase {
    // VisionModule Initialization
    public VisionModule() {}

    /**
     * Gets the current pose of the robot, as estimated by vision alone.
     * May be very accurate, if multiple AprilTag targets are visible,
     * or garbage, if none are visible. Prefer {@link PositionModule}'s estimated
     * position.
     * @return Current visual robot position as a {@link Pose3d}.
     */
    public Pose3d getVisionPose() {
        return LimelightHelpers.getBotPose3d(ModuleConstants.kLimelightName);
    }

    /**
     * Gets the confidence of the vision position estimate. If this function returns 0,
     * do not trust or use vision position estimates. If this function returns 2 or above,
     * it has a higher confidence than gyro or odometer.
     * @return Current confidence level of vision estimates as an int.
     */
    public int getVisionConfidence() {
        return LimelightHelpers.getTargetCount(ModuleConstants.kLimelightName);
    }

    @Override
    public void periodic() {
        // FIXME: Test PositionModule before starting auto.
        SmartDashboard.putNumber(
            "Limelight-TargetCount",
            LimelightHelpers.getTargetCount(ModuleConstants.kLimelightName)
        );
        Field2d field = new Field2d();
        Pose3d position = getVisionPose();
        field.setRobotPose(position.getX(), position.getY(), new Rotation2d(0, 0));
        SmartDashboard.putData("Limelight-EstimatedPose", field);
    }
}
