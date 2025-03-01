package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LimelightHelpers;

import static frc.robot.Constants.ModuleConstants.LIMELIGHT_HOSTNAME;

/**
 * Implements Limelight to detect AprilTags and map them to field layout.
 * Currently only used for dead reckoning.
 * @author Team 1288
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
        return LimelightHelpers.getBotPose3d(LIMELIGHT_HOSTNAME);
    }

    /**
     * Gets the confidence of the vision position estimate. If this function returns 0,
     * do not trust or use vision position estimates. If this function returns 2 or above,
     * it has a higher confidence than gyro or odometer.
     * @return Current confidence level of vision estimates as an int.
     */
    public int getVisionConfidence() {
        return LimelightHelpers.getTargetCount(LIMELIGHT_HOSTNAME);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Vision Target Count",
            LimelightHelpers.getTargetCount(LIMELIGHT_HOSTNAME)
        );
    }
}
