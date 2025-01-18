package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose3d;
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

    @Override
    public void periodic() {
        // FIXME: Test, and integrate into PositionModule before starting auto. First step: creating a test .fmap "fieldmap" file and uploading it to limelight with webui
        SmartDashboard.putNumber(
            "Limelight-TargetCount",
            LimelightHelpers.getTargetCount(ModuleConstants.kLimelightName)
        );
    }
}
