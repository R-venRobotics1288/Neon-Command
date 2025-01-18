package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import static frc.robot.Constants.ModuleConstants;

/**
 * Implements the Pigeon IMU and utilises AprilTag pose estimation to report a best-guess robot pose.
 * @author Joel Machens
 * @version 0.1.0
 * @since 17-JAN-2025
 */
public class PositionModule extends SubsystemBase {
    private PigeonIMU pigeon;
    private Pose3d robotPosition;

    // PositionModule Initialization
    public PositionModule() {
        pigeon = new PigeonIMU(ModuleConstants.kPigeonIMUDeviceNumber);
        pigeon.setYaw(0); // zeros initial gyrometer yaw reading
        System.out.println("INFO: PositionModule: Initialization Complete");
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
        pigeon.setYaw(0);
    }
    
    @Override
    public void periodic() {
        PigeonIMU.GeneralStatus generalStatus = new PigeonIMU.GeneralStatus();
        pigeon.getGeneralStatus(generalStatus);
        if (generalStatus.state != PigeonState.Ready) {
            System.out.println("ERROR: PositionModule: Pigeon IMU: State is NOT Ready, instead reported " + generalStatus.state.name());
        }
        if (generalStatus.lastError != ErrorCode.OK) {
            System.out.println("ERROR: PositionModule: Pigeon IMU: " + generalStatus.lastError.name());
        }

        // FIXME: Add Odometry to help with translation, and detecting gyroscope drift
        robotPosition = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, pigeon.getYaw()));
    }
}
