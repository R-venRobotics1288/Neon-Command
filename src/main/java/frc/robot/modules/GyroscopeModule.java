package frc.robot.modules;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ModuleConstants.PIGEON_IMU_CAN_ID;;

/**
 * Implements the Pigeon IMU.
 * @author Joel Machens
 * @version 0.1.0
 * @since 18-JAN-2025
 */
public class GyroscopeModule extends SubsystemBase {
    private PigeonIMU pigeon;

    public GyroscopeModule() {
        pigeon = new PigeonIMU(PIGEON_IMU_CAN_ID);
        pigeon.setYaw(0); // zeros initial gyrometer yaw reading
        System.out.println("INFO: PositionModule: Initialization Complete");
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
    } 

    /**
     * Gets the current yaw reported by the Pigeon IMU.
     * @return Current yaw as a double.
     */
    public double getGyroscopeYawDegrees() {
        return pigeon.getYaw();
    }

    /**
     * Gets the current yaw reported by the Pigeon IMU.
     * @return Current yaw as a double.
     */
    public double getGyroscopeYawRadians() {
        return Math.toRadians(pigeon.getYaw());
    }

    /**
     * Gets the current uptime of the Pigeon IMU in seconds.
     * @return Current uptime as an int (backed uint8_t).
     */
    public double getGyroscopeUptime() {
        return pigeon.getUpTime();
    }

    /**
     * Gets the direct IMU interface.
     * WARNING: Do not use this unless you know what you're doing!
     * @return The {@link PigeonIMU} device interface.
     */
    public PigeonIMU getDirectIMU() {
        return pigeon;
    }

    /**
     * Resets the Gyroscope heading to 0 degrees
     */
    public void resetGyroscope() {
        pigeon.setYaw(0);
    }

    /**
     * Resets the Gyroscope heading to angleDeg degrees.
     * @param angleDeg New reference heading, in degrees.
     */
    public void resetGyroscope(double angleDeg) {
        pigeon.setYaw(angleDeg);
    }
}
