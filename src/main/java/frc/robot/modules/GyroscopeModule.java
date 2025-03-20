package frc.robot.modules;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ModuleConstants.*;

/**
 * Implements the Pigeon IMU.
 * @author NULL
 * @version 0.1.0
 * @since 18-JAN-2025
 */
public class GyroscopeModule extends SubsystemBase {
    private final Pigeon2 pigeon = new Pigeon2(PIGEON_IMU_CAN_ID, "rio");
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    //private final Alert gyroStateAlert = new Alert("Pigeon IMU State NOT OK!", AlertType.kError);

    public GyroscopeModule() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0);
        yaw.setUpdateFrequency(80);
        yawVelocity.setUpdateFrequency(40);
        pigeon.optimizeBusUtilization();
        DataLogManager.log("GyroscopeModule: Initialization Complete");
    }

    @Override
    public void periodic() {
        //StatusSignal<Integer> status = pigeon.getFaultField();
        //if (status.getStatus() != StatusCode.OK) {
        //    gyroStateAlert.set(true);
        //    gyroStateAlert.setText("Pigeon IMU State NOT OK, instead reported " + status.getStatus().name());
        //} else {
        //    gyroStateAlert.set(false);
        //}
    } 

    /**
     * Gets the current yaw reported by the Pigeon IMU.
     * @return Current yaw as a double.
     */
    @Logged
    public double getGyroscopeYawDegrees() {
        return yaw.getValueAsDouble();
    }

    @Logged
    public double getTurnRate() {
        return yawVelocity.getValueAsDouble();
    }

    /**
     * Gets the current yaw reported by the Pigeon IMU.
     * @return Current yaw as a double.
     */
    @Logged
    public double getGyroscopeYawRadians() {
        return Math.toRadians(yaw.getValueAsDouble());
    }

    /**
     * Gets the current uptime of the Pigeon IMU in seconds.
     * @return Current uptime as an double.
     */
    public double getGyroscopeUptime() {
        return pigeon.getUpTime().getValueAsDouble();
    }

    /**
     * Gets the direct IMU interface.
     * WARNING: Do not use this unless you know what you're doing!
     * @return The {@link Pigeon2} device interface.
     */
    public Pigeon2 getDirectIMU() {
        return pigeon;
    }

    /**
     * Resets the Gyroscope heading to 0 degrees
     */
    public void resetGyroscope() {
        pigeon.getConfigurator().setYaw(0);
    }

    /**
     * Resets the Gyroscope heading to angleDeg degrees.
     * @param angleDeg New reference heading, in degrees.
     */
    public void resetGyroscope(double angleDeg) {
        pigeon.getConfigurator().setYaw(angleDeg);
    }
}