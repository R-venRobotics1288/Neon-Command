package frc.robot.modules;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import static frc.robot.Constants.LegConstants.*;

/**
 * Implements the Leg of the robot.
 * @author Nirmaha Mukherjee and Vaibhav Mohankumar
 * @version 0.1.0
 * @since 13-FEB-2025
 */
public class LegModule extends SubsystemBase {

    private SparkMax legMotor;
    private AbsoluteEncoder legEncoder;

    // LegModule
    public LegModule() {
        legMotor = new SparkMax(MOTOR_CANID, MotorType.kBrushless);
        legMotor.configure(Configs.LegModuleConfig.legConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
        legEncoder = legMotor.getAbsoluteEncoder();
    }

    /**
     * Gets the position of the leg motor encoder.
     * @return encoder position, in rotations of the motor
     */
    public double getEncoderPosition() {
        return legEncoder.getPosition();
    }

    /**
     * Sets the desired state of the leg motor.
     * @param state desired speed of the leg motor, normalized from -1.0 to 1.0
     */
    public void setMotorState(double state) {
        legMotor.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }
}
