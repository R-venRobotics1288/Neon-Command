package frc.robot.modules;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import static frc.robot.Constants.ArmConstants.*;

/**
 * Implements the Arm of the robot.
 * @author Nirmaha Mukherjee and Vaibhav Mohankumar
 * @version 0.1.0
 * @since 13-FEB-2025
 */
public class ArmModule extends SubsystemBase {

    private SparkMax armMotor;
    private AbsoluteEncoder armEncoder;

    // ArmModule
    public ArmModule() {
        armMotor = new SparkMax(MOTOR_CANID, MotorType.kBrushless);
        armMotor.configure(Configs.ArmModuleConfig.armConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
        armEncoder = armMotor.getAbsoluteEncoder();
    }

    /**
     * Gets the position of the arm motor encoder.
     * @return encoder position, in rotations of the motor
     */
    public double getEncoderPosition() {
        return armEncoder.getPosition();
    }

    /**
     * Sets the desired state of the arm motor.
     * @param state desired speed of the arm motor, normalized from -1.0 to 1.0
     */
    public void setMotorState(double state) {
        armMotor.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }
}
