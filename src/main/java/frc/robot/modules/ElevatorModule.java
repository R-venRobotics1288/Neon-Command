package frc.robot.modules;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import frc.robot.Configs;

import static frc.robot.Constants.ElevatorConstants.*;

/**
 * Implements the Elevator of the robot.
 * @author Vaibhav Mohankumar, Vrishank Vadi, & Tejas Cherukuru
 * @version 0.1.0
 * @since 22-FEB-2025
 */
public class ElevatorModule extends SubsystemBase {

    private RelativeEncoder elevatorEncoder;
    private SparkFlex elevatorMotor;
    
    /**
     * Initializes the Elevator of the robot.
     */
    public ElevatorModule() {
        elevatorMotor = new SparkFlex(MOTOR_CANID, MotorType.kBrushless);
        elevatorMotor.configure(Configs.ElevatorModuleConfig.elevatorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
        elevatorEncoder = elevatorMotor.getEncoder();
        reset();
    }

    /**
     * Resets the state of the elevator subsystem.
     */
    public void reset() {
        setMotorState(0);
        elevatorEncoder.setPosition(0);
    }

    /**
     * Gets a command to reset the Elevator module.
     * @return the reset command
     */
    public Command resetCommand() {
        return this.runOnce(
            () -> {
                reset();
            });
    }

    /**
     * Gets the current position of the elevator encoder.
     * @return current position
     */
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }

    /**
     * Sets the current state of the elevator motor.
     * @param state desired speed of the motor
     */
    public void setMotorState(double state) {
        elevatorMotor.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }
}