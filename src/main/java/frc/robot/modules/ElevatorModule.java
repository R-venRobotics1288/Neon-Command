package frc.robot.modules;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import frc.robot.Configs;
import frc.robot.utilities.ElevatorState;

import static frc.robot.Constants.ElevatorConstants.*;

/**
 * Implements the Elevator of the robot.
 * 
 * @author Vaibhav Mohankumar, Vrishank Vadi, & Tejas Cherukuru
 * @version 0.1.0
 * @since 22-FEB-2025
 */
public class ElevatorModule extends SubsystemBase {

    private RelativeEncoder rightElevatorEncoder;
    private SparkMax rightElevatorMotor;
    private SparkMax leftElevatorMotor;

    public ElevatorState elevatorState;

    /**
     * Initializes the Elevator of the robot.
     */
    public ElevatorModule() {
        rightElevatorMotor = new SparkMax(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        rightElevatorMotor.configure(Configs.ElevatorModuleConfig.elevatorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        rightElevatorEncoder = rightElevatorMotor.getEncoder();
        leftElevatorMotor = new SparkMax(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        leftElevatorMotor.configure(Configs.ElevatorModuleConfig.elevatorConfig, ResetMode.kResetSafeParameters, 
                PersistMode.kPersistParameters);
        elevatorState = ElevatorState.LEVEL_ZERO;
        reset();
    }

    /**
     * Resets the state of the elevator subsystem.
     */
    public void reset() {
        setRightMotorState(0);
        setLeftMotorState(0);
        elevatorState = ElevatorState.LEVEL_ZERO;
    }

    /**
     * Gets a command to reset the Elevator module.
     * 
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
     * 
     * @return current position
     */
    @Logged
    public double getEncoderPosition() {
        return -rightElevatorEncoder.getPosition();
    }

    /**
     * Sets the current state of the elevator motor.
     * 
     * @param state desired speed of the motor
     */
    public void setRightMotorState(double state) {
        rightElevatorMotor.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    public void setLeftMotorState(double state) {
        leftElevatorMotor.set(MathUtil.clamp(-state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    public void setElevatorState(ElevatorState elevatorState) {
        this.elevatorState = elevatorState;
    }

    @Logged
    public ElevatorState getElevatorState() {
        return this.elevatorState;
    }

    public Command manualElevatorCommand(double power) {
        return new StartEndCommand(
            () -> { 
                setRightMotorState(power); 
                setLeftMotorState(power);
            },
            () -> { 
                setRightMotorState(0); 
                setLeftMotorState(0);
            }, 
            this
        );
    }
}