package frc.robot.modules;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import frc.robot.Configs;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberModule extends SubsystemBase {
    private RelativeEncoder climberEncoderR;
    private RelativeEncoder climberEncoderL;
    
    private SparkMax climberMotorR;
    private SparkMax climberMotorL;
    
    /**
     * Initializes the climber of the robot.
     */
    public ClimberModule() {
        climberMotorR = new SparkMax(RIGHT_CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climberMotorR.configure(Configs.ClimberModuleConfig.climberConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
        climberEncoderR = climberMotorR.getEncoder();

        climberMotorL = new SparkMax(LEFT_CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climberMotorL.configure(Configs.ClimberModuleConfig.climberConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
        climberEncoderL = climberMotorL.getEncoder();
        reset();
    }

    /**
     * Resets the state of the climber subsystem.
     */
    public void reset() {
        setMotorStateRight(0);
        setMotorStateLeft(0);
        climberEncoderR.setPosition(0);
        climberEncoderL.setPosition(0);
    }

    /**
     * Gets a command to reset the climber module.
     * @return the reset command
     */
    public Command resetCommand() {
        return this.runOnce(
            () -> {
                reset();
            });
    }

    /**
     * Gets the current position of the climber encoder.
     * @return current position
     */
    @Logged
    public double getEncoderPositionR() {
        return climberEncoderR.getPosition();
    }
    
    @Logged
    public double getEncoderPositionL() {
        return climberEncoderL.getPosition();
    }

    /**
     * Sets the current state of the climber motor.
     * @param state desired speed of the motor
     */
    public void setMotorStateRight(double state) {
        climberMotorR.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }
    public void setMotorStateLeft(double state) {
        climberMotorL.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }
        
}