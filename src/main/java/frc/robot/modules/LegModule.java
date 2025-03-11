package frc.robot.modules;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.utilities.LegState;

import static frc.robot.Constants.FootConstants;
import static frc.robot.Constants.LegConstants;

/**
 * Implements the Leg of the robot.
 * 
 * @author Nirmaha Mukherjee, Sean Ryan, and Vaibhav Mohankumar
 * @version 0.2.0
 * @since 13-FEB-2025
 */
public class LegModule extends SubsystemBase {

    private SparkFlex legMotor;
    private RelativeEncoder legEncoder;
    private SparkMax footMotor;
    private LegState legState;

    // LegModule
    public LegModule() {
        legMotor = new SparkFlex(LegConstants.MOTOR_CANID, MotorType.kBrushless);
        legMotor.configure(Configs.LegModuleConfig.legConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        legEncoder = legMotor.getEncoder();

        footMotor = new SparkMax(FootConstants.MOTOR_CANID, MotorType.kBrushless);
        footMotor.configure(Configs.FootModuleConfig.footConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.legState = LegState.REST;
    }

    /**
     * Gets the position of the leg motor encoder.
     * 
     * @return encoder position, in rotations of the motor
     */
    @Logged
    public double getEncoderPosition() {
        return legEncoder.getPosition();
    }

    /**
     * Sets the desired state of the leg motor.
     * 
     * @param state desired speed of the leg motor, normalized from -1.0 to 1.0
     */
    public void setMotorState(double state) {
        legMotor.set(MathUtil.clamp(state, -LegConstants.MAX_MOTOR_SPEED, LegConstants.MAX_MOTOR_SPEED));
    }

    /**
     * Sets the desired state of the foot motor.
     * 
     * @param state desired speed of the foot motor, normalized from -1.0 to 1.0
     */
    public void setFootMotorState(double state) {
        footMotor.set(MathUtil.clamp(state, -FootConstants.MAX_MOTOR_SPEED, FootConstants.MAX_MOTOR_SPEED));
    }

    /**
     * Gets the current foot encoder velocity.
     * @return current velocity of the foot encoder.
     */
    @Logged
    public double getFootEncoderVelocity() {
        return footMotor.getEncoder().getVelocity();
    }

    /**
     * Gets the current Current of the footMotor in Amps
     * @return current of the foot motor in Amps
     */
    @Logged
    public double getFootMotorOutputCurrent() {
        return footMotor.getOutputCurrent();
    }

    public void setLegState(LegState legState) {
        this.legState = legState;
    }

    @Logged
    public LegState getLegState() {
        return this.legState;
    }

    public boolean isIntakingPosition() {
        return legState == LegState.INTAKING;
    }

    public boolean isNotIntakingPosition() {
        return legState != LegState.INTAKING;
    }

    public boolean isNotInPositionOne() {
        return legState != LegState.POSITION_ONE;
    }
    
    public boolean isNotInPositionTwo() {
        return legState != LegState.POSITION_TWO;
    }

    public boolean isNotInPositionThree() {
        return legState != LegState.POSITION_THREE;
    }

    public boolean isNotInPositionFour() {
        return legState != LegState.POSITION_FOUR;
    }

    public boolean isNotAtRest() {
        return legState != LegState.REST;
    }
}
