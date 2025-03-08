package frc.robot.modules;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.utilities.IntakeState;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.EnumSet;
import java.util.Set;

public class IntakeModule extends SubsystemBase {
    private SparkFlex pivotMotor;
    private RelativeEncoder pivotEncoder;

    private SparkMax leftOpenerMotor;
    private SparkMax rightOpenerMotor;
    private RelativeEncoder openerEncoder;

    private SparkFlex intakeMotor;
    private SparkFlex leftFeederMotor;
    private SparkFlex rightFeederMotor;
    
    private final Set<IntakeState> state = EnumSet.of(IntakeState.CLOSED, IntakeState.UP);

    /**
     * Initializes the {@link IntakeModule}, containing six motors as well as internal state. Intake operates as a state machine with inputs changing outputs.
     */
    public IntakeModule() {
        pivotMotor = new SparkFlex(PIVOT_MOTOR_CANID, MotorType.kBrushless);
        pivotMotor.configure(Configs.IntakeConfig.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotEncoder = pivotMotor.getEncoder();

        leftOpenerMotor = new SparkMax(LEFT_OPENER_MOTOR_CANID, MotorType.kBrushless);
        leftOpenerMotor.configure(Configs.IntakeConfig.openerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightOpenerMotor = new SparkMax(RIGHT_OPENER_MOTOR_CANID, MotorType.kBrushless);
        rightOpenerMotor.configure(Configs.IntakeConfig.openerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        openerEncoder = rightOpenerMotor.getEncoder();

        intakeMotor = new SparkFlex(INTAKE_MOTOR_CANID, MotorType.kBrushless);
        intakeMotor.configure(Configs.IntakeConfig.wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftFeederMotor = new SparkFlex(LEFT_FEEDER_MOTOR_CANID, MotorType.kBrushless);
        leftFeederMotor.configure(Configs.IntakeConfig.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFeederMotor = new SparkFlex(RIGHT_FEEDER_MOTOR_CANID, MotorType.kBrushless);
        rightFeederMotor.configure(Configs.IntakeConfig.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Gets the current pivot encoder position.
     * @return current position of the pivot encoder.
     */
    public double getPivotEncoderPosition() {
        return pivotEncoder.getPosition();
    }

    /**
     * Gets the current opener encoder position.
     * @return current position of the opener encoder.
     */
    public double getOpenerEncoderPosition() {
        return openerEncoder.getPosition();
    }

    /**
     * Gets the current feeder encoder velocity.
     * @return current velocity of the feeder encoder.
     */
    public double getFeederEncoderVelocity() {
        return rightFeederMotor.getEncoder().getVelocity();
    }

    /**
     * Gets the current intake encoder velocity.
     * @return current velocity of the intake encoder.
     */
    public double getIntakeEncoderVelocity() {
        return intakeMotor.getEncoder().getVelocity();
    }

    /**
     * Changes the state of the Intake module.
     * @param newState new current state
     */
    public void setIntakeState(IntakeState newState) {
        switch (newState) {
            case UP:
                state.remove(IntakeState.DOWN);
                state.add(IntakeState.UP);
                break;
            case DOWN:
                state.remove(IntakeState.UP);
                state.add(IntakeState.DOWN);
                break;
            case OPEN:
                state.remove(IntakeState.CLOSED);
                state.add(IntakeState.OPEN);
                break;
            case CLOSED:
                state.remove(IntakeState.OPEN);
                state.add(IntakeState.CLOSED);
                break;
        }
    }

    /**
     * Checks the current state of the Intake against any potential state.
     * @param stateToCheck the potential intake state to check
     * @return true if stateToCheck is currently true
     */
    public boolean hasIntakeState(IntakeState stateToCheck) {
        for (IntakeState i : state) {
            if (i == stateToCheck) {
                return true;
            }
        }
        return false;
    }

    public boolean isDown() {
        return hasIntakeState(IntakeState.DOWN);
    }

    public boolean isClosed() {
        return hasIntakeState(IntakeState.CLOSED);
    }

    /**
     * Sets the power of the pivot motor, responsible for moving the Intake up and down.
     * @param power the power to command
     */
    public void setPivotMotorState(double power) {
        pivotMotor.set(MathUtil.clamp(power, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    /**
     * Sets the power of the opener motors, responsible for opening or closing the jaw or the Intake.
     * @param power the power to command
     */
    public void setOpenerMotorState(double power) {
        leftOpenerMotor.set(MathUtil.clamp(power, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
        rightOpenerMotor.set(MathUtil.clamp(-power, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    /**
     * Sets the power of the feeder motors, responsible for feeding coral from intake to foot.
     * @param power the power to command
     */
    public void setFeederMotorState(double power) {
        leftFeederMotor.set(MathUtil.clamp(power, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
        rightFeederMotor.set(MathUtil.clamp(-power, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    /**
     * Sets the power of the intake motor, responsible for initial pickup of game pieces from the ground.
     * @param power the power to command
     */
    public void setIntakeMotorState(double power) {
        intakeMotor.set(MathUtil.clamp(power, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }
}
