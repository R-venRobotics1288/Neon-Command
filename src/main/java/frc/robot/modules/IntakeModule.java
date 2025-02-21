package frc.robot.modules;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.utilities.IntakeState;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeModule extends SubsystemBase {

    private SparkFlex pivotMotor;
    private SparkFlex frontWheelsMotor;
    private SparkFlex backWheelsMotor;

    private RelativeEncoder pivotEncoder;

    private IntakeState currentState;

    public IntakeModule() {
        pivotMotor = new SparkFlex(PIVOT_MOTOR_CANID, MotorType.kBrushless);
        pivotMotor.configure(Configs.IntakeConfig.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotEncoder = pivotMotor.getEncoder();
    }

    public double getPivotEncoderPosition() {
        return pivotEncoder.getPosition();
    }

    /**
     * Changes the state of the Intake module.
     * @param state new current state
     */
    public void setIntakeState(IntakeState state) {
        currentState = state;
    }

    /**
     * Gets the current state of the intake module.
     * @return the current {@link IntakeState}
     */
    public IntakeState getIntakeState() {
        return currentState;
    }

    public void setPivotMotorState(double state) {
        pivotMotor.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    public void setFrontWheelsMotorState(double state) {
        frontWheelsMotor.set(state);
    }

    public void setBackWheelsMotorState(double state) {
        backWheelsMotor.set(state);
    }
}
