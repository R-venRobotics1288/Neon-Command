package frc.robot.modules;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.utilities.IntakeState;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.EnumSet;
import java.util.Set;

public class IntakeModule extends SubsystemBase {

    private SparkFlex pivotMotor;
    private SparkFlex frontWheelsMotor;
    private SparkFlex backWheelsMotor;
    private SparkMax intakeOpenerLeftMotor;
    private SparkMax intakeOpenerRightMotor;
    private SparkFlex leftFeederMotor;
    private SparkFlex rightFeederMotor;

    private RelativeEncoder pivotEncoder;
    private RelativeEncoder openerEncoder;

    private boolean intakeAlgae = false;

    private final Set<IntakeState> state = EnumSet.of(IntakeState.CLOSED, IntakeState.UP);
    private final PIDController feederPIDController = new PIDController(FEEDER_PID_P, FEEDER_PID_I, FEEDER_PID_D);
    private final PIDController intakePIDController = new PIDController(INTAKE_PID_P, INTAKE_PID_I, INTAKE_PID_D);

    public IntakeModule() {
        // pivotMotor = new SparkFlex(PIVOT_MOTOR_CANID, MotorType.kBrushless);
        // pivotMotor.configure(Configs.IntakeConfig.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // pivotEncoder = pivotMotor.getEncoder();

        // frontWheelsMotor = new SparkFlex(FRONT_WHEELS_MOTOR_CANID, MotorType.kBrushless);
        // frontWheelsMotor.configure(Configs.IntakeConfig.wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // backWheelsMotor = new SparkFlex(BACK_WHEELS_MOTOR_CANID, MotorType.kBrushless);
        // backWheelsMotor.configure(Configs.IntakeConfig.wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        intakeOpenerLeftMotor = new SparkMax(INTAKE_OPENER_LEFT_MOTOR_CANID, MotorType.kBrushless);
        intakeOpenerLeftMotor.configure(Configs.IntakeConfig.openerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeOpenerRightMotor = new SparkMax(INTAKE_OPENER_RIGHT_MOTOR_CANID, MotorType.kBrushless);
        intakeOpenerRightMotor.configure(Configs.IntakeConfig.openerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        openerEncoder = intakeOpenerRightMotor.getEncoder();

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
     * Gets the current state of the intake module.
     * @return set of current {@link IntakeState}
     */
    public Set<IntakeState> getIntakeStates() {
        return state;
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

    public void setPivotMotorState(double state) {
        pivotMotor.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    public void setOpenerMotorState(double state) {
        intakeOpenerLeftMotor.set(MathUtil.clamp(-state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
        intakeOpenerRightMotor.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    public void setMotorState(double feederPower, double intakePower) {
        leftFeederMotor.set(MathUtil.clamp(-feederPower, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
        rightFeederMotor.set(MathUtil.clamp(feederPower, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
        frontWheelsMotor.set(MathUtil.clamp(intakePower, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
        backWheelsMotor.set(MathUtil.clamp(intakePower, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }

    /**
     * If the intake is down, runs motors when toIntake is true.
     * @param toIntake Sets the current state of the intake and feeder motors.
     */
    public Command intakeAlgae(boolean toIntake) {
        return this.runOnce(() -> {
            intakeAlgae = toIntake;
        });
    }

    @Override
    public void periodic() {
        if (state.contains(IntakeState.DOWN) && (state.contains(IntakeState.CLOSED) || intakeAlgae)) {
            double feederPower = feederPIDController.calculate(rightFeederMotor.getEncoder().getVelocity(), INTAKE_SPEED_RPS);
            double intakePower = intakePIDController.calculate(frontWheelsMotor.getEncoder().getVelocity(), INTAKE_SPEED_RPS);
            setMotorState(feederPower, intakePower);
        } else {
            setMotorState(0, 0);
        }
    }
}
