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

public class ElevatorModule extends SubsystemBase {

    private RelativeEncoder elevatorEncoder;
    private SparkFlex elevatorMotor;
    
    public ElevatorModule() {
        elevatorMotor = new SparkFlex(MOTOR_CANID, MotorType.kBrushless);
        elevatorMotor.configure(Configs.ElevatorModuleConfig.elevatorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
        elevatorEncoder = elevatorMotor.getEncoder();
        reset();
    }

    public void reset() {
        setMotorState(0);
        elevatorEncoder.setPosition(0);
    }

    public Command resetCommand() {
        return this.runOnce(
            () -> {
                reset();
            });
    }

    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }

    public void setMotorState(double state) {
        elevatorMotor.set(MathUtil.clamp(state, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
    }
}