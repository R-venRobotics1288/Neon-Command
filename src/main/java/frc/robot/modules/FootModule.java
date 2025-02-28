package frc.robot.modules;

import static frc.robot.Constants.FootConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

// TODO: Add a limit to foot
public class FootModule extends SubsystemBase {
    private final SparkMax footMotor;

    public FootModule() {
        footMotor = new SparkMax(MOTOR_CANID, MotorType.kBrushless);
        footMotor.configure(Configs.FootModuleConfig.footConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setMotorState(double state) {
        footMotor.set(state);
    }
}
