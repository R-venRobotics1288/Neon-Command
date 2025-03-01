package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.FootConstants;
import static frc.robot.Constants.IntakeConstants;
import static frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.WHEEL_DIAMETER_METRES * Math.PI
                    / ModuleConstants.DRIVING_MOTOR_REDUCTION;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_RPS;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake);
            // .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(false)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class IntakeConfig {
        public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();
        public static final SparkFlexConfig wheelsConfig = new SparkFlexConfig();
        public static final SparkMaxConfig openerConfig = new SparkMaxConfig();
        public static final SparkFlexConfig feederConfig = new SparkFlexConfig();

        static {
                pivotConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80);
                pivotConfig.encoder.positionConversionFactor((2 * Math.PI)/IntakeConstants.PIVOT_GEAR_FACTOR); // Axle Radians

                openerConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80);
                openerConfig.encoder.positionConversionFactor((2 * Math.PI)/IntakeConstants.OPENER_GEAR_FACTOR); // Axle Radians

                wheelsConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80);
                wheelsConfig.encoder.velocityConversionFactor(1 / 60); // Rotations Per Second

                feederConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(80);
                feederConfig.encoder.velocityConversionFactor(1 / 60); // Rotations Per Second
        }
    }

    public static final class ElevatorModuleConfig {
        public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();

        static {
            elevatorConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
        }
    }

    public static final class LegModuleConfig {
        public static final SparkMaxConfig legConfig = new SparkMaxConfig();

        static {
            legConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
        }
    }

    public static final class FootModuleConfig {
        public static final SparkMaxConfig footConfig = new SparkMaxConfig();

        static {
            footConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(80).encoder
                    .velocityConversionFactor(1 / (60.0 * FootConstants.FOOT_GEAR_FACTOR));
        }
    }
}