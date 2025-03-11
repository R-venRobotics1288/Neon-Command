// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 * </p>
 * 
 * @author WPILib and Team 1288
 * @version 1.0.1
 * @since 14-JAN-2025
 */
public final class Constants {
    public static final class DriveConstants {
        public static boolean SHUFFLE_MANAGER_ENABLED = true;
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds, in metres per second.
        public static double MAX_ROBOT_SPEED = 4; // 15

        public static double MAX_ROBOT_ROTATIONS_PER_SECOND = 1;
        public static double MAX_ANGULAR_SPEED = 2 * Math.PI * MAX_ROBOT_ROTATIONS_PER_SECOND; // radians per second

        public static double BASE_SLEW_RATE = 15;
        public static SlewRateLimiter SLEW_FILTER_X = new SlewRateLimiter(BASE_SLEW_RATE);
        public static SlewRateLimiter SLEW_FILTER_Y = new SlewRateLimiter(BASE_SLEW_RATE);

        public static double ROT_SLEW_RATE = 15;
        public static SlewRateLimiter ROTATION_FILTER = new SlewRateLimiter(ROT_SLEW_RATE);

        public static boolean FIELD_RELATIVE_DRIVING = true;

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(22.5);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        );

        // Angular offsets of the modules relative to the chassis in radians
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI;

        // P, I, D coefficients
        public static final double TRANSLATION_COEFFICIENT_P = 0.25;
        public static final double TRANSLATION_COEFFICIENT_I = 0;
        public static final double TRANSLATION_COEFFICIENT_D = 0;
        public static final double ROTATION_COEFFICIENT_P = 0.6;
        public static final double ROTATION_COEFFICIENT_I = 0;
        public static final double ROTATION_COEFFICIENT_D = 0;

        // SPARK MAX CAN IDs
        public static final int  FRONT_LEFT_DRIVE_MOTOR_CAN_ID = 1;
        public static final int   REAR_LEFT_DRIVE_MOTOR_CAN_ID = 7;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = 8;
        public static final int  REAR_RIGHT_DRIVE_MOTOR_CAN_ID = 3;

        public static final int  FRONT_LEFT_TURN_MOTOR_CAN_ID = 10;
        public static final int   REAR_LEFT_TURN_MOTOR_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURN_MOTOR_CAN_ID = 2;
        public static final int  REAR_RIGHT_TURN_MOTOR_CAN_ID = 9;

        public static final int  FRONT_LEFT_TURN_ENCODER_CAN_ID = 20;
        public static final int FRONT_RIGHT_TURN_ENCODER_CAN_ID = 21;
        public static final int   REAR_LEFT_TURN_ENCODER_CAN_ID = 22;
        public static final int  REAR_RIGHT_TURN_ENCODER_CAN_ID = 23;

        // Don't set this to true without making it work first
        public static final boolean GYROSCOPE_REVERSED = false;

        public static final double WHEEL_RADIUS_RAMP_RATE = 0.05;
        public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
    }

    public static final class ModuleConstants {
        /**
         * The MAXSwerve module can be configured with one of three pinion gears: 12T,
         * 13T, or 14T. This changes the drive speed of the module (a pinion gear with
         * more teeth will result in a robot that drives faster).
         */        
        public static final int DRIVING_MOTOR_PINION_TEETH = 14;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METRES = 0.09;
        public static final double WHEEL_CIRCUMFERENCE_METRES = WHEEL_DIAMETER_METRES * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = 8.14;
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METRES)
                / DRIVING_MOTOR_REDUCTION;

        // PositionModule System Bus Constants
        public static final int PIGEON_IMU_CAN_ID = 30;

        // VisionModule Constants
        public static final String LIMELIGHT_HOSTNAME = "limelight-raven";
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DRIVE_DEADBAND = 0.05;
        public static final double OPERATOR_DEADBAND = 0.15;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final int SWERVE_ALIGNMENT_BUTTON = 7;
        public static final int ENABLE_TESTING_BUTTON = 8;
    }

    public static final class AutoConstants {
        public static final double AUTO_MAX_SPEED = 2; // 10 // in metres per second
        public static final double AUTO_MAX_ACCELERATION = 5; // in metres per second squared
        public static final double AUTO_MAX_ANGULAR_SPEED = 4 * Math.PI; // in radians per second
        public static final double AUTO_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // in radians per second squared

        public static final double PX_CONTROLLER = 1;
        public static final double PY_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        // P, I, D coefficients
        public static final double TRANSLATION_COEFFICIENT_P = 4.0;
        public static final double TRANSLATION_COEFFICIENT_I = 0;
        public static final double TRANSLATION_COEFFICIENT_D = 0;
        public static final double ROTATION_COEFFICIENT_P = 6.0;
        public static final double ROTATION_COEFFICIENT_I = 0;
        public static final double ROTATION_COEFFICIENT_D = 0;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                AUTO_MAX_ANGULAR_SPEED, AUTO_MAX_ANGULAR_ACCELERATION);

        private static final RobotConfig GET_PATHPLANNER_ROBOT_CONFIG() {
            try {
                return RobotConfig.fromGUISettings();
            } catch (Exception e) {
                e.printStackTrace();
            }
            return null;
        }
        public static final RobotConfig PATHPLANNER_ROBOT_CONFIG = GET_PATHPLANNER_ROBOT_CONFIG();
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }
    
    public static final class IntakeConstants {
        public static final double POSITION_TOLERANCE = 0.05;
        public static final double VELOCITY_TOLERANCE = 0.1;
        public static final double MAX_MOTOR_SPEED = 0.2;
        public static final double INTAKE_SPEED_RPS = Math.PI; // rotations per second target of wheel and feeders

        public static final int PIVOT_MOTOR_CAN_ID = 0; // TODO

        public static final int INTAKE_MOTOR_CAN_ID = 0; // TODO

        public static final int LEFT_OPENER_MOTOR_CAN_ID = 0; // TODO
        public static final int RIGHT_OPENER_MOTOR_CAN_ID = 5;

        public static final int LEFT_FEEDER_MOTOR_CAN_ID = 0; // TODO
        public static final int RIGHT_FEEDER_MOTOR_CAN_ID = 0; // TODO


        public static final double PIVOT_GEAR_FACTOR = 16;
        public static final double PIVOT_DEGREE_UP = -119;
        public static final double PIVOT_DEGREE_DOWN = 0;
        public static final double PIVOT_PID_P = 0.4;
        public static final double PIVOT_PID_I = 0.0;
        public static final double PIVOT_PID_D = 0.0;
        
        public static final double INTAKE_PID_P = 0.4;
        public static final double INTAKE_PID_I = 0.0;
        public static final double INTAKE_PID_D = 0.0;
        
        public static final double OPENER_DEGREE_OPEN = 50;
        public static final double OPENER_DEGREE_CLOSE = 0;
        public static final double OPENER_GEAR_FACTOR = 60;
        public static final double OPENER_PID_P = 0.4;
        public static final double OPENER_PID_I = 0.0;
        public static final double OPENER_PID_D = 0.0;

        public static final double FEEDER_PID_P = 0.4;
        public static final double FEEDER_PID_I = 0.0;
        public static final double FEEDER_PID_D = 0.0;
    }

    public static final class ElevatorConstants {
        public static final int MOTOR_CAN_ID = 4;

        public static final double LEVEL_FOUR_POS = 100.0;
        public static final double ELEVATOR_SAFE_HEIGHT = 25.0;
        public static final double ELEVATOR_ZERO_POS = 0.0;
        public static final double ELEVATOR_GEAR_FACTOR = 16.0;

        public static final double MAX_MOTOR_SPEED = 0.5;

        public static final double ELEVATOR_PID_P = 0.075;
        public static final double ELEVATOR_PID_I = 0.0; //0.7
        public static final double ELEVATOR_PID_D = 0.00125;
        public static final double ELEVATOR_TOLERANCE = 0.1;
    }
    
    public static final class LegConstants {
        public static final int MOTOR_CAN_ID = 5;

        public static final double LEG_POS_INTAKING = 0;
        public static final double LEG_POS_REST = 0.2;
        public static final double LEG_POS_ONE = 0.4;
        public static final double LEG_POS_TWO = 0.6;
        public static final double LEG_POS_THREE = 0.8;
        public static final double LEG_POS_FOUR = 1.0;

        public static final double MAX_MOTOR_SPEED = 1.0;
        public static final double LEG_TOLERANCE = 0.02; // how many rotations off is considered fine for the PID controller.

        public static final double LEG_GEAR_FACTOR = 144.0;

        public static final double LEG_PID_P = 2.0; // TODO: calibrate
        public static final double LEG_PID_I = 0.0; // TODO: calibrate
        public static final double LEG_PID_D = 0.0; // TODO: calibrate
    }

    public static final class FootConstants {
        public static final int MOTOR_CAN_ID = 40;
        public static final double MAX_MOTOR_SPEED = 1;
        public static final double FOOT_SPEED_RPS = 8; // rotations per second target of foot wheels
        public static final double FOOT_TOLERANCE = 0.02;
        public static final double FOOT_GEAR_FACTOR = 4.0;

        public static final double MOTOR_PID_P = 0.04; // TODO: calibrate
        public static final double MOTOR_PID_I = 0.0; // TODO: calibrate
        public static final double MOTOR_PID_D = 0.00110; // TODO: calibrate
    }

    public static final class ClimberConstants {
        public static final int LEFT_CLIMBER_MOTOR_CAN_ID = 0; //CHANGEME
        public static final int RIGHT_CLIMBER_MOTOR_CAN_ID = 0; //CHANGEME

        public static final double MAX_MOTOR_SPEED = 1.0;

        public static final double CLIMBER_PID_P = 4.0;
        public static final double CLIMBER_PID_I = 0.0;
        public static final double CLIMBER_PID_D = 0.0;

        public static final double CLIMBER_GEAR_FACTOR = 108.0;
        
        public static final double CLIMBER_TOLERANCE = 0.01;

    }
}