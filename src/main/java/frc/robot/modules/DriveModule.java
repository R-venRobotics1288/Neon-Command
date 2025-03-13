// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import static frc.robot.Constants.DriveConstants.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class DriveModule extends SubsystemBase {
    private GyroscopeModule m_gyro;
    private double driveCoefficient = 1;
    private final Alert fieldRelativeAlert = new Alert("Field relative drive active.", AlertType.kInfo);

    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        FRONT_LEFT_DRIVE_MOTOR_CAN_ID,
        FRONT_LEFT_TURN_MOTOR_CAN_ID,
        FRONT_LEFT_TURN_ENCODER_CAN_ID,
        FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
    );

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        FRONT_RIGHT_DRIVE_MOTOR_CAN_ID,
        FRONT_RIGHT_TURN_MOTOR_CAN_ID,
        FRONT_RIGHT_TURN_ENCODER_CAN_ID,
        FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
    );

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        REAR_LEFT_DRIVE_MOTOR_CAN_ID,
        REAR_LEFT_TURN_MOTOR_CAN_ID,
        REAR_LEFT_TURN_ENCODER_CAN_ID,
        REAR_LEFT_CHASSIS_ANGULAR_OFFSET
    );

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
        REAR_RIGHT_DRIVE_MOTOR_CAN_ID,
        REAR_RIGHT_TURN_MOTOR_CAN_ID,
        REAR_RIGHT_TURN_ENCODER_CAN_ID,
        REAR_RIGHT_CHASSIS_ANGULAR_OFFSET
    );

    /** Creates a new DriveSubsystem. */
    public DriveModule(GyroscopeModule m_gyro) { this.m_gyro = m_gyro; }

    /**
     * Gets the current module positions as an array.
     * @return Array of {@link SwerveModulePosition}
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
    }

    /**
     * Gets the current chassis speed of the robot.
     * @return {@link ChassisSpeeds} chassis speed.
     */
    @Logged
    public ChassisSpeeds getChassisSpeeds() {
        return DRIVE_KINEMATICS.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        );
    }

    @Logged
    public double[] getEncoderPositions() {
        return new double[] {
            m_frontLeft.getAbsoluteEncoderRad(),
            m_frontRight.getAbsoluteEncoderRad(),
            m_rearLeft.getAbsoluteEncoderRad(),
            m_rearRight.getAbsoluteEncoderRad(),
        };
    }

    /**
     * Method to drive the robot.
     *
     * @param xSpeed                Speed of the robot in the x direction (forward).
     * @param ySpeed                Speed of the robot in the y direction (sideways).
     * @param rot                     Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                                            field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = SLEW_FILTER_X.calculate(xSpeed * MAX_ROBOT_SPEED * driveCoefficient);
        double ySpeedDelivered = SLEW_FILTER_Y.calculate(ySpeed * MAX_ROBOT_SPEED * driveCoefficient);
        double rotDelivered = ROTATION_FILTER.calculate(rot * MAX_ANGULAR_SPEED * driveCoefficient);

        ChassisSpeeds swerveChassisSpeed =
            fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered, 
                Rotation2d.fromDegrees(m_gyro.getGyroscopeYawDegrees())
              )
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
                        
        setModuleStates(swerveChassisSpeed.unaryMinus());
    }

    /**
     * Method that sets states of swervemodule from chassis speeds object.
     * @param desiredState Desired {@link ChassisSpeeds}
     */
    public void setModuleStates(ChassisSpeeds desiredState) {
        SwerveModuleState[] desiredModuleStates = DRIVE_KINEMATICS.toSwerveModuleStates(desiredState.unaryMinus());
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, MAX_ROBOT_SPEED);
        m_frontLeft.setDesiredState(desiredModuleStates[0]);
        m_frontRight.setDesiredState(desiredModuleStates[1]);
        m_rearLeft.setDesiredState(desiredModuleStates[2]);
        m_rearRight.setDesiredState(desiredModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public Command cutSpeed(boolean action) {
        return this.runOnce(() -> {
            if (action) {
                driveCoefficient = 0.5;
            } else {
                driveCoefficient = 1;
            }
        });
    }

    public Command toggleFieldRelative() {
        return this.runOnce(() -> {
            m_gyro.resetGyroscope();
            FIELD_RELATIVE_DRIVING = !FIELD_RELATIVE_DRIVING;
            fieldRelativeAlert.set(FIELD_RELATIVE_DRIVING);
        });
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }
    
    @Override
    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getTurnRate() * (GYROSCOPE_REVERSED ? -1.0 : 1.0);
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double drivingFactor = ModuleConstants.WHEEL_DIAMETER_METRES * Math.PI
			    / ModuleConstants.DRIVING_MOTOR_REDUCTION;
        double[] values = new double[4];
        values[0] = m_frontLeft.getPosition().distanceMeters / drivingFactor;
        values[1] = m_frontRight.getPosition().distanceMeters / drivingFactor;
        values[2] = m_rearLeft.getPosition().distanceMeters / drivingFactor;
        values[3] = m_rearRight.getPosition().distanceMeters / drivingFactor;
        return values;
    }
    
    public static Command wheelRadiusCharacterization(DriveModule drive, GyroscopeModule gyro) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> {
                    limiter.reset(0.0);
                }),
                Commands.run(() -> {
                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                    drive.setModuleStates(new ChassisSpeeds(0.0, 0.0, speed));
                }, drive)
            ),
            Commands.sequence(
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = Rotation2d.fromRadians(gyro.getGyroscopeYawRadians());
                    state.gyroDelta = 0.0;
                }),
                Commands.run(() -> {
                    var rotation = Rotation2d.fromRadians(gyro.getGyroscopeYawRadians());
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                })
                .finallyDo(() -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * DriveConstants.WHEEL_BASE) / wheelDelta;
                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                })
            )
        );
    }

}