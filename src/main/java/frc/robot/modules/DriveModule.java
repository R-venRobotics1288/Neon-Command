// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

public class DriveModule extends SubsystemBase {
    private GyroscopeModule m_gyro;
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

    // Odometry class for tracking robot pose
    private SwerveDriveOdometry m_odometry;

    /** Creates a new DriveSubsystem. */
    public DriveModule(GyroscopeModule m_gyro) {
        this.m_gyro = m_gyro;
        this.m_odometry = new SwerveDriveOdometry(
            DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(m_gyro.getGyroscopeYawDegrees()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            }
        );
    }

    public static double driveCoefficient = 1;

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
            Rotation2d.fromDegrees(m_gyro.getGyroscopeYawDegrees()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            }
        );
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(m_gyro.getGyroscopeYawDegrees()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose
        );
    }

    /**
     * Method to drive the robot using joystick info.
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

        SwerveModuleState[] swerveModuleStates = DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(m_gyro.getGyroscopeYawDegrees()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, MAX_ROBOT_SPEED);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
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
            resetOdometry(new Pose2d());
            FIELDRELATIVEDRIVING = !FIELDRELATIVEDRIVING;
        });
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }
    

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getGyroscopeYawDegrees() * (GYROSCOPE_REVERSED ? -1.0 : 1.0);
    }
}