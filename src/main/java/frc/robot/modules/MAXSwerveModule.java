// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

import static frc.robot.Constants.DriveConstants.*;

public class MAXSwerveModule {
	private final SparkMax m_drivingSpark;
	private final SparkMax m_turningSpark;

	private final RelativeEncoder m_drivingEncoder;
	private final RelativeEncoder m_turningEncoder;

	private final PIDController m_drivingPIDController;
	private final PIDController m_turningPIDController;
	private final PIDController m_feedForwardPIDController;

	private final CANcoder m_absoluteEncoder;

	private double m_chassisAngularOffset = 0;
	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
	 * Encoder.
	 */
	public MAXSwerveModule(int drivingCANId, int turningCANId, int absoluteEncoderCANId, double chassisAngularOffset) {
		m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
		m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

		m_drivingEncoder = m_drivingSpark.getEncoder();
		m_drivingEncoder.setPosition(0);
		m_turningEncoder = m_turningSpark.getEncoder();
		m_turningEncoder.setPosition(0);
		m_absoluteEncoder = new CANcoder(absoluteEncoderCANId);

		m_drivingPIDController = new PIDController(TRANSLATION_COEFFICIENT_P, TRANSLATION_COEFFICIENT_I, TRANSLATION_COEFFICIENT_D);
		m_turningPIDController = new PIDController(ROTATION_COEFFICIENT_P, ROTATION_COEFFICIENT_I, ROTATION_COEFFICIENT_D);
		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
		m_feedForwardPIDController = new PIDController(1, 0, 0);

		// Apply the respective configurations to the SPARKS. Reset parameters before
		// applying the configuration to bring the SPARK to a known good state. Persist
		// the settings to the SPARK to avoid losing them on a power cycle.
		m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
		m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

		m_chassisAngularOffset = chassisAngularOffset;
		m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
		m_drivingEncoder.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(m_drivingEncoder.getVelocity(),
				new Rotation2d(getAbsoluteEncoderRad() + m_chassisAngularOffset));
	}

	public void periodic() {
		// SmartDashboard.putNumber("Swerve" + this.m_drivingSpark.getDeviceId(), m_drivingEncoder.getPosition());
		// SmartDashboard.putNumber("AbsoluteEncoder" + this.m_drivingSpark.getDeviceId(), Math.toDegrees(getAbsoluteEncoderRad()));
		// SmartDashboard.putNumber("Swerve" + this.m_drivingSpark.getDeviceId() + "Angles", getPosition().angle.getDegrees());
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(
				m_drivingEncoder.getPosition(),
				new Rotation2d(getAbsoluteEncoderRad() + m_chassisAngularOffset)); // TODO: Investagate if offset should be applied
	}

	public double getAbsoluteEncoderRad() {
		return (m_absoluteEncoder.getAbsolutePosition().getValueAsDouble()) * (2 * Math.PI);
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

		// Optimize the reference state to avoid spinning further than 90 degrees.
		correctedDesiredState.optimize(new Rotation2d(getAbsoluteEncoderRad()));

		// Command driving and turning SPARKS towards their respective setpoints.
        final double driveOutput = m_drivingPIDController.calculate(m_drivingEncoder.getVelocity(), correctedDesiredState.speedMetersPerSecond);		
        final double turnOutput = m_turningPIDController.calculate(getAbsoluteEncoderRad(), correctedDesiredState.angle.getRadians());

        final double driveFeedforward = m_feedForwardPIDController.calculate(correctedDesiredState.speedMetersPerSecond);
		m_feedForwardPIDController.reset();

        m_drivingSpark.set(driveOutput + driveFeedforward / 3);
		//m_drivingSpark.set(driveOutput);
        m_turningSpark.set(turnOutput / 3);

		m_desiredState = desiredState;
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		m_drivingEncoder.setPosition(0);
	}
}