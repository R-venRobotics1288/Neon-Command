// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class MAXSwerveModule {
	private final SparkMax m_drivingSpark;
	private final SparkMax m_turningSpark;

	private final RelativeEncoder m_drivingEncoder;
	private final RelativeEncoder m_turningEncoder;

	private final PIDController m_drivingClosedLoopController;
	private final PIDController m_turningClosedLoopController;

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
		m_turningEncoder = m_turningSpark.getEncoder();
		m_absoluteEncoder = new CANcoder(absoluteEncoderCANId);

		m_drivingClosedLoopController = new PIDController(0.1, 0, 0);
		m_turningClosedLoopController = new PIDController(0.4, 0, 0);

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
		return new SwerveModuleState(m_turningEncoder.getVelocity(),
				new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
				new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	public double getAbsoluteEncoderRad() {
		return m_absoluteEncoder.getAbsolutePosition().getValueAsDouble();
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
        final double driveOutput = m_drivingClosedLoopController.calculate(m_drivingEncoder.getVelocity(), correctedDesiredState.speedMetersPerSecond);		
        final double turnOutput = m_turningClosedLoopController.calculate(getAbsoluteEncoderRad(), correctedDesiredState.angle.getRadians());

        final double driveFeedforward = new PIDController(1, 0, 0).calculate(correctedDesiredState.speedMetersPerSecond);

        m_drivingSpark.set(driveOutput + driveFeedforward / 3);
        m_turningSpark.set(turnOutput / 3);

		m_desiredState = desiredState;
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		m_drivingEncoder.setPosition(0);
	}
}
