// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.ElevatorConstants;
import static frc.robot.Constants.IntakeConstants;
import static frc.robot.Constants.LegConstants;
import static frc.robot.Constants.OIConstants;

import org.littletonrobotics.urcl.URCL;

import frc.robot.modules.AutonomousModule;
import frc.robot.modules.DriveModule;
import frc.robot.modules.ElevatorModule;
import frc.robot.modules.GyroscopeModule;
import frc.robot.modules.IntakeModule;
import frc.robot.modules.PositionModule;
import frc.robot.modules.VisionModule;
import frc.robot.modules.LegModule;

import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OpenIntakeCommand;
import frc.robot.commands.intake.PivotIntakeCommand;
import frc.robot.commands.leg.RunFootCommand;
import frc.robot.commands.leg.MoveLegCommand;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final GyroscopeModule m_gyroscope;
	private final VisionModule m_vision;
	public final DriveModule m_drive;
	private final ElevatorModule m_elevator;
	private final IntakeModule m_intake;
	private final LegModule m_leg;
	private final PositionModule m_position;
	private final AutonomousModule m_auto;

	// The driver's controller
	CommandXboxController m_driverController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
	CommandXboxController m_operatorController = new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_gyroscope = new GyroscopeModule();
		m_vision = new VisionModule();
		m_drive = new DriveModule(m_gyroscope);
		m_position = new PositionModule(m_drive, m_vision, m_gyroscope);
		m_elevator = new ElevatorModule();
		m_leg = new LegModule();
		m_intake = new IntakeModule();

		// Register and Initialize Autonomous Module
		NamedCommands.registerCommand("ElevatorMax", new MoveElevatorCommand(ElevatorConstants.ELEVATOR_MAX_POS, m_elevator));
		NamedCommands.registerCommand("ElevatorSafe", new MoveElevatorCommand(ElevatorConstants.ELEVATOR_SAFE_POS, m_elevator));
		NamedCommands.registerCommand("LegRest", new MoveLegCommand(LegConstants.LEG_POS_REST, m_leg));
		NamedCommands.registerCommand("LegMax", new MoveLegCommand(LegConstants.LEG_POS_FOUR, m_leg));
		NamedCommands.registerCommand("Score", new RunFootCommand(false, m_leg));
		NamedCommands.registerCommand("IntakeDown", new PivotIntakeCommand(Math.toRadians(IntakeConstants.PIVOT_DEGREE_DOWN), m_intake));
		NamedCommands.registerCommand("IntakeUp", new PivotIntakeCommand(Math.toRadians(IntakeConstants.PIVOT_DEGREE_UP), m_intake));
		NamedCommands.registerCommand("Intake", new IntakeCommand(true, false, m_intake).alongWith(new RunFootCommand(true, m_leg)));
		m_auto = new AutonomousModule(m_position, m_drive, m_gyroscope);
		
		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// The left stick controls translation of the robot.
		// Turning is controlled by the X axis of the right stick.
		m_drive.setDefaultCommand(
				new RunCommand(() -> {
					double xInput = m_driverController.getLeftX();
					double yInput = m_driverController.getLeftY();
					double thetaInput = m_driverController.getRightX();
					double distanceFromZero = Math.sqrt(Math.pow(xInput, 2) + Math.pow(yInput, 2));
					if (distanceFromZero < OIConstants.DRIVE_DEADBAND) {
						xInput = 0;
						yInput = 0;
					}
					m_drive.drive(
							Math.pow(yInput, 3) * Math.abs(yInput),
							Math.pow(xInput, 3) * Math.abs(xInput),
							Math.pow(MathUtil.applyDeadband(-thetaInput, OIConstants.DRIVE_DEADBAND), 3)
									* Math.abs(thetaInput),
							DriveConstants.FIELD_RELATIVE_DRIVING);
				}, m_drive));

		DataLogManager.start();
		URCL.start();
		DriverStation.startDataLog(DataLogManager.getLog());
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		// DRIVER Left Bumper -> Cut Speed
		m_driverController.leftBumper().onTrue(m_drive.cutSpeed(true)).onFalse(m_drive.cutSpeed(false));

		// DRIVER Left Middle Button -> Swerve Alignment
		m_driverController.button(OIConstants.SWERVE_ALIGNMENT_BUTTON).onTrue(new RunCommand(() -> {
			m_gyroscope.resetGyroscope();
		}, m_gyroscope));

		// #region Normal Bindings
		// DRIVER Button A -> Toggles Pivot
		Command pivotCommand = Commands.either(
			/*new MoveLegCommand(LegConstants.LEG_POS_REST, m_leg)
				.onlyIf(m_leg::isIntakingPosition)
				.andThen*/(new PivotIntakeCommand(Math.toRadians(IntakeConstants.PIVOT_DEGREE_UP), m_intake)),
			new PivotIntakeCommand(Math.toRadians(IntakeConstants.PIVOT_DEGREE_DOWN), m_intake),
			m_intake::isDown);
		m_driverController.a().onTrue(pivotCommand);

		// DRIVER Button B -> Toggles Jaw
		Command jawCommand = Commands.either(
			new OpenIntakeCommand(Math.toRadians(IntakeConstants.OPENER_DEGREE_OPEN), m_intake),
			new OpenIntakeCommand(Math.toRadians(IntakeConstants.OPENER_DEGREE_CLOSE), m_intake),
			m_intake::isClosed).onlyWhile(m_intake::isDown);
		m_driverController.b().onTrue(jawCommand);

		// OPERATOR Right Trigger -> Intakes
		Command intakeCommand = Commands.either(
			new MoveLegCommand(LegConstants.LEG_POS_INTAKING, m_leg)
				.onlyIf(m_leg::isNotIntakingPosition)
				.andThen(new IntakeCommand(true, false, m_intake).alongWith(new RunFootCommand(true, m_leg))),
			new IntakeCommand(false, false, m_intake),
			m_intake::isClosed).onlyWhile(m_intake::isDown);
		m_operatorController.rightTrigger().whileTrue(intakeCommand);

		// OPERATOR Left Bumper -> Clears stuck intake.
		m_operatorController.leftBumper().whileTrue(new IntakeCommand(true, true, m_intake).onlyWhile(m_intake::isDown));

		// OPERATOR Right Bumper -> Score with foot.
		m_operatorController.rightBumper().whileTrue(new RunFootCommand(false, m_leg));

		// OPERATOR Button X -> Leg Position One
		m_operatorController.x()
			.onTrue(new MoveElevatorCommand(ElevatorConstants.ELEVATOR_SAFE_POS, m_elevator)/*.andThen(new MoveLegCommand(LegConstants.LEG_POS_ONE, m_leg)).onlyIf(m_leg::isNotInPositionOne)*/);

		// OPERATOR Button A -> Leg Position Two
		m_operatorController.a()
			.onTrue(/*new MoveElevatorCommand(ElevatorConstants.ELEVATOR_SAFE_POS, m_elevator).andThen*/(new MoveLegCommand(LegConstants.LEG_POS_TWO, m_leg)).onlyIf(m_leg::isNotInPositionTwo));

		// OPERATOR Button B -> Leg Position Three
		m_operatorController.b()
			.onTrue(/*new MoveElevatorCommand(ElevatorConstants.ELEVATOR_SAFE_POS, m_elevator).andThen*/(new MoveLegCommand(LegConstants.LEG_POS_THREE, m_leg)).onlyIf(m_leg::isNotInPositionThree));

		// OPERATOR Button Y -> Leg Position Four
		m_operatorController.y()
			.onTrue(/*new MoveElevatorCommand(ElevatorConstants.ELEVATOR_MAX_POS, m_elevator).andThen*/(new MoveLegCommand(LegConstants.LEG_POS_FOUR, m_leg)).onlyIf(m_leg::isNotInPositionFour));

		// OPERATOR Joystick Right BUTTON -> Leg to Rest Position and Reset Elevator
		m_operatorController.rightStick()
			.onTrue(new MoveLegCommand(LegConstants.LEG_POS_REST, m_leg).andThen(new MoveElevatorCommand(ElevatorConstants.ELEVATOR_ZERO_POS, m_elevator)).onlyIf(m_leg::isNotAtRest));
		// #endregion Normal Bindings

		// #region Test Bindings
		// TEST INTAKE AT 15% POWER
		m_operatorController.povLeft().and(m_operatorController.button(OIConstants.ENABLE_TESTING_BUTTON))
				.onTrue(
						new RunCommand(() -> {
							m_intake.setIntakeMotorState(0.15);
						}, m_intake))
				.onFalse(
						new RunCommand(() -> {
							m_intake.setIntakeMotorState(0);
						}, m_intake));
		// TEST FEEDER AT 15% POWER
		m_operatorController.povUp().and(m_operatorController.button(OIConstants.ENABLE_TESTING_BUTTON))
				.onTrue(
						new RunCommand(() -> {
							m_intake.setFeederMotorState(0.15);
						}, m_intake))
				.onFalse(
						new RunCommand(() -> {
							m_intake.setFeederMotorState(0);
						}, m_intake));
		// TEST FOOT AT 15% POWER
		m_operatorController.povRight().and(m_operatorController.button(OIConstants.ENABLE_TESTING_BUTTON))
				.onTrue(
						new RunCommand(() -> {
							m_leg.setFootMotorState(0.15);
						}, m_leg))
				.onFalse(
						new RunCommand(() -> {
							m_leg.setFootMotorState(0);
						}, m_leg));
		// #endregion Test Bindings
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return m_auto.getAutonomousCommand();
	}
}
