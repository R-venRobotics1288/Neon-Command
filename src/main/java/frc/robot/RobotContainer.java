// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OpenIntakeCommand;
import frc.robot.commands.intake.PivotIntakeCommand;
import frc.robot.modules.DriveModule;
import frc.robot.modules.ElevatorModule;
import frc.robot.modules.GyroscopeModule;
import frc.robot.modules.IntakeModule;
import frc.robot.modules.PositionModule;
import frc.robot.modules.VisionModule;
import frc.robot.utilities.IntakeState;
import frc.robot.modules.ElevatorModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // private final GyroscopeModule m_gyroscope;
  // private final VisionModule m_vision;
  // private final DriveModule m_drive;
  private final ElevatorModule m_elevator;
  private final IntakeModule m_intake;

  private Command elevatorToLevelFourCommand;
  private Command elevatorToSafeHeightCommand;
  private Command elevatorToZeroCommand;

  //@SuppressWarnings("unused") private final PositionModule m_position;

	// The driver's controller
	CommandXboxController m_driverController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
	CommandXboxController m_operatorController = new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

	// Intake commands
	private Command m_intakePivotUpCommand;
	private Command m_intakePivotDownCommand;
	private Command m_intakeOpenCommand;
	private Command m_intakeCloseCommand;
	private Command m_intakeCoralCommand;
	private Command m_intakeAlgaeCommand;
	private Command m_intakeReverseCommand;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// // m_gyroscope = new GyroscopeModule();
		// // m_vision = new VisionModule();
		// // m_drive = new DriveModule(m_gyroscope);
		// // m_position = new PositionModule(m_drive, m_vision, m_gyroscope);
		m_intake = new IntakeModule();
    m_elevator = new ElevatorModule();
	
		m_intakePivotUpCommand = new PivotIntakeCommand(Math.toRadians(IntakeConstants.PIVOT_DEGREE_UP), m_intake);
		m_intakePivotDownCommand = new PivotIntakeCommand(Math.toRadians(IntakeConstants.PIVOT_DEGREE_DOWN), m_intake);
		m_intakeOpenCommand = new OpenIntakeCommand(Math.toRadians(IntakeConstants.OPENER_DEGREE_OPEN), m_intake);
		m_intakeCloseCommand = new OpenIntakeCommand(Math.toRadians(IntakeConstants.OPENER_DEGREE_CLOSE), m_intake);
		m_intakeCoralCommand = new IntakeCommand(true, false, m_intake);
		m_intakeAlgaeCommand = new IntakeCommand(false, false, m_intake);
		m_intakeReverseCommand = new IntakeCommand(true, true, m_intake);

    // Configures elevator commands
    elevatorToLevelFourCommand = new MoveElevatorCommand(ElevatorConstants.LEVEL_FOUR_POS, m_elevator);
    elevatorToSafeHeightCommand = new MoveElevatorCommand(ElevatorConstants.ELEVATOR_SAFE_HEIGHT, m_elevator);
    elevatorToZeroCommand = new MoveElevatorCommand(ElevatorConstants.ELEVATOR_ZERO_POS, m_elevator);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// m_drive.setDefaultCommand(
		//   // The left stick controls translation of the robot.
		//   // Turning is controlled by the X axis of the right stick.
		//   new RunCommand(
		//     () -> {
		//       double xInput = m_driverController.getLeftX();
		//       double yInput = m_driverController.getLeftY();
		//       double thetaInput = m_driverController.getRightX();
		//       double distanceFromZero = Math.sqrt(Math.pow(xInput, 2) + Math.pow(yInput, 2));
		//       if (distanceFromZero < OIConstants.DRIVE_DEADBAND) {
		//         xInput = 0;
		//         yInput = 0;
		//       }
		//       m_drive.drive(
		//         Math.pow(yInput, 3) * Math.abs(yInput),
		//               Math.pow(xInput, 3) * Math.abs(xInput),
		//               Math.pow(MathUtil.applyDeadband(-thetaInput, OIConstants.DRIVE_DEADBAND), 3)
		//               * Math.abs(thetaInput),
		//         FIELDRELATIVEDRIVING);
		//     }, m_drive
		//   ));
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
		// m_driverController.leftBumper().onTrue(m_drive.cutSpeed(true))
		//     .onFalse(m_drive.cutSpeed(false));
		// m_driverController.y().onTrue(m_drive.toggleFieldRelative());

		// TODO: ensure leg is moved away BEFORE we schedule pivot up
		m_driverController.a().onTrue(new RunCommand(() -> {
			if (m_intake.hasIntakeState(IntakeState.UP)) {
				if (!m_intakePivotDownCommand.isScheduled()) {
					if (m_intakePivotUpCommand.isScheduled())
						m_intakePivotUpCommand.cancel();
					m_intakePivotDownCommand.schedule();
				}
			} else {
				if (!m_intakePivotUpCommand.isScheduled()) {
					if (m_intakePivotDownCommand.isScheduled())
						m_intakePivotDownCommand.cancel();
					m_intakePivotUpCommand.schedule();
				}
			}
		}, m_intake));

		m_driverController.b().onTrue(new RunCommand(() -> {
			if (m_intake.hasIntakeState(IntakeState.OPEN)) {
				if (!m_intakeCloseCommand.isScheduled()) {
					if (m_intakeOpenCommand.isScheduled())
						m_intakeOpenCommand.cancel();
					m_intakeCloseCommand.schedule();
				}
			} else {
				if (!m_intakeOpenCommand.isScheduled()) {
					if (m_intakeCloseCommand.isScheduled())
						m_intakeCloseCommand.cancel();
					m_intakeOpenCommand.schedule();
				}
			}
		}, m_intake));

		m_driverController.rightTrigger().onChange(new RunCommand(() -> {
			if (m_driverController.rightTrigger().getAsBoolean() && m_intake.hasIntakeState(IntakeState.DOWN)) {
				if (m_intakeReverseCommand.isScheduled())
					m_intakeReverseCommand.cancel();

				if (m_intake.hasIntakeState(IntakeState.CLOSED)) {
					m_intakeCoralCommand.schedule();
				} else {
					m_intakeAlgaeCommand.schedule();
				}
			} else {
				if (m_intakeCoralCommand.isScheduled())
					m_intakeCoralCommand.cancel();
				if (m_intakeAlgaeCommand.isScheduled())
					m_intakeAlgaeCommand.cancel();
			}
		}, m_intake));

		m_driverController.rightBumper().onChange(new RunCommand(() -> {
			if (m_driverController.rightBumper().getAsBoolean() && m_intake.hasIntakeState(IntakeState.DOWN)) {
				if (m_intakeCoralCommand.isScheduled())
					m_intakeCoralCommand.cancel();
				if (m_intakeAlgaeCommand.isScheduled())
					m_intakeAlgaeCommand.cancel();

				m_intakeReverseCommand.schedule();
			} else {
				m_intakeReverseCommand.cancel();
			}
		}, m_intake));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return new RunCommand(() -> {
		});
	}
}
