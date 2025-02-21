// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.FIELDRELATIVEDRIVING;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.intake.IntakePivotCommand;
import frc.robot.modules.DriveModule;
import frc.robot.modules.GyroscopeModule;
import frc.robot.modules.IntakeModule;
import frc.robot.modules.PositionModule;
import frc.robot.modules.VisionModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // private final GyroscopeModule m_gyroscope;
  // private final VisionModule m_vision;
  // private final DriveModule m_drive;
  // @SuppressWarnings("unused")
  // private final PositionModule m_position;

  private final IntakeModule m_intake;

  // The driver's controller
  // CommandXboxController m_driverController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  private Command m_intakeCoralUpCommand;
  private Command m_intakeCoralDownCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // m_gyroscope = new GyroscopeModule();
    // m_vision = new VisionModule();
    // m_drive = new DriveModule(m_gyroscope);
    // m_position = new PositionModule(m_drive, m_vision, m_gyroscope);

    m_intake = new IntakeModule();
    m_intakeCoralUpCommand = new IntakePivotCommand(Math.toRadians(IntakeConstants.PIVOT_DEGREE_UP), m_intake);
    m_intakeCoralDownCommand = new IntakePivotCommand(Math.toRadians(IntakeConstants.PIVOT_DEGREE_DOWN), m_intake);

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
    // m_driverController.rightBumper().onTrue(m_drive.cutSpeed(true))
    //     .onFalse(m_drive.cutSpeed(false));
    // m_driverController.y().onTrue(m_drive.toggleFieldRelative());
    m_operatorController.leftTrigger().onTrue(m_intakeCoralUpCommand);
    m_operatorController.rightTrigger().onTrue(m_intakeCoralDownCommand);
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
