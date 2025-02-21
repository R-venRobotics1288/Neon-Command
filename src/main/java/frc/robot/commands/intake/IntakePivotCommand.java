package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.IntakeModule;
import frc.robot.utilities.IntakeState;

import static frc.robot.Constants.IntakeConstants.*;

/**
 * Executes the intake mechanism of our robot (specifically for collecting coral)
 * @author Nirmaha Mukherjee
 * @version 0.1.0
 * @since 15-FEB-2025
 */
public class IntakePivotCommand extends Command {
    private final IntakeModule intakeModule;

    private PIDController pivotPIDController;
    private double desiredPosition = 0;
    private boolean finished = false;

    public IntakePivotCommand(double desiredPosition, IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
        this.desiredPosition = desiredPosition;
        this.pivotPIDController = new PIDController(PIVOT_PID_P, PIVOT_PID_I, PIVOT_PID_D);
        this.pivotPIDController.setTolerance(PIVOT_POSITION_TOLERANCE);
        super.addRequirements(this.intakeModule);
    }

    @Override
    public void initialize() {
        finished = false;
        pivotPIDController.setSetpoint(desiredPosition);
    }

    @Override
    public void execute() {
        double pivotEncoderPosition = intakeModule.getPivotEncoderPosition(); 
        double output = pivotPIDController.calculate(pivotEncoderPosition);
        intakeModule.setPivotMotorState(output);
        SmartDashboard.putNumber("Intake Encoder Pos", pivotEncoderPosition);
        SmartDashboard.putNumber("Intake Error", output);
        if (pivotPIDController.atSetpoint()) {
            finished = true;
            intakeModule.setIntakeState(pivotEncoderPosition < 0 ? IntakeState.UP : IntakeState.DOWN); // up is negative
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeModule.setPivotMotorState(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
