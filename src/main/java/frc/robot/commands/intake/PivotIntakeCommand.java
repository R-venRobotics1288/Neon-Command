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
public class PivotIntakeCommand extends Command {
    private final IntakeModule intakeModule;

    private PIDController pivotPIDController;
    private double desiredPosition = 0;
    private boolean finished = false;

    public PivotIntakeCommand(double desiredPosition, IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
        this.desiredPosition = desiredPosition;
        this.pivotPIDController = new PIDController(
            desiredPosition == Math.toRadians(PIVOT_DEGREE_DOWN) ? (PIVOT_PID_P * 0.75) : PIVOT_PID_P,
            PIVOT_PID_I,
            PIVOT_PID_D
        );
        this.pivotPIDController.setTolerance(POSITION_TOLERANCE);
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
        SmartDashboard.putNumber("Intake Pivot Encoder Pos", pivotEncoderPosition);
        SmartDashboard.putNumber("Intake Pivot Error", output);
        if (pivotPIDController.atSetpoint()) {
            finished = true;
            intakeModule.setIntakeState(desiredPosition == Math.toRadians(PIVOT_DEGREE_DOWN) ? IntakeState.DOWN : IntakeState.UP);
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
