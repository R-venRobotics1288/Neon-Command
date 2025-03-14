package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.IntakeModule;
import frc.robot.utilities.IntakeState;

import static frc.robot.Constants.IntakeConstants.*;

public class OpenIntakeCommand extends Command {
    private final IntakeModule intakeModule;
    private final double desiredPosition;

    private boolean finished = false;

    private PIDController rightOpenerMotorPIDController;
    private PIDController leftOpenerMotorPidController;

    public OpenIntakeCommand(double desiredPosition, IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
        this.desiredPosition = desiredPosition;
        this.rightOpenerMotorPIDController = new PIDController(RIGHT_OPENER_PID_P, RIGHT_OPENER_PID_I, RIGHT_OPENER_PID_D);
        this.leftOpenerMotorPidController = new PIDController(LEFT_OPENER_PID_P, LEFT_OPENER_PID_I, LEFT_OPENER_PID_D);
        this.rightOpenerMotorPIDController.setTolerance(POSITION_TOLERANCE);
        this.leftOpenerMotorPidController.setTolerance(POSITION_TOLERANCE);
        super.addRequirements(this.intakeModule);
    }

    @Override
    public void initialize() {
        finished = false;
        rightOpenerMotorPIDController.reset();
        rightOpenerMotorPIDController.setSetpoint(desiredPosition);
        leftOpenerMotorPidController.reset();
        leftOpenerMotorPidController.setSetpoint(-desiredPosition);
    }

    @Override
    public void execute() {
        double rightOpenerEncoderPosition = intakeModule.getRightOpenerEncoderPosition();
        double rightOutput = rightOpenerMotorPIDController.calculate(rightOpenerEncoderPosition);
        intakeModule.setRightOpenerMotorState(rightOutput);
        SmartDashboard.putNumber("Right Intake Opener Encoder Pos", rightOpenerEncoderPosition);
        SmartDashboard.putNumber("Right Intake Opener Error", rightOutput);
        double leftOpenerEncoderPosition = intakeModule.getLeftOpenerEncoderPosition();
        double leftOutput = leftOpenerMotorPidController.calculate(leftOpenerEncoderPosition);
        intakeModule.setLeftOpenerMotorState(leftOutput);
        SmartDashboard.putNumber("Right Intake Opener Encoder Pos", leftOpenerEncoderPosition);
        SmartDashboard.putNumber("Right Intake Opener Error", leftOutput);
        if (rightOpenerMotorPIDController.atSetpoint() && leftOpenerMotorPidController.atSetpoint()) {
            finished = true;
            intakeModule.setIntakeState(leftOpenerEncoderPosition < 0 ? IntakeState.OPEN : IntakeState.CLOSED); // up is negative
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeModule.setRightOpenerMotorState(0);
        intakeModule.setLeftOpenerMotorState(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
