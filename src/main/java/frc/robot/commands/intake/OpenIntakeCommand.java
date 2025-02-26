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

    private PIDController openerPIDController;

    public OpenIntakeCommand(double desiredPosition, IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
        this.desiredPosition = desiredPosition;
        this.openerPIDController = new PIDController(OPENER_PID_P, OPENER_PID_I, OPENER_PID_D);
        this.openerPIDController.setTolerance(POSITION_TOLERANCE);
        super.addRequirements(this.intakeModule);
    }

    @Override
    public void initialize() {
        finished = false;
        openerPIDController.reset();
        openerPIDController.setSetpoint(desiredPosition);
    }

    @Override
    public void execute() {
        double openerEncoderPosition = intakeModule.getOpenerEncoderPosition(); 
        double output = openerPIDController.calculate(openerEncoderPosition);
        intakeModule.setOpenerMotorState(output);
        SmartDashboard.putNumber("Intake Opener Encoder Pos", openerEncoderPosition);
        SmartDashboard.putNumber("Intake Opener Error", output);
        if (openerPIDController.atSetpoint()) {
            finished = true;
            intakeModule.setIntakeState(openerEncoderPosition < 0 ? IntakeState.OPEN : IntakeState.CLOSED); // up is negative
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeModule.setOpenerMotorState(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
