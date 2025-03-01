package frc.robot.commands.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.ClimberModule;
import static frc.robot.Constants.ClimberConstants.*;

/**
 * Moves the Climber of the robot to a specified position.
 * @author Kara Smith and Aidan Fiedler
 * @version 0.1.0
 * @since 27-FEB-2025
 */

public class ClimberPositionCommand extends Command {
    private ClimberModule climberModule;
    private double desiredPosition;
    private PIDController rightClimberPIDController;
    private PIDController leftClimberPIDController;
    private boolean finished = false;

    /**
     * Creates a new command to move the Climber arms to the specified position.
     * @param to desired position of the Climber
     * @param climberModule reference to the {@link ClimberModule}
     */
    public void MoveClimberCommand(double to, ClimberModule climberModule) {
        this.climberModule = climberModule;
        this.rightClimberPIDController = new PIDController(RIGHT_CLIMBER_PID_P, RIGHT_CLIMBER_PID_I, RIGHT_CLIMBER_PID_D);
        this.leftClimberPIDController = new PIDController(LEFT_CLIMBER_PID_P, LEFT_CLIMBER_PID_I, LEFT_CLIMBER_PID_D);
        this.rightClimberPIDController.setTolerance(CLIMBER_TOLERANCE);
        this.leftClimberPIDController.setTolerance(CLIMBER_TOLERANCE);
        this.desiredPosition = to;
        super.addRequirements(this.climberModule);
    }

    @Override
    public void initialize() {
        finished = false;
        leftClimberPIDController.reset();
        rightClimberPIDController.reset();
        leftClimberPIDController.setSetpoint(desiredPosition);
        rightClimberPIDController.setSetpoint(desiredPosition);
    }

    @Override
    public void execute() {
        double outputRight = rightClimberPIDController.calculate(climberModule.getEncoderPositionR());
        double outputLeft = leftClimberPIDController.calculate(climberModule.getEncoderPositionL());
        climberModule.setMotorStateRight(outputRight);
        climberModule.setMotorStateLeft(outputLeft);
     
        SmartDashboard.putNumber("Climber Encoder Pos Right", climberModule.getEncoderPositionR());
        SmartDashboard.putNumber("Climber Encoder Pos Left", climberModule.getEncoderPositionL());
        SmartDashboard.putNumber("Climber Desired Pos", desiredPosition);
        SmartDashboard.putNumber("Commanded Motor Output Right", outputRight);
        SmartDashboard.putNumber("Commanded Motor Output Left", outputLeft);
        if (rightClimberPIDController.atSetpoint() && leftClimberPIDController.atSetpoint()) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberModule.setMotorStateRight(0.0);
        climberModule.setMotorStateRight(0.0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}