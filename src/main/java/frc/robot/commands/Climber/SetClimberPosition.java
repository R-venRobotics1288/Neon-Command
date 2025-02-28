package frc.robot.commands.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.ClimberModule;

/**
 * Moves the Climber of the robot to a specified position.
 * @author Kara Smith and Aidan Fiedler
 * @version 0.1.0
 * @since 27-FEB-2025
 */
import static frc.robot.Constants.ClimberConstants.*;

public class SetClimberPosition extends Command {
    private ClimberModule ClimberModule;
    private double desiredPosition;
       private PIDController ClimberPIDController;
    private boolean finished = false;

    /**
     * Creates a new command to move the Climber arms to the specified position.
     * @param to desired position of the Climber
     * @param ClimberModule reference to the {@link ClimberModule}
     */
    public void MoveClimberCommand(double to, ClimberModule ClimberModule) {
        this.ClimberModule = ClimberModule;
        this.ClimberPIDController = new PIDController(CLIMBER_PID_P, CLIMBER_PID_I, CLIMBER_PID_D);
        this.ClimberPIDController.setTolerance(CLIMBER_TOLERANCE);
        this.desiredPosition = to;
        super.addRequirements(this.ClimberModule);
    }

    @Override
    public void initialize() {
        finished = false;
        ClimberPIDController.reset();
        ClimberPIDController.setSetpoint(desiredPosition);
    }

    @Override
    public void execute() {
        double outputRight = ClimberPIDController.calculate(ClimberModule.getEncoderPositionR());
        double outputLeft = ClimberPIDController.calculate(ClimberModule.getEncoderPositionL());
        ClimberModule.setMotorStateRight(outputRight);
        ClimberModule.setMotorStateLeft(outputLeft);
     
        SmartDashboard.putNumber("Climber Encoder Pos Right", ClimberModule.getEncoderPositionR());
        SmartDashboard.putNumber("Climber Encoder Pos Left", ClimberModule.getEncoderPositionL());
        SmartDashboard.putNumber("Climber Desired Pos", desiredPosition);
        SmartDashboard.putNumber("Commanded Motor Output Right", outputRight);
        SmartDashboard.putNumber("Commanded Motor Output Left", outputLeft);
        if (ClimberPIDController.atSetpoint()) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        ClimberModule.setMotorStateRight(0.0);
        ClimberModule.setMotorStateRight(0.0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}