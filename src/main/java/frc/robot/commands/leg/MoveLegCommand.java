package frc.robot.commands.leg;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.LegModule;
import frc.robot.utilities.LegState;

import static frc.robot.Constants.LegConstants.*;

/**
 * Moves the Leg of the robot to a specified position.
 * 
 * @author Nirmaha Mukherjee and Vaibhav Mohankumar
 * @version 0.1.0
 * @since 13-FEB-2025
 */
public class MoveLegCommand extends Command {
    private final LegModule legModule;

    private PIDController legPidController;
    private double desiredPosition = 0;
    private boolean finished = false;

    /**
     * Creates a new command to move the leg to the specified position.
     * 
     * @param to        desired position of the leg, in rotations
     * @param legModule reference to the {@link LegModule}
     */
    public MoveLegCommand(double to, LegModule legModule) {
        this.legModule = legModule;
        this.legPidController = new PIDController(LEG_PID_P, LEG_PID_I, LEG_PID_D);
        this.legPidController.setTolerance(LEG_TOLERANCE);
        this.desiredPosition = to;
        super.addRequirements(this.legModule);
    }

    @Override
    public void initialize() {
        finished = false;
        legPidController.reset();
        legPidController.setSetpoint(desiredPosition);
        if (desiredPosition == LEG_POS_INTAKING) {
            legModule.setLegState(LegState.INTAKING);
        } else if (desiredPosition == LEG_POS_REST) {
            legModule.setLegState(LegState.REST);
        } else if (desiredPosition == LEG_POS_ONE) {
            legModule.setLegState(LegState.POSITION_ONE);
        } else if (desiredPosition == LEG_POS_TWO) {
            legModule.setLegState(LegState.POSITION_TWO);
        } else if (desiredPosition == LEG_POS_THREE) {
            legModule.setLegState(LegState.POSITION_THREE);
        } else {
            legModule.setLegState(LegState.POSITION_FOUR);
        }
    }

    @Override
    public void execute() {
        double output = legPidController.calculate(legModule.getEncoderPosition(), desiredPosition);
        legModule.setMotorState(output);
        SmartDashboard.putNumber("Leg Encoder Pos", legModule.getEncoderPosition());
        SmartDashboard.putNumber("Leg Error", output);
        if (legPidController.atSetpoint()) {
            finished = true;
            System.out.println("Finished moving leg!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        legModule.setMotorState(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
