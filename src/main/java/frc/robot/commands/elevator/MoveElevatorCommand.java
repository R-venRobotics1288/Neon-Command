package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.ElevatorModule;
import frc.robot.utilities.ElevatorState;

/**
 * Moves the Elevator of the robot to a specified position.
 * @author Nirmaha Mukherjee and Vaibhav Mohankumar
 * @version 0.1.0
 * @since 22-FEB-2025
 */
import static frc.robot.Constants.ElevatorConstants.*;

public class MoveElevatorCommand extends Command {
    private final ElevatorModule elevatorModule;
    private final double desiredPosition;

    private PIDController elevatorPIDController;
    private boolean finished = false;

    /**
     * Creates a new command to move the elevator to the specified position.
     * 
     * @param to             desired position of the elevator
     * @param elevatorModule reference to the {@link ElevatorModule}
     */
    public MoveElevatorCommand(double to, ElevatorModule elevatorModule) {
        this.elevatorModule = elevatorModule;
        this.elevatorPIDController = new PIDController(ELEVATOR_PID_P, ELEVATOR_PID_I, ELEVATOR_PID_D);
        this.elevatorPIDController.setTolerance(ELEVATOR_TOLERANCE);
        this.desiredPosition = to;

        super.addRequirements(this.elevatorModule);
    }

    @Override
    public void initialize() {
        finished = false;
        elevatorPIDController.reset();
        elevatorPIDController.setSetpoint(desiredPosition);

        if (desiredPosition == ELEVATOR_SAFE_POS) {
            elevatorModule.setElevatorState(ElevatorState.LEVEL_SAFE);
        } else if (desiredPosition == ELEVATOR_ZERO_POS) {
            elevatorModule.setElevatorState(ElevatorState.LEVEL_ZERO);
        } else {
            elevatorModule.setElevatorState(ElevatorState.LEVEL_MAX);
        }
    }

    @Override
    public void execute() {
        double output = -elevatorPIDController.calculate(elevatorModule.getEncoderPosition());
        elevatorModule.setMotorState(output);
        SmartDashboard.putNumber("Elevator Encoder Pos", elevatorModule.getEncoderPosition());
        SmartDashboard.putNumber("Elevator Desired Pos", desiredPosition);
        SmartDashboard.putNumber("Commanded Motor Output", output);
        if (elevatorPIDController.atSetpoint()) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorModule.setMotorState(0.0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
