package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.ElevatorModule;

import static frc.robot.Constants.ElevatorConstants.*;

public class MoveElevatorCommand extends Command {
    private final ElevatorModule elevatorModule;
    private final double desiredPosition;

    private PIDController elevatorPIDController;
    private boolean finished = false;

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
    }

    @Override
    public void execute() {
        double output = elevatorPIDController.calculate(elevatorModule.getEncoderPosition());
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
