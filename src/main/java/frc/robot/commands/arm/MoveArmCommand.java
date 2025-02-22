package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.ArmModule;

import static frc.robot.Constants.ArmConstants.*;

/**
 * Moves the Arm of the robot to a specified position.
 * @author Nirmaha Mukherjee and Vaibhav Mohankumar
 * @version 0.1.0
 * @since 13-FEB-2025
 */
public class MoveArmCommand extends Command {
    private final ArmModule armModule;

    private PIDController armPidController;
    private double desiredPosition = 0;
    private boolean finished = false;

    /**
     * Creates a new command to move the arm to the specified position.
     * @param to desired position of the arm, in rotations
     * @param armModule reference to the {@link ArmModule}
     */
    public MoveArmCommand(double to, ArmModule armModule) {
        this.armModule = armModule;
        this.armPidController = new PIDController(ARM_PID_P, ARM_PID_I, ARM_PID_D);
        this.armPidController.setTolerance(ARM_TOLERANCE);
        this.desiredPosition = to;
        super.addRequirements(this.armModule);
    }

    @Override
    public void initialize() {
        finished = false;
        armPidController.reset();
        armPidController.setSetpoint(desiredPosition);
    }

    @Override
    public void execute() {
        double output = armPidController.calculate(armModule.getEncoderPosition(), desiredPosition);
        armModule.setMotorState(output);
        SmartDashboard.putNumber("Arm Encoder Pos", armModule.getEncoderPosition());
        SmartDashboard.putNumber("Arm Error", output);
        if (armPidController.atSetpoint()) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        armModule.setMotorState(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
