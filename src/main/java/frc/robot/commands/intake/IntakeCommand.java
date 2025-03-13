package frc.robot.commands.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.IntakeModule;

/**
 * Command used to run the intake system when deployed.
 * @author Joel Machens & Vaibhav Mohankumar
 * @version 0.1.0
 * @since 27-FEB-2025
 */
public class IntakeCommand extends Command {
    private final IntakeModule intakeModule;
    private final boolean coral;
    private final boolean reversed;

    private PIDController feederPID = new PIDController(FEEDER_PID_P, FEEDER_PID_I, FEEDER_PID_D);
    private PIDController intakePID = new PIDController(INTAKE_PID_P, INTAKE_PID_I, INTAKE_PID_D);

    /**
     * Initializes the intake command.
     * @param coral true for inhaling coral, false for inhaling algae
     * @param reversed true for outtake, false for intake
     * @param intakeModule reference to the {@link IntakeModule}
     * @param runFootCommand reference to the foot intake command
     */
    public IntakeCommand(boolean coral, boolean reversed, IntakeModule intakeModule) {
        this.coral = coral;
        this.reversed = reversed;
        this.intakeModule = intakeModule;

        this.feederPID.setTolerance(VELOCITY_TOLERANCE);
        this.feederPID.setSetpoint(this.reversed ? -INTAKE_SPEED_RPS : INTAKE_SPEED_RPS);
        this.intakePID.setTolerance(VELOCITY_TOLERANCE);
        this.intakePID.setSetpoint(this.reversed ? -INTAKE_SPEED_RPS : INTAKE_SPEED_RPS);

        super.addRequirements(intakeModule);
    }

    @Override
    public void initialize() {
        feederPID.reset();
        intakePID.reset();
    }

    @Override
    public void execute() {
        intakeModule.setIntakeMotorState(intakePID.calculate(intakeModule.getIntakeEncoderVelocity()));
        if (coral) {
            intakeModule.setFeederMotorState(feederPID.calculate(intakeModule.getFeederEncoderVelocity()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.err.println("IntakeCommand exited without being interrupted! This shouldn't happen!");
            return;
        }
        intakeModule.setFeederMotorState(0);
        intakeModule.setIntakeMotorState(0);
    }
}
