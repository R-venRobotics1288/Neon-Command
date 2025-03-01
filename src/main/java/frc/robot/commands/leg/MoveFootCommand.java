package frc.robot.commands.leg;

import static frc.robot.Constants.FootConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.LegModule;

public class MoveFootCommand extends Command {
    private final LegModule legModule;
    private final PIDController footPIDController;

    private final boolean isIntaking;

    public MoveFootCommand(boolean isIntaking, LegModule legModule) {
        this.legModule = legModule;
        this.isIntaking = isIntaking;

        this.footPIDController = new PIDController(MOTOR_PID_P, MOTOR_PID_I, MOTOR_PID_D);
        this.footPIDController.setTolerance(FOOT_TOLERANCE);
        this.footPIDController.setSetpoint(this.isIntaking ? FOOT_SPEED_RPS : -FOOT_SPEED_RPS);

        super.addRequirements(this.legModule);
    }

    @Override
    public void initialize() {
        footPIDController.reset();
    }

    @Override
    public void execute() {
        double state = footPIDController.calculate(legModule.getFootEncoderVelocity());
        SmartDashboard.putNumber("Foot Velocity", legModule.getFootEncoderVelocity());
        SmartDashboard.putNumber("Foot State", FOOT_SPEED_RPS);
        SmartDashboard.putNumber("Foot Current (Amps)", legModule.getFootMotorOutputCurrent());
        legModule.setFootMotorState(state);
    }

    @Override
    public void end(boolean interrupted) {
        legModule.setFootMotorState(0);
    }
}
