package frc.robot.commands.leg;

import static frc.robot.Constants.FootConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.LegModule;

public class MoveFootCommand extends Command {
    private final LegModule legModule;

    private boolean isIntaking = true;

    public MoveFootCommand(boolean isIntaking, LegModule legModule) {
        this.legModule = legModule;
        this.isIntaking = isIntaking;
        super.addRequirements(this.legModule);
    }

    @Override
    public void execute() {
        legModule.setFootMotorState(isIntaking ? MOTOR_SPEED : -MOTOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        legModule.setFootMotorState(0);
    }
}
