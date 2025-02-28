package frc.robot.commands.leg;

import static frc.robot.Constants.FootConstants.MOTOR_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.FootModule;

public class MoveFootCommand extends Command {
    private final FootModule footModule;

    private boolean isIntaking = true;

    public MoveFootCommand(boolean isIntaking, FootModule footModule) {
        this.footModule = footModule;
        this.isIntaking = isIntaking;
        super.addRequirements(this.footModule);
    }

    @Override
    public void execute() {
        footModule.setMotorState(isIntaking ? MOTOR_SPEED : -MOTOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        footModule.setMotorState(0);
    }
}
