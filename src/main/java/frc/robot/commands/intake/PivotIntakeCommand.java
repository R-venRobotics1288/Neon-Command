package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.IntakeModule;
import frc.robot.utilities.IntakeState;

import static frc.robot.Constants.IntakeConstants.*;

/**
 * Executes the intake mechanism of our robot (specifically for collecting coral)
 * @author Nirmaha Mukherjee
 * @version 0.1.0
 * @since 15-FEB-2025
 */
public class PivotIntakeCommand extends Command {
    private final IntakeModule intakeModule;
    private final double desiredPosition;

    public PivotIntakeCommand(double desiredPosition, IntakeModule intakeModule) {
        this.intakeModule = intakeModule;
        this.desiredPosition = desiredPosition;
        super.addRequirements(this.intakeModule);
    }

    @Override
    public void initialize() {
        if (desiredPosition == PIVOT_DEGREE_UP) {
            intakeModule.setPivotMotorState(0.15);
        } else {
            intakeModule.setPivotMotorState(-0.1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeModule.setPivotMotorState(0);
        IntakeState state = desiredPosition == PIVOT_DEGREE_DOWN ? IntakeState.DOWN : IntakeState.UP;
        intakeModule.setIntakeState(state);
        System.out.println("PIVOT COMPLETE: now at " + state);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(Units.degreesToRadians(desiredPosition), intakeModule.getPivotEncoderPosition(), POSITION_TOLERANCE);
    }
}
