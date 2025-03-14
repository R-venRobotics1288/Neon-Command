package frc.robot.modules;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Manages autonomous mode, implementing path planning and calling other commands to run auto.
 * @author Joel Machens
 * @version 0.1.1
 * @since 27-JAN-2025
 */
public class AutonomousModule extends SubsystemBase {
    private final PositionModule positionModule;
    private final DriveModule driveModule;

    public SendableChooser<Command> autoChooser;

    public AutonomousModule(PositionModule positionModule, DriveModule driveModule, GyroscopeModule gyroModule) {
        this.positionModule = positionModule;
        this.driveModule = driveModule;

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError(e.getLocalizedMessage(), e.getStackTrace());
            throw new RuntimeException("Failed to load PathPlanner configuration settings! Auto has not been initialized!");
        }

        AutoBuilder.configure(
            this.positionModule::getRobotPose,
            this.positionModule::resetPosition,
            this.driveModule::getChassisSpeeds,
            this.driveModule::setModuleStates,
            new PPHolonomicDriveController(
                new PIDConstants(TRANSLATION_COEFFICIENT_P, TRANSLATION_COEFFICIENT_I, TRANSLATION_COEFFICIENT_D),
                new PIDConstants(ROTATION_COEFFICIENT_P, ROTATION_COEFFICIENT_I, ROTATION_COEFFICIENT_D)
            ),
            config,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    //return alliance.get() == Alliance.Red;
                }
                return false;
            },
            this, this.positionModule, this.driveModule
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Wheel Radius Characterization", DriveModule.wheelRadiusCharacterization(driveModule, gyroModule));
        SmartDashboard.putData("Selected Autonomous Routine", autoChooser);
    }

    /**
     * Gets the preloaded command to execute autonomously.
     * @return Autonomous {@link Command}
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Leave");//autoChooser.getSelected();
    }
}
