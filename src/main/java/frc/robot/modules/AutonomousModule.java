package frc.robot.modules;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    @SuppressWarnings("unused")
    private final PathPlannerAuto test_auto;
    @SuppressWarnings("unused")
    private final PathPlannerPath test_path;

    public SendableChooser<Command> autoChooser;

    private final Field2d test_field;

    public AutonomousModule(PositionModule positionModule, DriveModule driveModule) {
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
                    return alliance.get() == Alliance.Red;
                }
                return false;
            },
            this, this.positionModule, this.driveModule
        );

        autoChooser = AutoBuilder.buildAutoChooser();

        test_field = new Field2d();
        SmartDashboard.putData(test_field);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        test_auto = new PathPlannerAuto("Test");
        try {
            test_path = PathPlannerPath.fromPathFile("StraightLine");
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
            throw new IllegalStateException();
        }

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            test_field.setRobotPose(pose);
        });
    }

    /**
     * Gets the preloaded command to execute autonomously.
     * @return Autonomous {@link Command}
     */
    public Command getAutonomousCommand() {
        @SuppressWarnings("unused")
        PathConstraints constraints = new PathConstraints(AUTO_MAX_SPEED, AUTO_MAX_ACCELERATION, AUTO_MAX_ANGULAR_SPEED, AUTO_MAX_ANGULAR_ACCELERATION);
        // return AutoBuilder.pathfindThenFollowPath(test_path, constraints);
        return autoChooser.getSelected();
    }
}
