package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Implements runtime-editable constants using the WPILib Shuffleboard
 * @author Aidan Fiedler
 * @version 0.1.0
 * @since 17-JAN-25
 */
public class ShuffleManager {
    private ShuffleboardTab shuffleTab = Shuffleboard.getTab("tooning");
    private GenericEntry slew = shuffleTab.addPersistent("xy slew", BASE_SLEW_RATE).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 50)).getEntry();
    private GenericEntry maxSpeed = shuffleTab.addPersistent("max speed", MAX_ROBOT_SPEED).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 75)).getEntry();
    private GenericEntry maxRot = shuffleTab.addPersistent("max rot per s", MAX_ROBOT_ROTATIONS_PER_SECOND).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 30)).getEntry();
    private GenericEntry rotSlew = shuffleTab.addPersistent("rotation slew", MAX_ROBOT_ROTATIONS_PER_SECOND).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 30)).getEntry();
    public GenericEntry elevator_target = shuffleTab.add("elevator target", 0).getEntry();
    public GenericEntry elevator_real =  shuffleTab.add("elevator real", 0).getEntry();

    public void initShuffleboard() {
        updateShuffleboard();
    }
    public void updateShuffleboard() {
        if (SHUFFLE_MANAGER_ENABLED) {
            //MAX_ROBOT_SPEED = maxSpeed.getDouble(MAX_ROBOT_SPEED);
            if (MAX_ROBOT_SPEED != maxSpeed.getDouble(MAX_ROBOT_SPEED)) {
                MAX_ROBOT_SPEED = maxSpeed.getDouble(MAX_ROBOT_SPEED);
                System.out.println("changed speed");
            }
            if (MAX_ROBOT_ROTATIONS_PER_SECOND != maxRot.getDouble(MAX_ROBOT_ROTATIONS_PER_SECOND)){
                MAX_ROBOT_ROTATIONS_PER_SECOND = maxRot.getDouble(MAX_ROBOT_ROTATIONS_PER_SECOND);
                MAX_ANGULAR_SPEED = maxRot.getDouble(MAX_ROBOT_ROTATIONS_PER_SECOND) * 2 * Math.PI;
            }
            if (BASE_SLEW_RATE != slew.getDouble(BASE_SLEW_RATE)) {
                BASE_SLEW_RATE = slew.getDouble(BASE_SLEW_RATE);
                SLEW_FILTER_X = new SlewRateLimiter(BASE_SLEW_RATE);
                SLEW_FILTER_Y = new SlewRateLimiter(BASE_SLEW_RATE);
            }
            if (ROT_SLEW_RATE != rotSlew.getDouble(ROT_SLEW_RATE)) {
                ROT_SLEW_RATE = rotSlew.getDouble(ROT_SLEW_RATE);
                ROTATION_FILTER = new SlewRateLimiter(rotSlew.getDouble(ROT_SLEW_RATE));
            }
        }
    }
}