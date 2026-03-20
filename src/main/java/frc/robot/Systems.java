package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public final class Systems {
    private static ShuffleboardTab tab = Shuffleboard.getTab("Systems");
    public static final GenericEntry enableDrive = tab
            .add("Drive", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    public static final GenericEntry enableIntakeWheels = tab
            .add("Intake Wheels", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    public static final GenericEntry enableIntakeArm = tab
            .add("Intake Arm", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    public static final GenericEntry enableIndexer = tab
            .add("Indexer", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    public static final GenericEntry enableBelt = tab
            .add("Belt", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    public static final GenericEntry enableShooter = tab
            .add("Shooter", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    /*
     * Is a system enabled?
     * 
     * If we're in a competition match or a simulation, all systems are enabled by
     * default.
     * Otherwise, all systems are disabled by default.
     */
    public static boolean isSystemEnabled(GenericEntry system) {
        return system.getBoolean(true);
    }
}
