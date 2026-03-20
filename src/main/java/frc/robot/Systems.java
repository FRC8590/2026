package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public final class Systems {
    private static ShuffleboardTab tab = Shuffleboard.getTab("Systems");
    public static final GenericEntry enableDrive = tab
        .add("Enable Drive", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
    public static final GenericEntry enableIntakeWheels = tab
        .add("Enable Intake Wheels", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
    public static final GenericEntry enableIntakeArm = tab
        .add("Enable Intake Arm", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
    public static final GenericEntry enableShooter = tab
        .add("Enable Shooter", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

    /* Is a system enabled?
     * 
     * If we're in a competition match or a simulation, all systems are enabled by default.
     * Otherwise, all systems are disabled by default.
     */
    public static boolean isSystemEnabled(GenericEntry system)
    {
        return system.getBoolean(true);
    }
}
