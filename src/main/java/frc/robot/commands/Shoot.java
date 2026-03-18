package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
public class Shoot extends ParallelCommandGroup {
    public Shoot() {
        addCommands(
            Constants.shooter.shooterSetGoalRPM(2000),
            Constants.belt.beltAndIndexerRun()
        );
        addRequirements(getRequirements());
        
    
    }
    
}
