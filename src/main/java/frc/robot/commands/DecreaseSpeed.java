package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class DecreaseSpeed extends Command {
    public DecreaseSpeed()
    {
        Constants.shooterspeed -= 0.01;
        if(Constants.shooterspeed <= 0)
        {
            Constants.shooterspeed = 0;
        }
    }
    
}
