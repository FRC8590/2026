package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class IncreaseSpeed extends Command {
    public IncreaseSpeed()
    {
        Constants.shooterspeed += 0.01;
        if(Constants.shooterspeed >= 1)
        {
            Constants.shooterspeed = 1;
        }
    }
    
}