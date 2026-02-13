package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Climb extends SubsystemBase
{
    /**
     * Climb will deffinetly be more complicated,
     * but this is a start 
     * - Riley
     */
    /**
     * Climb is really not that difficult 
     * - Nathan
     */
    private final SparkMax climbMotor = new SparkMax(Constants.CLIMB_CONSTANTS.climbMotorID(), MotorType.kBrushless);

    private final SparkMaxConfig climbMotorConfig = new SparkMaxConfig(); 
    public RelativeEncoder climbEncoder = climbMotor.getEncoder();

    public Climb ()
    {
        climbMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.001);
        
        climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    private void stopClimbMotor ()
    {
        climbMotor.set(0);
    } 
    public void extendClimbMotor(){
            climbMotor.set(-0.2);
        
    }
    public void retractClimbMotor() {
            climbMotor.set(0.2);
        

    }
    public void extendComplete() {
        if(climbEncoder.getPosition() < 100) {
            extendClimbMotor();
        }

    }

    /**
     * Start the ladder climb
     * @return Command that starts the ladder climb
     */
    public Command extend ()
    {
        return run(() -> extendClimbMotor());
    }

     /**
     * Retract the ladder climb 
     * @return Command that starts the ladder climb
     */
    public Command retract ()
    {
        return run(() -> retractClimbMotor());
    }

    /**
     * stop the ladder climb
     * @return Command that stops the ladder climb
     */
    public Command stopClimb ()
    {
        return run(() -> stopClimbMotor());
    }
}
