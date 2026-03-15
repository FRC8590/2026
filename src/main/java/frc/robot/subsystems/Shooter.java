package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Shooter extends SubsystemBase {
    private final SparkFlex frontMotor = new SparkFlex(Constants.SHOOTER_CONSTANTS.frontMotorID(),
            MotorType.kBrushless);
    private final SparkFlex backMotor = new SparkFlex(Constants.SHOOTER_CONSTANTS.backMotorID(), MotorType.kBrushless);

    private final SparkFlexConfig shooterConfig = new SparkFlexConfig();

    /**cV: cruiseVelocity. mA: maxAcceleration */
    private double goalRPM = 0, p, i, d, kA, kV, cV, mA;

    public Shooter() {
        p = 0.0001;
        i = 0;
        d = 0;
        kA = 0;
        kV = 0.0019;
        cV = 6700;
        mA = 4000; //todo increase

        shooterConfig // configure motors
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(60)
            .closedLoopRampRate(0.001); //todo look at this
        shooterConfig.closedLoop // configure PID
            .pid(p,i,d);
        shooterConfig.closedLoop.feedForward
            .kA(kA)
            .kV(kV);
        shooterConfig.closedLoop.maxMotion // configure maxMotion
            .cruiseVelocity(cV)
            .maxAcceleration(mA);

        frontMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterConfig.inverted(true);
        backMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.putNumber("Shooter Speed", goalRPM);
    }

    private void setGoalRPM (double rpm) {
        System.out.println("Set shooter goal RPM to " + rpm);
        goalRPM = rpm;
    }

    public double distanceToRPM ()
    {
        return 0;
    }

    /**
     * If the average RPM of the front and back shooter motors is at or above the goal RPM
     * @return true if average RPM is at or above goal RPM
     */
    public boolean atRPM ()
    {
        System.out.println("DEBUG " + goalRPM);
        return (frontMotor.getEncoder().getVelocity() + backMotor.getEncoder().getVelocity()) / 2 >= goalRPM;
    }

    /**
     * Change the goal RPM of the shooter. If rpm is greater than 6784, goal RPM is not changed.
     * @param rpm rotations per minute you want the shooter to run at
     * @return command that sets the goal RPM
     */
    public Command shooterSetGoalRPM (double rpm)
    {
        if (rpm > 6784) return run(()->setGoalRPM(goalRPM));
        return run(()->setGoalRPM (rpm));
    }

    /**
     * Sets the goal RPM to zero
     * @return command that sets the goal RPM to 0
     */
    public Command shooterStop ()
    {
        return run(()->setGoalRPM(0));
    }

    /**
     * This method will be called once per scheduler run
     * Put values you want to moniter here
     */
    @Override
    public void periodic() {
        frontMotor.getClosedLoopController().setSetpoint(goalRPM, SparkBase.ControlType.kMAXMotionVelocityControl);
        backMotor.getClosedLoopController().setSetpoint(goalRPM, SparkBase.ControlType.kMAXMotionVelocityControl);

        goalRPM = SmartDashboard.getNumber("Shooter Speed", goalRPM);
        SmartDashboard.putNumber("Set Shooter Speed ", goalRPM);
        SmartDashboard.putNumber("Front Motor RPM ", frontMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Back Motor RPM ", backMotor.getEncoder().getVelocity());
    }
}
