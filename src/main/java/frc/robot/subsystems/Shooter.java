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
    private double setSpeed = 0, p, i, d, kA, kV, cV, mA;

    public Shooter() {
        p = 0.0001;
        i = 0;
        d = 0;
        kA = 0;
        kV = 0.0019;
        cV = 6700;
        mA = 2000;

        shooterConfig // configure motors
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.001);
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

        SmartDashboard.putNumber("Shooter Speed", setSpeed);
    }

    private void runMotors(double speed) {
        setSpeed = speed;
    }

    private void stopMotors() {
        setSpeed = 0;
    }

    /**
     * Run the shooter motors at the speed set on the SmartDashboard
     * 
     * @return a command tht runs the shooter motors at a set speed
     */
    public Command runShooter() {
        return run(() -> runMotors(setSpeed));
    }

    /**
     * runs both shooter motors at full speed
     * 
     * @param speed target RPM for the shooter motors
     * @return Command that sets both motors to full speed
     */
    public Command runShooter(double speed) {
        return run(() -> runMotors(speed));
    }

    /**
     * stops both shooter motors
     * 
     * @return Command that stops both motors
     */
    public Command stopShooter() {
        return run(() -> stopMotors());
    }

    /**
     * This method will be called once per scheduler run
     * Put values you want to moniter here
     */
    @Override
    public void periodic() {
        frontMotor.getClosedLoopController().setSetpoint(setSpeed, SparkBase.ControlType.kMAXMotionVelocityControl);
        backMotor.getClosedLoopController().setSetpoint(setSpeed, SparkBase.ControlType.kMAXMotionVelocityControl);

        setSpeed = SmartDashboard.getNumber("Shooter Speed", setSpeed);
        SmartDashboard.putNumber("Set Shooter Speed ", setSpeed);
        SmartDashboard.putNumber("Front Motor RPM ", frontMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Back Motor RPM ", backMotor.getEncoder().getVelocity());
    }
}
