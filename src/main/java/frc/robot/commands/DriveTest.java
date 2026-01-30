package frc.robot.commands;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.math.SwerveMath;
/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * <a href="https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java">...</a>
 */
public class DriveTest extends Command
{

  private final SwerveSubsystem swerveSubsystem;
  private final PIDController   driveController;
  private final SimpleMotorFeedforward driveFF;
  private double latestX, latestY, previousXPos, previousYPos;

  public DriveTest(SwerveSubsystem swerveSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    driveController = new PIDController(0.02, 0.0, 0);
    driveFF = new SimpleMotorFeedforward(0.01, 0.3, 0.2);
    previousXPos = 0;
    previousYPos = 0;
    
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
    latestX = 0;
    latestY = 0;
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {

    List<PhotonPipelineResult> list = null;

    DoubleSupplier x = new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        for(PhotonPipelineResult result : list){
          if(result.hasTargets()){
            latestX = -1 * (driveController.calculate(result.getBestTarget().getBestCameraToTarget().getX(), 2) + driveFF.calculate(0.5));
            return latestX;
          }
        }
        return latestX;
      }
    };

    DoubleSupplier y = new DoubleSupplier() {

      @Override
      public double getAsDouble() {
        for(PhotonPipelineResult result : list){
          if(result.hasTargets()){
            latestY = -1 * (driveController.calculate(result.getBestTarget().getBestCameraToTarget().getY(), 0) + driveFF.calculate(0.5));
            return latestY;
          }
        }
        return latestY;
      }
      
    };
    DoubleSupplier rot = new DoubleSupplier() {

      @Override
      public double getAsDouble() {
        return 0;
      }
      
    };


      swerveSubsystem.swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            x.getAsDouble() * swerveSubsystem.swerveDrive.getMaximumChassisVelocity(),
                            y.getAsDouble() * swerveSubsystem.swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(rot.getAsDouble(), 3) * swerveSubsystem.swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    swerveSubsystem.lock();
  }
}
