
package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;

public class FlywheelStopCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Flywheel flywheel;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlywheelStopCommand(Flywheel _flywheel) {
      flywheel = _flywheel;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(flywheel);
    }
  @Override
  public void initialize() {
  }

  @Override
  public void execute(){
    flywheel.runRPM(0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
