package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;

public class FlywheelSpinUpCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Flywheel flywheel;
  private final double inches;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlywheelSpinUpCommand(Flywheel _flywheel, double _inches) {
      flywheel = _flywheel;
      inches = _inches;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(flywheel);
    }
  @Override
  public void initialize() {
  }

  @Override
  public void execute(){
    flywheel.shootFromDistance(inches);
  }
  
  @Override
  public boolean isFinished() {
    return true;
}
}
