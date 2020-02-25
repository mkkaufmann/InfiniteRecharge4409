package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GyroDriveStraightCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Drivetrain drivetrain;
private final boolean isHighGear ; 
private final double inches;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GyroDriveStraightCommand( Drivetrain _drivetrain, boolean _isHighGear, double _inches) {
      drivetrain = _drivetrain;
      isHighGear = _isHighGear;
      inches = _inches;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
      @Override
  public void initialize() {
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
return true;
  }
}