package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Limelight limelight;
  private final Drivetrain drivetrain;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimelightAimCommand(Limelight _limelight, Drivetrain _drivetrain) {
      limelight = _limelight;
      drivetrain = _drivetrain;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(limelight);
      addRequirements(drivetrain);
    }
      @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    limelight.autoAim(drivetrain, 0);
  }

  @Override
  public boolean isFinished() {
    
    return true;
  }
}