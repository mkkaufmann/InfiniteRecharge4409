package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardForTime extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain drivetrain;
  private final double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForwardForTime(Drivetrain _drivetrain, double _speed) {
      drivetrain = _drivetrain;
      speed = _speed;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
  @Override
  public void initialize() {
  }

  @Override
  public void execute(){
    drivetrain.cheesyDrive(speed, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
