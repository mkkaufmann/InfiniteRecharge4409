package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberStopCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Climber climber;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberStopCommand(Climber _climber) {
      climber = _climber;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(climber);
    }
      @Override
  public void initialize() {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}