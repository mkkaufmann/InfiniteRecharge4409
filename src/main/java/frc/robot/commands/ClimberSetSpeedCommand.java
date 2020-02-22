package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberSetSpeedCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Climber climber;
  private final Double setSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberSetSpeedCommand(Climber _climber, double _setSpeed) {
      climber = _climber;
      setSpeed = _setSpeed;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(climber);
    }
      @Override
  public void initialize() {
    climber.up(setSpeed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}