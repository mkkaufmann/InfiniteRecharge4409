package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeStopCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Intake intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeStopCommand(Intake _intake) {
      intake = _intake;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intake);
    }
      @Override
  public void initialize() {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}