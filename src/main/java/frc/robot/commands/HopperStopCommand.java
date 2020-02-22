package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class HopperStopCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Hopper hopper;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HopperStopCommand(Hopper _hopper) {
      hopper = _hopper;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(hopper);
    }
      @Override
  public void initialize() {
    hopper.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}