package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class HopperFeedCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Hopper hopper;
  private final double feedSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HopperFeedCommand(Hopper _hopper, double _feedSpeed) {
      hopper = _hopper;
      feedSpeed = _feedSpeed;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(hopper);
    }
      @Override
  public void initialize() {
    hopper.feed(0.75);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}