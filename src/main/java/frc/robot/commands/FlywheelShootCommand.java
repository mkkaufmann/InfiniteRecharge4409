package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;

public class FlywheelShootCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Flywheel flywheel;
  private final Hopper hopper;
  private final Limelight limelight;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlywheelShootCommand(Flywheel _flywheel, Hopper _hopper, Limelight _limelight) {
      flywheel = _flywheel;
      limelight = _limelight;
      hopper = _hopper;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(flywheel);
    }
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
      flywheel.shootFromDistance(limelight.getLLdistance());
      if(flywheel.getRPM() > flywheel.getDistanceRPM(limelight.getLLdistance())*.95){
          hopper.feed(-.4);
      }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}