package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

public class FlywheelShootCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Flywheel flywheel;
  private final Hopper hopper;
  private double distance;
  private double timer;
  private double time;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlywheelShootCommand(Flywheel _flywheel, Hopper _hopper, double _distance, double _time) {
      flywheel = _flywheel;
      distance = _distance;
      hopper = _hopper;
      time = _time;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(flywheel);
    }
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
      flywheel.shootFromDistance(distance);
      if(flywheel.getRPM() > flywheel.getDistanceRPM(distance)*.95){
          hopper.feed(-.4);
          timer = DriverStation.getInstance().getMatchTime();
      }
  }

  @Override
  public boolean isFinished() {
    return DriverStation.getInstance().getMatchTime() - timer >= time;
  }
}