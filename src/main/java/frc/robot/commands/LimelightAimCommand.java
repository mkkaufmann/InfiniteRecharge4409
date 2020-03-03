package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Limelight limelight;
  private final Drivetrain drivetrain;
  private final int settleTime = 1;
  private double startOfSettleTimestamp;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimelightAimCommand(Limelight _limelight, Drivetrain _drivetrain) {
      limelight = _limelight;
      drivetrain = _drivetrain;
      startOfSettleTimestamp = DriverStation.getInstance().getMatchTime();
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(limelight);
      addRequirements(drivetrain);
    }
      @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    limelight.autoAim(drivetrain);
  }

  @Override
  public boolean isFinished() {
    if(!(Math.abs(limelight.getOffset()) < 0.5)){
      startOfSettleTimestamp = DriverStation.getInstance().getMatchTime();
    }
    return DriverStation.getInstance().getMatchTime() - settleTime > startOfSettleTimestamp;
  }
}