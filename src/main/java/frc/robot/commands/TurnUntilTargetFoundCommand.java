package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TurnUntilTargetFoundCommand extends CommandBase{
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain drivetrain;
  private final Limelight limelight;
  
  public TurnUntilTargetFoundCommand(Drivetrain _drivetrain, Limelight _limelight){
    drivetrain = _drivetrain;
    limelight = _limelight;
    addRequirements(drivetrain, limelight); 
  }

  @Override
  public void initialize(){

  }

  @Override
  public void execute(){
    limelight.run();
    drivetrain.cheesyDrive(0, 0.4, true);
  }

  @Override
  public boolean isFinished(){
    return Limelight.targetFound();
  }
  
}
