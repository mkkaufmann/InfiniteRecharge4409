package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Utilities;

public class DrivetrainTurnPIDCommand extends CommandBase{
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain drivetrain;
  private final double angle;
  
  public DrivetrainTurnPIDCommand(Drivetrain _drivetrain, double _angle){
    drivetrain = _drivetrain;
    angle = _angle;
  }

  @Override
  public void initialize(){

  }

  @Override
  public void execute(){
    drivetrain.turnAnglePID(angle);
  }

  @Override
  public boolean isFinished(){
    return Utilities.epsilonEquals(angle, drivetrain.getAngle(), 0.5);
  }
  
}