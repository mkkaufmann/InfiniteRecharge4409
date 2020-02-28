package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainTurnCommand extends CommandBase{
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain drivetrain;
  private final double angle;
  
  public DrivetrainTurnCommand(Drivetrain _drivetrain, double _angle){
    drivetrain = _drivetrain;
    angle = _angle;
  }

  @Override
  public void initialize(){

  }

  @Override
  public void execute(){
    drivetrain.setOutput(angle/180.0, -angle/180.0);    
  }

  @Override
  public boolean isFinished(){
    if(angle < 0){
      return drivetrain.getAngle() < angle;
    }else{
      return drivetrain.getAngle() > angle;
    }
  }
  
}
