package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Utilities;

public class DrivetrainDriveForwardCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain drivetrain;
  private final double distance;
  private double leftEncoderInit;
  private double rightEncoderInit;
  private final double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivetrainDriveForwardCommand(Drivetrain _drivetrain, double _distance, double _speed) {
      drivetrain = _drivetrain;
      distance = _distance;
      if(distance >=0){
        speed = _speed;
      }else{
        speed = -_speed;
      }
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
  @Override
  public void initialize() {
      leftEncoderInit = drivetrain.getLeftEncoderValue()/404.285;
      rightEncoderInit = drivetrain.getRightEncoderValue()/404.285;
  }

  @Override
  public void execute(){
    drivetrain.shift(false);
    drivetrain.cheesyDrive(speed, 0, false);

  }

  @Override
  public boolean isFinished() {
    double leftValue = drivetrain.getLeftEncoderValue()/404.285 - leftEncoderInit;
    double rightValue = drivetrain.getRightEncoderValue()/404.285 - rightEncoderInit;
    return Utilities.epsilonEquals((leftValue + rightValue)/2, 1);
  }
}
