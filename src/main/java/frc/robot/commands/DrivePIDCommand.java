package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivePIDCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain drivetrain;
  private final double distance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivePIDCommand(Drivetrain _drivetrain, double _distanceInFeet) {
      drivetrain = _drivetrain;
      distance = _distanceInFeet;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
  @Override
  public void initialize() {
  }

  @Override
  public void execute(){
    double left = drivetrain.getLeftPIDCommandController().calculate(drivetrain.getLeftEncoderValue()/404.285, distance);
    double right = drivetrain.getRightPIDCommandController().calculate(drivetrain.getRightEncoderValue()/404.285, distance);
    drivetrain.setOutput(left, right);
  }

  @Override
  public boolean isFinished() {
    return drivetrain.getLeftPIDCommandController().atSetpoint() && drivetrain.getRightPIDCommandController().atSetpoint();
  }
}
