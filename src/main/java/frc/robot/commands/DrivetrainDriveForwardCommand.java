package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainDriveForwardCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain drivetrain;
  private final double distance;
  private double leftEncoderInit;
  private double rightEncoderInit;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivetrainDriveForwardCommand(Drivetrain _drivetrain, double _distance) {
      drivetrain = _drivetrain;
      distance = _distance;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);
    }
  @Override
  public void initialize() {
      leftEncoderInit = drivetrain.getLeftEncoder().getEncoder().getPosition()/20.054;
      rightEncoderInit = drivetrain.getRightEncoder().getEncoder().getPosition()/20.054;
  }

  @Override
  public void execute(){
    drivetrain.shift(false);
    drivetrain.cheesyDrive(.25, drivetrain.getAngle()/15.0, false);

  }

  @Override
  public boolean isFinished() {
    //return (drivetrain.getLeftEncoder()-);
  }
}