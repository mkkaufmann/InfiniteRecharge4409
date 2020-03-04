package frc.robot;

import frc.robot.commands.DrivetrainDriveForwardCommand;
import frc.robot.commands.DrivetrainTurnCommand;
import frc.robot.commands.DrivetrainTurnPIDCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.LimelightAimCommand;
import frc.robot.commands.FlywheelShootCommand;
import frc.robot.commands.IntakeIntakeCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

public class RobotContainer {
  private Limelight limelight = new Limelight();
  private Flywheel flywheel = new Flywheel();
  private Hopper hopper = new Hopper();
  private Drivetrain drivetrain = new Drivetrain();
  private Intake intake = new Intake();

  public Drivetrain getDrivetrain(){
    return drivetrain;
  }
  public Limelight getLimelight(){
    return limelight;
  }
  public Flywheel getFlywheel(){
    return flywheel;
  }
  public Hopper getHopper(){
    return hopper;
  }
  public Intake getIntake(){
    return intake;
  }
  public enum autonRoutine{
    DRIVE_OFF_LINE,
    DRIVE_OFF_LINE_AND_SHOOT,
    PUSH_TEAM_OFF_LINE,
    PUSH_OFF_LINE_AND_SHOOT,
    PICK_UP_TWO_AND_SHOOT
  }
  public Command getAutonomousCommand(autonRoutine routine) {
    Command command;

    TrajectoryConfig config = new TrajectoryConfig(2, 2);
    config.setKinematics(drivetrain.getKinematics());

    Trajectory trajectory;
    switch(routine){
      case DRIVE_OFF_LINE:
        command = new DrivetrainDriveForwardCommand(drivetrain, 1, 0.5);
        break;
      case DRIVE_OFF_LINE_AND_SHOOT:
        command = new DrivetrainDriveForwardCommand(drivetrain, -5, 0.5)
        .andThen(new LimelightAimCommand(limelight, drivetrain).withTimeout(3)
        .andThen(new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5)));
        break;
      case PUSH_TEAM_OFF_LINE:
        command = new DrivetrainDriveForwardCommand(drivetrain, 6, 0.5)
        .andThen(new DrivetrainDriveForwardCommand(drivetrain, -12, 0.5));
        break;
      case PUSH_OFF_LINE_AND_SHOOT:
        command = new DrivetrainDriveForwardCommand(drivetrain, 1, 0.5)
        .andThen(new DrivetrainDriveForwardCommand(drivetrain, -6, 0.5))
        .andThen(new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5));
        break;
      case PICK_UP_TWO_AND_SHOOT:
        command = new IntakeIntakeCommand(intake)
        .alongWith(new DrivetrainDriveForwardCommand(drivetrain, -1, 0.5)).andThen(new DrivetrainDriveForwardCommand(drivetrain, 12, 0.5)).andThen(new DrivetrainTurnPIDCommand(drivetrain, 180)).andThen(new LimelightAimCommand(limelight, drivetrain)).andThen(new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5));
        break;
      default:
        trajectory = TrajectoryGenerator.generateTrajectory(
          Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
          config
        );
        command = new RamseteCommand(
          trajectory,
          drivetrain::getPose,
          new RamseteController(),
          drivetrain.getFeedForward(),
          drivetrain.getKinematics(),
          drivetrain::getSpeeds,
          drivetrain.getleftPIDController(),
          drivetrain.getrightPIDController(),
          drivetrain::setOutput,
          drivetrain
        );
    }
    
    return command;
  }
};

