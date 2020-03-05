package frc.robot;

import frc.robot.commands.DriveForwardForTime;
import frc.robot.commands.DrivetrainDriveForwardCommand;
import frc.robot.commands.DrivetrainTurnCommand;
import frc.robot.commands.DrivetrainTurnPIDCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Arrays;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.commands.LimelightAimCommand;
import frc.robot.commands.FlywheelShootCommand;
import frc.robot.commands.IntakeIntakeCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

public class RobotContainer {
  private static Limelight limelight = new Limelight();
  private static Flywheel flywheel = new Flywheel();
  private static Hopper hopper = new Hopper();
  private static Drivetrain drivetrain = new Drivetrain();
  private static Intake intake = new Intake();

  public static Drivetrain getDrivetrain(){
    return drivetrain;
  }
  public static Limelight getLimelight(){
    return limelight;
  }
  public static Flywheel getFlywheel(){
    return flywheel;
  }
  public static Hopper getHopper(){
    return hopper;
  }
  public static Intake getIntake(){
    return intake;
  }
  public enum autonRoutine{
    TEST,
    DRIVE_OFF_LINE,
    DRIVE_OFF_LINE_AND_SHOOT,
    PUSH_TEAM_OFF_LINE,
    PUSH_OFF_LINE_AND_SHOOT,
    PICK_UP_TWO_AND_SHOOT
  }

  Command stopDrive = new DriveForwardForTime(drivetrain, 0);

  //private static Command driveForward = new StartEndCommand(()->RobotContainer.getDrivetrain().cheesyDrive(0.25, , isQuickTurn);), onEnd, requirements)
  public CommandGroupBase getAutonomousCommand(autonRoutine routine) {
    CommandGroupBase command;

    TrajectoryConfig config = new TrajectoryConfig(2, 2);
    config.setKinematics(drivetrain.getKinematics());

    Trajectory trajectory;
    switch(routine){
      case TEST:
        //command = new DriveForwardForTime(drivetrain, 0.25).withTimeout(1).andThen(stopDrive);
        //command = new SequentialCommandGroup(new WaitCommand(1), new DriveForwardForTime(drivetrain, 0.25)).withTimeout(3);
        command = (((((new DriveForwardForTime(drivetrain, -0.5).withTimeout(3)).andThen(new LimelightAimCommand(limelight, drivetrain)).withTimeout(2))).andThen(stopDrive)).andThen(new FlywheelShootCommand(flywheel, hopper, limelight)).withTimeout(5));
        break;
      case DRIVE_OFF_LINE:
      command = new DriveForwardForTime(drivetrain, 0.75).withTimeout(1).andThen(stopDrive);
        break;
      case DRIVE_OFF_LINE_AND_SHOOT:
        command = new DrivetrainDriveForwardCommand(drivetrain, -5, 0.5).andThen(stopDrive)
        .andThen(new LimelightAimCommand(limelight, drivetrain).withTimeout(3)
        .andThen(new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5)));
        break;
      case PUSH_TEAM_OFF_LINE:
        command = new DrivetrainDriveForwardCommand(drivetrain, 6, 0.5)
        .andThen(new DrivetrainDriveForwardCommand(drivetrain, -12, 0.5)).andThen(stopDrive);
        break;
      case PUSH_OFF_LINE_AND_SHOOT:
        command = new DrivetrainDriveForwardCommand(drivetrain, 1, 0.5)
        .andThen(new DrivetrainDriveForwardCommand(drivetrain, -6, 0.5)).andThen(stopDrive)
        .andThen(new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5));
        break;
      case PICK_UP_TWO_AND_SHOOT:
        command = new IntakeIntakeCommand(intake)
        .alongWith(new DrivetrainDriveForwardCommand(drivetrain, -1, 0.5)).andThen(new DrivetrainDriveForwardCommand(drivetrain, 12, 0.5)).andThen(stopDrive).andThen(new DrivetrainTurnPIDCommand(drivetrain, 180).withTimeout(2)).andThen(stopDrive).andThen(new LimelightAimCommand(limelight, drivetrain)).andThen(stopDrive).andThen(new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5));
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
        ).withTimeout(15);
    }
    
    return command;
  }
};

