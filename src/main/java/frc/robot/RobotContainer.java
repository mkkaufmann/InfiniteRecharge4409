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
import frc.robot.commands.FlywheelStopCommand;
import frc.robot.commands.HopperStopCommand;
import frc.robot.commands.IntakeIntakeCommand;
import frc.robot.commands.IntakeStopCommand;
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
        command = new SequentialCommandGroup(
          new DriveForwardForTime(drivetrain, -0.25).withTimeout(3),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1),
          new LimelightAimCommand(limelight, drivetrain).withTimeout(2),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1),
          new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5),
          new FlywheelStopCommand(flywheel),
          new HopperStopCommand(hopper)
        );
        break;
      case DRIVE_OFF_LINE:
          command = new SequentialCommandGroup(
            new DriveForwardForTime(drivetrain, 0.75).withTimeout(1),
            new DriveForwardForTime(drivetrain, 0).withTimeout(0.1)
            );
        break;
      case DRIVE_OFF_LINE_AND_SHOOT:
        command = new SequentialCommandGroup(
          new DriveForwardForTime(drivetrain, -0.5).withTimeout(3),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1),
          new LimelightAimCommand(limelight, drivetrain).withTimeout(2),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1),
          new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5),
          new FlywheelStopCommand(flywheel),
          new HopperStopCommand(hopper)
        );
        break;
      case PUSH_TEAM_OFF_LINE:
        command = new SequentialCommandGroup(
          new DriveForwardForTime(drivetrain, 0.75).withTimeout(1),
          new DriveForwardForTime(drivetrain, -0.75).withTimeout(2),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1)
        );
        break;
      case PUSH_OFF_LINE_AND_SHOOT:
        command = new SequentialCommandGroup(
          new DriveForwardForTime(drivetrain, 0.5).withTimeout(1),
          new DriveForwardForTime(drivetrain, -0.5).withTimeout(3.5),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1),
          new LimelightAimCommand(limelight, drivetrain).withTimeout(2),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1),
          new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5),
          new FlywheelStopCommand(flywheel),
          new HopperStopCommand(hopper)
        );
        break;
      case PICK_UP_TWO_AND_SHOOT:
        command = new SequentialCommandGroup(
          new IntakeIntakeCommand(intake),
          new DriveForwardForTime(drivetrain, 0.25).withTimeout(3.5),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1),
          new DrivetrainTurnPIDCommand(drivetrain, 180).withTimeout(3),
          new LimelightAimCommand(limelight, drivetrain).withTimeout(2),
          new DriveForwardForTime(drivetrain, 0).withTimeout(0.1),
          new FlywheelShootCommand(flywheel, hopper, limelight).withTimeout(5),
          new FlywheelStopCommand(flywheel),
          new HopperStopCommand(hopper),
          new IntakeStopCommand(intake)
        );
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

