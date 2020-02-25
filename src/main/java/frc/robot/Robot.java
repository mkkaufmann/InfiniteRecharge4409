/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.HopperStopCommand;
import frc.robot.RobotContainer.autonRoutine;
import frc.robot.commands.ClimberStopCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */


   Flywheel flywheel = new Flywheel();
   Limelight limelight = new Limelight();
   Drivetrain drivetrain = new Drivetrain();
   XboxController controller = new XboxController(0);
   XboxController partner = new XboxController(1);
   Intake intake = new Intake();
   Climber climber = new Climber();
   Hopper hopper = new Hopper();
   RobotContainer container = new RobotContainer();
   IntakeStopCommand stopIntake = new IntakeStopCommand(intake);
   HopperStopCommand stopHopper = new HopperStopCommand(hopper);
   ClimberStopCommand stopClimber = new ClimberStopCommand(climber);
   Compressor compressor = new Compressor();
   RamseteCommand command;

  @Override
  public void robotInit() {
    //CommandScheduler.getInstance().setDefaultCommand(intake, stopIntake);
    //CommandScheduler.getInstance().setDefaultCommand(hopper, stopHopper);
    //CommandScheduler.getInstance().setDefaultCommand(climber, stopClimber);
    CommandScheduler.getInstance().registerSubsystem(intake);
    CommandScheduler.getInstance().registerSubsystem(limelight);
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    CommandScheduler.getInstance().registerSubsystem(flywheel);
    CommandScheduler.getInstance().registerSubsystem(climber);
    CommandScheduler.getInstance().registerSubsystem(hopper);
    SmartDashboard.putNumber("Flywheel RPM", 0); 
    compressor.setClosedLoopControl(true);
  }

  @Override
  public void autonomousInit() {
<<<<<<< HEAD
    container.getAutonomousCommand(drivetrain, autonRoutine.DRIVE_OFF_LINE);
=======
    command = container.getAutonomousCommand(drivetrain)
    command.schedule();;

>>>>>>> d02abbdb918a146066d5eb7e236c267570b83560
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    drivetrain.driveDistance(2);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    //limelight.getLLdistance();
    limelight.run();
    if(Math.abs(controller.getTriggerAxis(Hand.kRight)) > 0.5){
      drivetrain.shift(false);
      limelight.autoAim(drivetrain, controller.getY(Hand.kRight));
    }else{
        //limelight.stop();
        double throttle = controller.getY(Hand.kRight);
        drivetrain.cheesyDrive(throttle, -controller.getX(Hand.kLeft), Math.abs(controller.getTriggerAxis(Hand.kLeft)) > 0.5 || Math.abs(throttle) < 0.1);
    }
    if(partner.getXButton()){
      intake.intake();
    }
    else if(partner.getBButton()){
      intake.outtake();
    }
    else{
      intake.stop();
    }
    if(controller.getYButton()){
      climber.up(1);
    }
    else if(controller.getAButton()){
      climber.down(1);
    }
    else{
      climber.stop();
    }
    if(partner.getTriggerAxis(Hand.kLeft) > 0.5){
      //flywheel.shootFromDistance(Math.abs(limelight.getLLdistance()));
      flywheel.runRPM(3850);
    }else{
      flywheel.runRPM(0);
    }
    if(controller.getBumperPressed(Hand.kRight)){
      drivetrain.shift(!drivetrain.getHighGear());
    }
    if(Math.abs(partner.getTriggerAxis(Hand.kRight)) > 0.5){
      hopper.feed(-0.4);
    }else if(partner.getAButton()){
      hopper.feed(0.4);
    }else{
      hopper.stop();
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
