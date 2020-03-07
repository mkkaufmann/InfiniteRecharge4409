/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.CommandGroup;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.HopperStopCommand;
import frc.robot.RobotContainer.autonRoutine;
import frc.robot.commands.ClimberStopCommand;
import frc.robot.commands.DrivetrainDriveForwardCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
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

   RobotContainer container = new RobotContainer();
   Limelight limelight = container.getLimelight();
   Flywheel flywheel = container.getFlywheel();
   Drivetrain drivetrain = container.getDrivetrain();
   XboxController controller = new XboxController(0);
   XboxController partner = new XboxController(1);
   Intake intake = container.getIntake();
   Climber climber = new Climber();
   Hopper hopper = container.getHopper();
   IntakeStopCommand stopIntake = new IntakeStopCommand(intake);
   HopperStopCommand stopHopper = new HopperStopCommand(hopper);
   ClimberStopCommand stopClimber = new ClimberStopCommand(climber);
   Compressor compressor = new Compressor();
   CommandGroupBase command;
   int inverse = 1;


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
    //SmartDashboard.putNumber("Flywheel RPM", 0); 
    compressor.setClosedLoopControl(true);
    CameraServer.getInstance().startAutomaticCapture();
    }

  @Override
  public void autonomousInit() {
    limelight.run();
    drivetrain.resetEncoders();
    intake.deploy();
    command = container.getAutonomousCommand(autonRoutine.DRIVE_OFF_LINE_AND_SHOOT);
    command.schedule();
    compressor.setClosedLoopControl(true);
  }

  @Override
  public void autonomousPeriodic() {
    //drivetrain.cheesyDrive(.25, 0, false);
    System.out.println(drivetrain.getLeftEncoderValue()+ "left right" + drivetrain.getRightEncoderValue());
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void teleopInit() {
    intake.deploy();
    compressor.setClosedLoopControl(true);
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    //limelight.getLLdistance();
    boolean autoAim = Math.abs(controller.getTriggerAxis(Hand.kRight)) > 0.5;
    if(autoAim){
      drivetrain.shift(false);
      limelight.autoAim(drivetrain);
    }else{
        //limelight.stop();
        double xLeft = -controller.getX(Hand.kLeft)*inverse;
        double yRight = -controller.getY(Hand.kRight)*inverse;
        boolean quickTurn = Math.abs(controller.getTriggerAxis(Hand.kLeft)) > 0.5 || Math.abs(yRight) < 0.1;

        if(Math.abs(xLeft) < .15)
          xLeft = 0;
        if(Math.abs(yRight) < .1)
          yRight = 0;

        // drivetrain.tankDrive(yLeft, yRight);
        if(quickTurn){
          xLeft*=0.65;
        }
        drivetrain.cheesyDrive(yRight, -xLeft, quickTurn);
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
    if(controller.getAButton()){
      inverse*=-1;
    }
    boolean runFlywheel = Math.abs(partner.getTriggerAxis(Hand.kLeft))>0.5;
    if(runFlywheel){
      //limelight.run();
      flywheel.shootFromDistance(Math.abs(limelight.getLLdistance()));
    }else if(partner.getBumper(Hand.kLeft)){
      flywheel.poopyshoot();
    }else{
      flywheel.runRPM(0);
      //limelight.stop();
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
    if(runFlywheel || autoAim){
      limelight.run();
    }else{
      limelight.stop();
    }
  }

  @Override
  public void testInit() {
    
    compressor.setClosedLoopControl(true);
    intake.reset();
  }

  @Override
  public void testPeriodic() {
    intake.reset();
  }
}
