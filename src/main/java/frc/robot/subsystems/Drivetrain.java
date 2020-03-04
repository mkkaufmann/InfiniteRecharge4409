package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CheesyDriveHelper;
import frc.robot.util.DriveSignal;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.geometry.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.util.Units;


public class Drivetrain extends SubsystemBase {

  private static final double LOW_GEAR_RATIO = 15.32; //more torque
  private static final double HIGH_GEAR_RATIO = 7.08; //more speed
  private static final double SHIFT_TO_LOW_THRESHOLD = 2; //fps
  private static final double SHIFT_TO_HIGH_THRESHOLD = 4; //fps
  private static final double OUTPUT_VOLTS = 12;
  private static final double WHEEL_RADIUS = 2.5;
  private boolean isHighGear = false;

  CANSparkMax leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave1 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave2 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave2 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  DoubleSolenoid shifter = new DoubleSolenoid(2, 5);


  double gearRatio = LOW_GEAR_RATIO;
  double ks = 0;
  double kv = 0;
  double ka = 0;
  double Kp = 0;
  double Ki = 0;
  double Kd = 0;


  AHRS navx = new AHRS(SPI.Port.kMXP);
  Pose2d currentPose = new Pose2d();

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

  PIDController leftPIDController = new PIDController(Kp, Ki, Kd);
  PIDController rightPIDController = new PIDController(Kp, Ki, Kd);
  
  PIDController leftPIDCommandController = new PIDController(Kp, Ki, Kd);
  PIDController rightPIDCommandController = new PIDController(Kp, Ki, Kd);

  PIDController leftController = new PIDController(0,0,0);
  PIDController rightController = new PIDController(0,0,0);

  CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

  public Drivetrain(){
    leftMaster.setInverted(true);
    rightMaster.setInverted(false);

    leftMaster.setSmartCurrentLimit(25);
    rightMaster.setSmartCurrentLimit(25);

    leftSlave1.setSmartCurrentLimit(25);
    leftSlave2.setSmartCurrentLimit(25);
    rightSlave1.setSmartCurrentLimit(25);
    rightSlave2.setSmartCurrentLimit(25);

    leftPIDCommandController.setTolerance(1);
    rightPIDCommandController.setTolerance(1);

    leftSlave1.follow(leftMaster);
    leftSlave2.follow(leftMaster);
    rightSlave1.follow(rightMaster);
    rightSlave2.follow(rightMaster);

    leftMaster.setOpenLoopRampRate(0.33);
    rightMaster.setOpenLoopRampRate(0.33);
  }
  
  public void shift(boolean toHighGear) {
    if(toHighGear){
      gearRatio = HIGH_GEAR_RATIO;
      isHighGear = true;
      shifter.set(DoubleSolenoid.Value.kReverse);
    }
    else{
      gearRatio = LOW_GEAR_RATIO;
      isHighGear = false;
      shifter.set(DoubleSolenoid.Value.kForward);
    }
  }



  public void autoShift() {
    DifferentialDriveWheelSpeeds speeds = getSpeeds();
    double leftSpeed = Math.abs(Units.metersToFeet(speeds.leftMetersPerSecond));
    double rightSpeed = Math.abs(Units.metersToFeet(speeds.rightMetersPerSecond));
    
    boolean leftIsNegative = Units.metersToFeet(speeds.leftMetersPerSecond)/leftSpeed == -1;
    boolean rightIsNegative = Units.metersToFeet(speeds.rightMetersPerSecond)/rightSpeed == -1;
    double averageSpeed;
    if((leftIsNegative && rightIsNegative) || (!leftIsNegative && !rightIsNegative)){
      averageSpeed = (leftSpeed + rightSpeed) / 2;
    }else{
      averageSpeed = Math.abs((leftSpeed - rightSpeed) / 2);
    }
    if(averageSpeed > SHIFT_TO_HIGH_THRESHOLD) {
      shift(true);
    }
    else if(averageSpeed < SHIFT_TO_LOW_THRESHOLD) {
      shift(false);
    }
  }

  public void cheesyDrive(double throttle, double turn, boolean isQuickTurn){
    DriveSignal signal = cheesyDriveHelper.cheesyDrive(throttle, turn, isQuickTurn, isHighGear);
    setOutput(signal.getLeft() * OUTPUT_VOLTS, OUTPUT_VOLTS * signal.getRight());
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getEncoder().getVelocity() / gearRatio * 2 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS) / 60,
        rightMaster.getEncoder().getVelocity() / gearRatio * 2 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS) / 60
    );
  }

  public SimpleMotorFeedforward getFeedForward() {
      return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return currentPose;
  }

  public boolean getHighGear(){
    return this.isHighGear;
  }

  public void setOutput(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / OUTPUT_VOLTS);
    rightMaster.set(rightVolts / OUTPUT_VOLTS);
  }

  public void tankDrive(double left, double right) {
    leftMaster.set(left);
    rightMaster.set(right);
  }


  public PIDController getleftPIDController() {
      return leftPIDController;
  }

  public PIDController getrightPIDController() {
      return rightPIDController;
  }

  public PIDController getLeftPIDCommandController() {
      return leftPIDCommandController;
  }

  public PIDController getRightPIDCommandController() {
      return rightPIDCommandController;
  }

  public void resetEncoders(){
    leftMaster.getEncoder().setPosition(0);
    rightMaster.getEncoder().setPosition(0);
  }

  public double getLeftEncoderValue(){
    return leftMaster.getEncoder().getPosition();
  }
  public double getRightEncoderValue(){
    return rightMaster.getEncoder().getPosition();
  }

  public double getAngle(){
    return navx.getAngle();
  }

  double curYaw;
  double prevYaw;
  double diffYaw;
  double totalError;
  static final double kP = -1.5/25.0;
  static final double kI = -0.0;
  static final double kD = -0;

  public void turnAnglePID(double targetAngle){
        prevYaw = curYaw;
        curYaw = targetAngle - getAngle();
        diffYaw = curYaw - prevYaw;
        totalError += diffYaw;
        if(Math.abs(curYaw) < .25)
            totalError = 0;
        double steering = curYaw * kP + diffYaw * kD + totalError * kI;
        cheesyDrive(0, steering, true);
  }

  @Override
  public void periodic(){
    currentPose = odometry.update(getHeading(), getSpeeds().leftMetersPerSecond, getSpeeds().rightMetersPerSecond);
    SmartDashboard.putBoolean("High Gear", isHighGear);
    
    //autoShift();
  }

}


