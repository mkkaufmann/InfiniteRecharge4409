package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
  private static final double SHIFT_TO_HIGH_THRESHOLD = 5; //fps
  boolean isHighGear = false;

  CANSparkMax leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave1 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave2 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave2 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  DoubleSolenoid shifter = new DoubleSolenoid(1, 2);


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

  Pose2d pose;
  
  PIDController leftController = new PIDController(0,0,0);
  PIDController rightController = new PIDController(0,0,0);

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

  public Drivetrain(){
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftMaster.setSmartCurrentLimit(25);
    rightMaster.setSmartCurrentLimit(25);

    leftSlave1.setSmartCurrentLimit(25);
    leftSlave2.setSmartCurrentLimit(25);
    rightSlave1.setSmartCurrentLimit(25);
    rightSlave2.setSmartCurrentLimit(25);

    leftSlave1.follow(leftMaster);
    leftSlave2.follow(leftMaster);
    rightSlave1.follow(rightMaster);
    rightSlave2.follow(rightMaster);
  }
  
  public void shift(boolean toHighGear) {
    if(toHighGear){
      gearRatio = HIGH_GEAR_RATIO;
      isHighGear = true;
      shifter.set(DoubleSolenoid.Value.kForward);
    }
    else{
      gearRatio = LOW_GEAR_RATIO;
      isHighGear = false;
      shifter.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void autoShift() {
    DifferentialDriveWheelSpeeds speeds = getSpeeds();
    double leftSpeed = Units.metersToFeet(speeds.leftMetersPerSecond);
    double rightSpeed = Units.metersToFeet(speeds.rightMetersPerSecond);
    double averageSpeed = (leftSpeed + rightSpeed) / 2;
    if(averageSpeed > SHIFT_TO_HIGH_THRESHOLD) {
      shift(true);
    }
    else if(averageSpeed < SHIFT_TO_LOW_THRESHOLD) {
      shift(false);
    }
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getEncoder().getVelocity() / gearRatio * 2 * Math.PI * Units.inchesToMeters(2.5) / 60,
        rightMaster.getEncoder().getVelocity() / gearRatio * 2 * Math.PI * Units.inchesToMeters(2.5) / 60
    );
  }

  public SimpleMotorFeedforward getFeedForward() {
      return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setOutput(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }


  public PIDController getleftPIDController() {
      return leftPIDController;
  }

  public PIDController getrightPIDController() {
      return rightPIDController;
  }


  @Override
  public void periodic(){
    pose = odometry.update(getHeading(), getSpeeds().leftMetersPerSecond, getSpeeds().rightMetersPerSecond);
    //autoShift();
  }
}


