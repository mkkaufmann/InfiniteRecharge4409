package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.util.Units;


class Drivetrain extends SubsystemBase {
    
  CANSparkMax leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave1 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave2 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave2 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

  AHRS navx = new AHRS(SPI.Port.kMXP);
  Pose2d currentPose = new Pose2d();

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());
  
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
  
  @Override
  public void periodic(){
//    pose = odometry.update(getHeading());
  }
}
