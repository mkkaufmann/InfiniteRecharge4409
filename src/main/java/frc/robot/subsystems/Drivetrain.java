package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;


class Drivetrain extends SubsystemBase {
    
  CANSparkMax leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(30);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

  public Drivetrain(){
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
  }
  
  @Override
  public void periodic(){

  }
}
