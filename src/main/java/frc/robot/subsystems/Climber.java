package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

class Climber extends SubsystemBase{
    TalonSRX master = new TalonSRX(420);
    VictorSPX slave = new VictorSPX(2);
    public Climber(){
        slave.follow(master);
    }
    public void up(){
        master.set(ControlMode.PercentOutput, 0.75);
    }
    public void down(){
        master.set(ControlMode.PercentOutput, -0.75);
    }
    public void stop(){
        master.set(ControlMode.PercentOutput, 0);
    }
}