package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//change the naming of these
class Hopper{
    TalonSRX hopperMotor = new TalonSRX(7);
    double wheelDiameter = 6.0;
    int targetRPM = 3000;
    public Hopper(){
        
    }
    public void setSpeed(int RPM){

    }
    public int getSpeed(){
        return 0;
    }
    public void stop(){
        setSpeed(0);
    }
}