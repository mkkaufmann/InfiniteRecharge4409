package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

class Intake{
    TalonSRX intakeMotor = new TalonSRX(7);
    double wheelDiameter = 6.0;
    int targetRPM = 3000;
    public Intake(){
        
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