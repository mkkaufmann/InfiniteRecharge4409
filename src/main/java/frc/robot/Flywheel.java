package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


class Flywheel {

    static final int MAX_MOTOR_RPM = 18370;
    static final double GEARBOX_RATIO = 3.25;
    static final int ENCODER_TICKS_PER_MOTOR_REVOLUTION = 12;
    static final double MAX_FLYWHEEL_RPM = MAX_MOTOR_RPM/GEARBOX_RATIO;
    static final double ENCODER_TICKS_PER_FLYWHEEL_REVOLUTION = ENCODER_TICKS_PER_MOTOR_REVOLUTION * GEARBOX_RATIO;
    static final double SPEED_TRANSFER_TO_BALL = 0.4103;
    static final double WHEEL_DIAMETER = 6.0;
    static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    TalonSRX flywheelMaster = new TalonSRX(0);
    VictorSPX flywheelFollower = new VictorSPX(1);
    
    public Flywheel(){
        flywheelMaster.setInverted(true);
        flywheelFollower.setInverted(true);
        flywheelFollower.follow(flywheelMaster);
        flywheelMaster.config_kP(0, 40);
        flywheelMaster.config_kF(0, 2.78443113772);
        flywheelMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }

    public void runFlywheel(double output){
        System.out.println("Encoder position" + flywheelMaster.getSelectedSensorPosition());
        flywheelMaster.set(ControlMode.PercentOutput, output);
    }

    private void runVelocity(double velocity){
        System.out.println("Encoder velocity" + (flywheelMaster.getSelectedSensorVelocity()));
        System.out.println("Encoder difference from velocity " + (Math.abs(velocity) - (Math.abs(flywheelMaster.getSelectedSensorVelocity()))));
        flywheelMaster.set(ControlMode.Velocity, velocity);
    }

    private void runRotationsPer100ms(double rotationsPer100ms){
        runVelocity(rotationsPer100ms * ENCODER_TICKS_PER_FLYWHEEL_REVOLUTION);
    }

    public void runRPM(double rpm){
        runRotationsPer100ms(rpmToRotationsPer100ms(rpm));
    }

    public void shootProjectileAtSpeed(double inchesPerSecond){
        runRotationsPer100ms(getDesiredSurfaceSpeed(inchesPerSecondToRotationsPer100ms(inchesPerSecond)));
    }

    private double rpmToRotationsPer100ms(double rpm){
        return rpm/600;
    }

    private double inchesToRotations(double inches){
        return inches/WHEEL_CIRCUMFERENCE;
    }

    private double inchesPerSecondToRotationsPer100ms(double inchesPerSecond){
        return inchesToRotations(inchesPerSecond)/10;
    }

    private double getDesiredSurfaceSpeed(double projectileSpeed){
        return projectileSpeed/SPEED_TRANSFER_TO_BALL; 
    }
}
