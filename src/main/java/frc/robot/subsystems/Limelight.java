package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{

    static final double kP = -1/25.0;
    static final double kI = -0.01;
    static final double kD = -0.15;
    static final double goalHeightMid = 89;
    static final double mountHeight = 25.5;
    static final double heightDiff = goalHeightMid-mountHeight;
    static final double mountAngle = Math.toRadians(-1.402);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double curYaw;
    double prevYaw;
    double diffYaw;
    double totalError;

    @Override
    public void periodic(){
        //System.out.println("Valid target: " + tv.getDouble(0.0));
        System.out.println("Horizontal offset: " + tx.getDouble(0.00123));
        System.out.println("Distance est: " + heightDiff/(Math.tan(Math.toRadians(ty.getDouble(0.0)))+mountAngle));
        //System.out.println("Vertical offset: " + ty.getDouble(0.0));
        //System.out.println("Target area: " + ta.getDouble(0.0));
        
    }

    public void run(){
        table.getEntry("ledMode").setNumber(3);
    }

    public void stop(){
        table.getEntry("ledMode").setNumber(1);
    }

    public void getLLdistance(){
        System.out.println("Vertical offset: " + ty.getDouble(0.0));
        System.out.println("Distance est: " + heightDiff/(Math.tan(Math.toRadians(ty.getDouble(0.0)))+mountAngle));
    }

    public void autoAim(Drivetrain drivetrain, double drive){
        run();
        prevYaw = curYaw;
        curYaw = tx.getDouble(0.00123);
        diffYaw = curYaw - prevYaw;
        totalError += diffYaw;
        if(Math.abs(curYaw) < .25)
            totalError = 0;
        double steering = tx.getDouble(0.00123) * kP + diffYaw * kD + totalError * kI;
        if(drive != 0){
            drivetrain.cheesyDrive(drive, -steering, false);
        }else{
            drivetrain.cheesyDrive(drive, -steering, true);
        }
    }

}
