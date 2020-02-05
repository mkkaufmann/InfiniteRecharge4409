package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Limelight extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    @Override
    public void periodic(){
        System.out.println("Valid target: " + tv);
        System.out.println("Horizontal offset: " + tx);
        System.out.println("Vertical offset: " + ty);
        System.out.println("Target area: " + ta);
        table.getEntry("ledMode").setNumber(3);
        
    }

}