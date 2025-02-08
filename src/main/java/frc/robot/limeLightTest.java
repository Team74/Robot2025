package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLightTest {

    public void LimeTest () {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        

        //read values periodically
        double limex = tx.getDouble(0.0);
        double limey = ty.getDouble(0.0);
        double limearea = ta.getDouble(0.0);

        //post to smart dashboard periodically
        /*SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        //System.out.println("tx value: " + tx.getDouble(limex) + "ty value:" + ty.getDouble(limey) + "ta value:" + ta.getDouble(limearea));
        if (limex < -15) {
            System.out.println("Too far Right!!");
        }
        if (limex > 15) {
            System.out.println("Too far Left!!");
        }
        if (limey > 10) {
            System.out.println("Too far Down!!");
        }*/

        if (limex > 10) {
            
        }



    }

}
