package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLightTest {
    AHRS gyro;
    limeLightTest(AHRS gyro1) {
        gyro = gyro1;
    }

    public double LimeTest () {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        

        //read values periodically
        double limex = tx.getDouble(0.0);
        double limey = ty.getDouble(0.0);
        double limearea = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", limex);
        SmartDashboard.putNumber("LimelightY", limey);
        SmartDashboard.putNumber("LimelightArea", limearea);
        System.out.println("tx value: " + limex + "ty value:" + limey + "ta value:" + limearea);
        
        if (limex < -5) {
            System.out.println("Too far Right!!");
            return -0.7;
        }
        if (limex > 5) {
            System.out.println("Too far Left!!");
            return 0.7;
        }
        return 0;
    }
    
    double ReefCenter() {
        double currentAngle = (gyro.getAngle() % 360); 
        int currentTarget =  60 * (int)Math.round((gyro.getAngle() % 360) / 60);

        if (currentAngle > currentTarget + 2) {
            return -0.7; 
        }
        if (currentAngle < currentTarget - 2) {
            return 0.7; 
        }
        return 0; 
   }
}
