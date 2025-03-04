package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLightTest {
    AHRS gyro;
    double limex;
    double limey;
    double limearea;
    double currentTarget;
    PIDController PIDAngle = new PIDController(0.016667*2, 0, 0);
    PIDController PIDPush = new PIDController(0.016667*1.2, 0, 0);
    limeLightTest(AHRS gyro1) {
        gyro = gyro1;
    }

    public double LimeTest () {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        

        //read values periodically
        limex = tx.getDouble(0.0);
        limey = ty.getDouble(0.0);
        limearea = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", limex);
        SmartDashboard.putNumber("LimelightY", limey);
        SmartDashboard.putNumber("LimelightArea", limearea);
       // System.out.println("tx value: " + limex + "ty value:" + limey + "ta value:" + limearea);
        
        if (limex < - (5 + limearea)) {
   
            return -PIDPush.calculate(limex, 0.0);
        }
        if (limex > (5 + limearea)) {
         
            return -PIDPush.calculate(limex, 0.0);
        }
        return 0;
    }
    
    double ReefCenter() {
        currentTarget =  60 * (int)Math.round((gyro.getAngle() % 360) / 60);
        return calculaterotation(currentTarget);
 
    }
    double calculaterotation(Double targetangle){
        double currentAngle = (gyro.getAngle() % 360); 
        //System.out.println("currentAngle: " + currentAngle + " targetangle:" + targetangle);
        
        return PIDAngle.calculate(currentAngle,targetangle);

        // if (currentAngle > targetangle + (2 + limearea)) {
      
        //     return PIDAngle.calculate(currentAngle,currentTarget);  
        // }
        // if (currentAngle < targetangle - (2 + limearea)) {
     
        //     return PIDAngle.calculate(currentAngle,targetangle);
        // }
        // return 0; 
    }
    double ReefPush() {
        if (!TrackCheck(gyro.getAngle(), currentTarget, (5 + limearea))){
            return 0.0;
        }
        if (!TrackCheck(limex, 0, (8 + limearea))){
            return 0.0;
        }
        
        if (limearea > 0 && limearea < 9) {
            if (ReefCenter() == 0 && LimeTest() == 0) {
           
                return -0.5;
            } else {
         
                return -0.2;
            }
        }
        
    return 0;
    }
    boolean TrackCheck(double input, double target,double error) {
    return (input < (target + error) && input > (target - error));
    }
    boolean CanSee(){
        if (limearea == 0.0){
            return false;
        } else {
            return true;
        }
    }
}