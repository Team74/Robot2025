package frc.robot;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import frc.robot.LimelightHelpers.RawFiducial;

public class Auton_2P {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    

    public Auton_2P(driveTrain _driveTrain, limeLightTest _limelightcam){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;

        

    }

    Object[] Run_2P(Object[] autoState) {
        double trackSide = limelightcam.LimeTest();
        double trackTurn = limelightcam.ReefCenter(); 
        double trackPush = limelightcam.ReefPush();
        String currentState = autoState[0].toString();

        switch(currentState){
            case "Starting":
            driveTrain.resetGyro();
            driveTrain.drive( 0, 0, 0, false);
            time = 0;
            currentState = "Move'nToReef";
            break;

            case "Move'nToReef":
            
            if(time > 0 && time <= 90) {
                driveTrain.drive(0, .6, 0, false);
            }
            if(time > 90 && time <= 110) {
                driveTrain.drive( 0, 0, -1, false);
            }
            if(time > 110 && time <= 135) {
                driveTrain.drive(0, .6, 0, false);
            }
            if (time > 135) {
                driveTrain.drive(0, 0, 0, false);
            }
            time = 0;
            currentState = "April_22Turn";
            break;

            case "April_22Turn":
            
                var April_22 = driveTrain.GetAprilTagTelemotry(22);
                
                if (April_22 != null){
                    var txnc_22 = April_22.txnc;
                    var ta_22 = April_22.ta;

                    if (txnc_22 != 0){
                        driveTrain.drive(-1*trackPush, -1*trackSide, -1*trackTurn, false);
                    }
                    if (ta_22 != 55){
                        driveTrain.drive(-1*trackPush, -1*trackSide, -1*trackTurn, false);
                    } else{
                        driveTrain.drive(0, 0, 0, false);
                    }
                } 

            time = 0;
            currentState = "Adjust'n";
            break;

            case "Adjust'n":
            if (time > 0) {
                driveTrain.drive(0, 0.3, 0, false);
            }
            if (time > 10) {
                driveTrain.drive(0, 0, 0, false);
            }
            time = 0;
            break;
                
             
            
        }
        time++;
        return new Object[]{currentState, time};
    }
}
