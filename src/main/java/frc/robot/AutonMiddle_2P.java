package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutonMiddle_2P {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    

    public AutonMiddle_2P(driveTrain _driveTrain, limeLightTest _limelightcam){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
    }

    Object[] Run_2P(Object[] autoState) {
        double trackSide = limelightcam.LimeTest();
        double trackTurn = limelightcam.ReefCenter(); 
        double trackPush = limelightcam.ReefPush();
        String currentState = autoState[0].toString();
        var April_22 = driveTrain.GetAprilTagTelemotry(22);
        DriverStation.Alliance redcolor = Alliance.Red;
        DriverStation.Alliance bluecolor = Alliance.Blue;

        switch(currentState){

            case "Starting":
               if (time > 0) {
                    driveTrain.drive(0, 0, 0, false, false);
                    driveTrain.resetGyro();
                    time = 0;
                    currentState = "Drive'nApril_22";
               }
            break;

            case "Drive'nApril_22":
            
                if (time > 10){
                    driveTrain.drive(0, 0.5, 0, false, false);  
                }
                if (time > 50){
                driveTrain.drive(0, 0, -0.5, false, false);
                }
                if (time > 55){
                    driveTrain.drive(0, 0, 0, false, false);
                }
                time = 0;
                currentState = "April_22Align";

            case "April_22Align":

            if(April_22 != null){

                var txnc_22 = April_22.txnc;
                var ta_22 = April_22.ta;

                if (txnc_22 != 0){
                    driveTrain.drive(-trackSide, -trackTurn, -trackPush, false, false);
                }
                if (ta_22 != 55){
                    driveTrain.drive(-trackSide, -trackTurn, -trackPush, false, false);
                } else {
                    driveTrain.drive(0, 0, 0, false, false);
                }

                time = 0;
                currentState = "ToPlayerStation";
                break;
            } 
        
            case "ToPlayerStation":
            if (time > 10) {
                driveTrain.drive(0, -0.3, 0, false, false);
            }
            if (time > 35) {
                driveTrain.drive(0, -.3 , -0.5 , false, false);
            }
            if (time > 45) {
                driveTrain.drive(0 , -.5, 0 , false, false);
            }  
            if (time > 90) {
                driveTrain.drive(0, 0, 0, false, false);
            }
            time = 0;
            currentState = "ToReef2"; 
            break;
            
            case "ToReef2":
            if (time > 10) {
                driveTrain.drive(0, 0.3, 0, false, false);
            }
            if (time > 25) {
                driveTrain.drive(0, 0.3 , -0.35 , false, false);
            }
            if (time > 50) {
                driveTrain.drive(0 , .5, 0 , false, false);
            }
            if (time > 75) {
                driveTrain.drive(0, 0, 0, false, false);
            }

            time = 0;
            break;
            

        }
        
        time++;
        return new Object[]{currentState, time};
    }

}
