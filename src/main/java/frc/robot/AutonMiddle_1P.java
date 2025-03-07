package frc.robot;

public class AutonMiddle_1P {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    

    public AutonMiddle_1P(driveTrain _driveTrain, limeLightTest _limelightcam){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
    }

    Object[] Run_2P(Object[] autoState) {
        double trackSide = limelightcam.LimeTest();
        double trackTurn = limelightcam.ReefCenter(); 
        double trackPush = limelightcam.ReefPush();
        String currentState = autoState[0].toString();
        var April_21 = driveTrain.GetAprilTagTelemotry(21);
        var April_12 = driveTrain.GetAprilTagTelemotry(12);

        switch(currentState){

            case "Starting":
               if (time > 0) {
                    driveTrain.drive(0, 0, 0, false, false);
                    driveTrain.resetGyro();
                    time = 0;
                    currentState = "Drive'nApril_21";
               }
            break;

            case "Drive'nApril_21":

                var txnc_21 = April_21.txnc;
                var ta_21 = April_21.ta;

                if (time > 10 && time < 25){
                    driveTrain.drive(-0.5, 0, 0, false, false);
                }

                time = 0;
                currentState = "TurnToStation";
            break;
        
            case "TurnToStation":
                if (time > 10) {
                    driveTrain.drive(0, -0.3, 0, false, false);
                }
                if (time > 35) {
                    driveTrain.drive(0, 0, 0.5 , false, false);
                }
                if (time > 45) {
                    driveTrain.drive(0 , 0.5, 0 , false, false);
                }  
                if (time > 90) {
                    driveTrain.drive(0, 0, 0, false, false);
                }
                time = 0;
                currentState = "DriveToApril_12"; 
            break;

            case "DriveToApril_12":

                var txnc_12 = April_12.txnc;
                var ta_12 = April_12.ta;
                
                if (April_12 != null && time > 10){

                    if (txnc_12 != 0){
                        driveTrain.drive(trackSide, trackTurn, trackPush, false, false);
                    }
                    if (ta_12 != 55){
                        driveTrain.drive(trackSide, trackTurn, trackPush, false, false);
                    } else {
                        driveTrain.drive(0, 0, 0, false, false);
                    }
                }
                if (time > 75){
                    driveTrain.outTakeSet(0.5);
                }
                /*if (HasPiece == true){
                    driveTrain.outTakeSet(0);
                }*/
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

