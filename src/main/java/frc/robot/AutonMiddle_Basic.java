package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class AutonMiddle_Basic {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;

    boolean hasPiece() {
        return !driveTrain.proxSensor.get();
    }
    

    public AutonMiddle_Basic(driveTrain _driveTrain, limeLightTest _limelightcam){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
    }

    Object[] Run_2P(Object[] autoState) {
        // double trackSide = limelightcam.LimeTest();
        // double trackTurn = limelightcam.ReefCenter(); 
        // double trackPush = limelightcam.ReefPush();
        String currentState = autoState[0].toString();
        // var April_21 = driveTrain.GetAprilTagTelemotry(21);
        // var April_12 = driveTrain.GetAprilTagTelemotry(12);
        var armPosition = 0;//driveTrain.armMotor.getEncoder().getPosition();
        double armMotorSpeed = 0;
        var liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();
        double liftMotorSpeed = 0;
        //System.out.println("cs: " +currentState + " time: " + time + " Armposition: " + armPosition + " liftposition" + liftMotorPosition);
        switch(currentState){

            case "Starting":
               if (time > 0) {
                    driveTrain.drive(0, 0, 0, false, false);
                    driveTrain.resetGyro();
                    time = 0;
                    currentState = "Drive'nForward";
               }
            break;

            case "Drive'nForward":

                //var txnc_21 = April_21.txnc;
                //var ta_21 = April_21.ta;

               

                if (time > 0 && time < 150){
                    driveTrain.drive(-0.3, 0, 0, false, false);
                } 
                if (time > 151){
                    driveTrain.drive(0, 0, 0, false, false);
                }    
                if (time > 1 && time < 151){
                    if(liftMotorPosition >= -0.5 && liftMotorPosition < 15) {
                        liftMotorSpeed = 1;
                    }
                    driveTrain.liftMotor.set(liftMotorSpeed);

                    if(armPosition >= -0.5 && armPosition < 540.9) {
                        armMotorSpeed = 1;
                    } else {
                        armMotorSpeed = 0;
                    }
                    driveTrain.armMotor.set(armMotorSpeed);

                    if (armPosition >= 540.9 && time > 203){
                        driveTrain.armMotor.set(0);
                        driveTrain.liftMotor.set(0);
                        driveTrain.outTakeSet(1);
                        time = 0;
                        currentState = "Score";
                    }
                }
               
            
            break;
        
            case "Score":

            if (time >= 0 && time < 10){
                driveTrain.outTakeSet(0);
                driveTrain.armMotor.set(0);
                driveTrain.liftMotor.set(0);
                time = 0;
                //currentState = "Reset";
            }
             
            break;

            case "Reset":
                
            if (time > 10 && time < 250){
                if(liftMotorPosition >= 0 && liftMotorPosition < 541.6) {
                    liftMotorSpeed = -1;
                }
                driveTrain.liftMotor.set(liftMotorSpeed);

                if(armPosition >= 0 && armPosition < 346.59) {
                    armMotorSpeed = -1;
                }
                driveTrain.armMotor.set(armMotorSpeed);
                time = 0; 
                currentState = "ToReef2";
            }
            
            break;
        }
        time++;
        return new Object[]{currentState, time};
    }

}

