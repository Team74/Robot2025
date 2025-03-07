package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class AutonMiddle_2P {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;

    boolean hasPiece() {
        return !driveTrain.proxSensor.get();
    }
    

    public AutonMiddle_2P(driveTrain _driveTrain, limeLightTest _limelightcam){
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
        var armPosition = driveTrain.armMotor.getEncoder().getPosition();
        double armMotorSpeed = 0;
        var liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();
        double liftMotorSpeed = 0;

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

               

                if (time > 10 && time < 35){
                    driveTrain.drive(-0.5, 0, 0, false, false);
                } 
                if (time > 40){
                    driveTrain.drive(0, 0, 0, false, false);
                    time = 0;
                    currentState = "Score";
                }
            
            break;
        
            case "Score":
                
            if (time > 10){
                if(liftMotorPosition >= 0 && liftMotorPosition < 15) {
                    liftMotorSpeed = 1;
                }
                driveTrain.liftMotor.set(liftMotorSpeed);

                if(armPosition >= 0 && armPosition < 346.59) {
                    armMotorSpeed = 1;
                }
                driveTrain.armMotor.set(armMotorSpeed);
            }

            if (liftMotorPosition >= 15 && armPosition >= 344.59){
                driveTrain.outTakeSet(-1);
            }

            if (hasPiece() != true && time >= 300){
                driveTrain.outTakeSet(0);
                driveTrain.armMotor.set(0);
                driveTrain.liftMotor.set(0);
                time = 0;
            }
            currentState = "Adjust'n"; 
            break;


            case "Adjust'n":
            if (time > 0) {
                driveTrain.drive(0.3, 0, 0, false, false);
            }
            if (time > 15) {
                driveTrain.drive(0, 0, 0, false, false);
                time = 0;
                currentState = "ToPlayerStation";
            }
            break;
            
            case "ToPlayerStation":
            if (time > 0 && time < 15) {
                driveTrain.drive(0, 0, -0.3, false, false);
            } 
            if (time > 15 && time < 35) {
                driveTrain.drive(-0.3, -0.3 , 0 , false, false);
            } 
            if (time > 35 && time < 50) {
                driveTrain.drive(0 , 0, 0.3 , false, false);
            } 
            if (time > 50 && time < 100) {
                driveTrain.drive(-0.3, 0, 0, false, false);
            } 
            if (time > 100){
                driveTrain.drive(0, 0, 0, false, false);
                time = 0;
                currentState = "intake";
            }
             
            break;
            
            case "intake":
            if (time > 0 && time < 100) {
                if(liftMotorPosition >= 0 && liftMotorPosition < 265.957) {
                    liftMotorSpeed = 1;
                }
                driveTrain.liftMotor.set(liftMotorSpeed);

                if(armPosition >= 0 && armPosition < 425.0636) {
                    armMotorSpeed = 1;
                }
                driveTrain.armMotor.set(armMotorSpeed);
                time = 0;
            } 
            if (time > 100 && time < 110){
                driveTrain.outTakeSet(.5);
            } 
            if (time > 110 && hasPiece() == true && time < 111){
                driveTrain.outTakeSet(0);
                time = 0;
                //currentState = "DriveBack";
            }
           break;
        }
        time++;
        return new Object[]{currentState, time};
    }

}