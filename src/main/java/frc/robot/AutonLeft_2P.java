package frc.robot;

import static edu.wpi.first.units.Units.Second;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.driveTrain.ShortcutType;

public class AutonLeft_2P {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    

    public AutonLeft_2P(driveTrain _driveTrain, limeLightTest _limelightcam){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;

        

    }

    Object[] Run_2P(Object[] autoState) {
        String currentState = autoState[0].toString();
        
        System.out.println("current state: " + currentState);

        switch(currentState){
            
            case "Starting":
                driveTrain.drive( 0, 0, 0, false, false);
                time = 0;
                currentState = "Move'nToReef";
            break;

            case "Move'nToReef":
            var armPosition = driveTrain.armMotor.getEncoder().getPosition();
            double armMotorSpeed = 0;
            var liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();
            double liftMotorSpeed = 0;

            if(time > 0 && time <= 290) {
                //???Could this work???
                //driveTrain.ShortCut(ShortcutType.L4);
                
                if(armPosition >= 0 && armPosition < 330.42) {
                    armMotorSpeed = 1;
                }
                driveTrain.armMotor.set(armMotorSpeed);

                if(liftMotorPosition >= 0 && liftMotorPosition < 500.6) {
                    liftMotorSpeed = 1;
                }
                driveTrain.liftMotor.set(liftMotorSpeed);
            }

            if(time > 10 && time <= 75) {
                driveTrain.drive(-0.6, 0, 0, false, false);
            }
            if(time > 75 && time <= 95) {
                driveTrain.drive( 0, 0, 1, false, false);
            }
            if(time > 95 && time <= 135) {
                driveTrain.drive(-0.5, 0.5, 0, false, false);
            }
            if (time > 135 && time <= 290) {
                driveTrain.drive(0, 0, 0, false, false);
                driveTrain.armMotor.set(0);
           }
           if (time > 290) {
                driveTrain.armMotor.set(0);
                driveTrain.liftMotor.set(0);

                time = 0;
                currentState = "Score";    
            }
             break;

            case "Score": 

                liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();

                if (time > 10 && time <= 30) {
                    driveTrain.outTakeSet(-0.3);
                } 
                if(time > 30) {
                    driveTrain.outTakeSet(0);

                    time = 0;
                    currentState = "Adjust'n111111111";    
                }
            break;


            case "Adjust'n":
            if (time > 10) {
                driveTrain.drive(0.3, 0, 0, false, false);
            }
            if (time > 15) {
                driveTrain.drive(0, 0, 0, false, false);
            }
            time = 0;
            /*currentState - "ToPlayerStation" */
            break;
            
            case "ToPlayerStation":
            if (time > 10) {
                driveTrain.drive(-0.3, 0, 0, false, false);
            }
            if (time > 35) {
                driveTrain.drive(-0.3, 0 , -0.5 , false, false);
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
                driveTrain.drive(0.3, 0, 0, false, false);
            }
            if (time > 25) {
                driveTrain.drive(0.3, 0, -0.35, false, false);
            }
            if (time > 50) {
                driveTrain.drive(0.5 , 0, 0 , false, false);
            }
            if (time > 85) {
                driveTrain.drive(0, 0, 0, false, false);
            }

            time = 0;
            break;
            
             
            
        }
        time++;
        return new Object[]{currentState, time};
    }

    @SuppressWarnings("unused")
    Object[] Run_2P1(Object[] autoState) {
        String currentState = autoState[0].toString();
        
        System.out.println("current state: " + currentState);

        switch(currentState){
            
            case "Starting":
            driveTrain.resetGyro();
            driveTrain.drive( 0, 0, 0, false, false);
            time = 0;
            currentState = "Move'nToReef";
            break;

            case "Move'nToReef":

            /*if (April_22 == null) {
                driveTrain.drive(0, .3, 0, false);
            }
            if (time > 50){
                driveTrain.drive(0, 0, 0, false);
            }*/
            
            if(time > 10 && time <= 70) {
                driveTrain.drive(-0.6, 0, 0, false, false);
            }
            if(time > 70 && time <= 80) {
                driveTrain.drive( 0, 0, 1, false, false);
            }
            // if(time > 70 && time <= 90) {
            //     driveTrain.drive(-0.6, 0, 0, false, false);
            // }
            if (time > 90) {
                 driveTrain.drive(0, 0, 0, false, false);
                 time = 0;
                 currentState = "April_22Turn";    
            }
            break;

            case "April_22Turn":       
            var April_22 = driveTrain.GetAprilTagTelemotry(22);
  
    
            if (April_22 != null){
               
                double trackSide = limelightcam.LimeTest();
                double trackTurn = limelightcam.ReefCenter(); 
                double trackPush = limelightcam.ReefPush();
    
                System.out.println("ts: " + trackSide + " tt: " + trackTurn + " tp: " + trackPush);
                
                var txnc_22 = April_22.txnc;
                var ta_22 = April_22.ta;

                if (time > 25){
                    if (txnc_22 != 0){
                        driveTrain.drive(-1*trackPush, 0, 0, false, false);
                    }
                    else{
                        driveTrain.drive(0, 0, 0, false, false);
                    }
                }
            } 

            time = 0;
            currentState = "Adjust'n";
            break;

            case "Adjust'n":
            if (time > 10) {
                driveTrain.drive(0.3, 0, 0, false, false);
            }
            if (time > 15) {
                driveTrain.drive(0, 0, 0, false, false);
            }
            time = 0;
            currentState = "ToPlayerStation";
            break;
            
            case "ToPlayerStation":
            if (time > 10) {
                driveTrain.drive(-0.3, 0, 0, false, false);
            }
            if (time > 35) {
                driveTrain.drive(-0.3, 0 , -0.5 , false, false);
            }
            if (time > 45) {
                driveTrain.drive(-0.5 , 0, 0 , false, false);
            }  
            if (time > 90) {
                driveTrain.drive(0, 0, 0, false, false);
            }
            time = 0;
            currentState = "ToReef2"; 
            break;
            
            case "ToReef2":
            if (time > 10) {
                driveTrain.drive(0.3, 0, 0, false, false);
            }
            if (time > 25) {
                driveTrain.drive(0.3, 0, -0.35, false, false);
            }
            if (time > 50) {
                driveTrain.drive(0.5 , 0, 0 , false, false);
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
