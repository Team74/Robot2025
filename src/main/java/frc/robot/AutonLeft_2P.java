package frc.robot;

import static edu.wpi.first.units.Units.Second;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.driveTrain.ShortcutType;
import frc.robot.limeLightTest;

public class AutonLeft_2P {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    boolean hasPeice;
    
    public AutonLeft_2P(driveTrain _driveTrain, limeLightTest _limelightcam, boolean _hasPeice){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
        hasPeice = _hasPeice;

    }

    Object[] Run_2P(Object[] autoState, double getPeriod) {
        
        String currentState = autoState[0].toString();
        var rangeOutput = limelightcam.LLGetRangeOutput();
        var rotationOutput = limelightcam.LLGetRotation();
        var currentTargetId = LimelightHelpers.getFiducialID("limelight");

        double armMotorSpeed = 0;
        double liftMotorSpeed = 0;

        switch(currentState){
            
            case "Starting":
                driveTrain.drive( 0, 0, 0, false, false);
                time = 0;
                currentState = "Move'nToReef";
            break;

            case "Move'nToReef":
            
            if(time > 0 && time <= 150) {
                driveTrain.ShortCutArm(ShortcutType.L1);
                driveTrain.ShortCutLift(ShortcutType.L1);
                driveTrain.armMotor.set(armMotorSpeed);
                driveTrain.liftMotor.set(liftMotorSpeed);  
            }
            if (time > 150 && time < 151){
                driveTrain.armMotor.set(0);
                driveTrain.liftMotor.set(0);
            }

            if(time > 10 && time <= 75) {
                driveTrain.drive(-0.6, 0, 0, false, false);
            }
            if(time > 75 && time <= 95) {
                driveTrain.drive( 0, 0, 1, false, false);
            }
            if(time > 95 && time <= 135) {
                driveTrain.drive(-0.5, 0.7, 0, false, false);
            }
            if (time > 135 && time <= 290) {
                driveTrain.drive(0, 0, 0, false, false);
                time = 0;
                currentState = "adjust";
           }
           
                   
             break;
                
            case "adjust": 
                var April_22 = driveTrain.GetAprilTagTelemotry(22);
                var April_9 = driveTrain.GetAprilTagTelemotry(9);

            if (April_22 != null){
                
                if (currentTargetId == 22){
                    driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    if(time < 100){
                        driveTrain.outTakeMotorInner.set(-.5);
                        if (time > 150){
                            driveTrain.outTakeMotorInner.set(0);
                            time = 0;
                            currentState = "ToPlayerStation";
                        }
                        
                    }
                }
                
            }

            if (April_9 != null){
                
                if (currentTargetId == 9){
                    driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    if(time < 100){
                        driveTrain.outTakeMotorInner.set(-.5);
                        if (time > 150){
                            driveTrain.outTakeMotorInner.set(0);
                            time = 0;
                            currentState = "Score";
                        }
                        
                    }
                }
                
            }
            break;
            
            case "Score":
            if (time > 10 && time < 35) {
                driveTrain.drive(-0.3, 0, 0, false, false);
            }
            if (time > 35 && time < 45) {
                driveTrain.drive(-0.3, 0 , -0.5 , false, false);
            }
            if (time > 45 && time < 90) {
                driveTrain.drive(0 , -.5, 0 , false, false);
            }  
            if (time > 90) {
                driveTrain.drive(0, 0, 0, false, false);
                time = 0;
                currentState = "Adjust2"; 
            }
            
            break;
            
            case "Adjust2":
            April_22 = driveTrain.GetAprilTagTelemotry(22);
            April_9 = driveTrain.GetAprilTagTelemotry(9);

            if (April_22 != null){
                
                if (currentTargetId == 22){
                    driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    if(time < 100){
                        time = 0;
                        currentState = "ToPlayerStation";
                    }
                }
                
            }

            if (April_9 != null){
                
                if (currentTargetId == 9){
                    driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    if(time < 100){
                        time = 0;
                        currentState = "ToPlayerStation";
                    }
                }
                
            }
            break;

            case "ToPlayerStation":

            var April_2 = driveTrain.GetAprilTagTelemotry(22);
            var April_12 = driveTrain.GetAprilTagTelemotry(9);

            if (time > 0 && time > 20){
                driveTrain.drive(0, 0, 0.4, false, false);
            }

            
            if (April_2 != null){
                
                if (currentTargetId == 2){
                    driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    if(time < 100){ 
                        time = 0;
                        currentState = "intake";  
                    }
                }
                
            }

            if (April_12 != null){
                
                if (currentTargetId == 12){
                    driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    if(time < 100){
                        time = 0;
                        currentState = "intake";        
                    }
                }
                
            }
            break;

            case "intake":

                if (time > 0 && time < 150){
                    driveTrain.outTakeMotorInner.set(-0.5);
                } else if (hasPeice == true){
                    driveTrain.outTakeMotorInner.set(0);
                } else if (time > 200){
                    driveTrain.outTakeMotorInner.set(0);
                    time = 0;
                }
            


            
             
            
        }
        time++;
        return new Object[]{currentState, time};
    }
}



    /* 
    @SuppressWarnings("unused")
    Object[] Run_2P1(Object[] autoState, double currPeriod) {
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

            if (April_22 == null) {
                driveTrain.drive(0, .3, 0, false);
            }
            if (time > 50){
                driveTrain.drive(0, 0, 0, false);
            }
            
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
                final var rot_limelight = limelightcam.limelight_aim_proportional(0.3);
                var rot = rot_limelight;

                final var forward_limelight = limelightcam.limelight_range_proportional(0.3);
                var xSpeed = forward_limelight;
                var ySpeed = 0.0;

                //while using Limelight, turn off field-relative driving.
                var fieldRelative = false;
                
                
                // var txnc_22 = April_22.txnc;
                // var ta_22 = April_22.ta;

                if(time > 0 && time <= 25) {
                    driveTrain.driveLL(xSpeed, ySpeed, rot, fieldRelative, currPeriod);

                    System.out.println("xSpeed: " + xSpeed + " ySpeed: " + ySpeed + " rot: " + rot);
                    // if (txnc_22 != 0){
                    //     driveTrain.drive(-1*trackPush, 0, 0, false, false);
                    // }
                    // else{
                    //     driveTrain.drive(0, 0, 0, false, false);
                    // }
                }
                if (time > 25) {
                    driveTrain.drive(0, 0, 0, false, false);

                    time = 0;
                    currentState = "Adjust'n";
                }
   
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
*/