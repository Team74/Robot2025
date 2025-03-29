package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.driveTrain.ShortcutType;

public class Auton_1P_SetUp {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    LimelightHelpers Limehelp;

    boolean hasPiece() {
        return !driveTrain.proxSensor.get();
    }
    

    public Auton_1P_SetUp(driveTrain _driveTrain, limeLightTest _limelightcam, LimelightHelpers _Limehelp){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
        Limehelp = _Limehelp;
    }

    Object[] Run_2P(Object[] autoState, double getPeriod) {

        // double trackSide = limelightcam.LimeTest();
        // double trackTurn = limelightcam.ReefCenter(); 
        // double trackPush = limelightcam.ReefPush();

        String currentState = autoState[0].toString();

        //var liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();
        
        var rangeOutput = limelightcam.LLGetRangeOutput();
        var rotationOutput = limelightcam.LLGetRotation();
        var currentTargetId = LimelightHelpers.getFiducialID("limelight");

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
            
            break;
        
            case "Score":
            
            var liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L1);
            var armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L1);

            if (time > 1 && time < 151){
                driveTrain.ShortCutArm(ShortcutType.L1);
                driveTrain.ShortCutLift(ShortcutType.L1);
                driveTrain.armMotor.set(armMotorSpeed);
                driveTrain.liftMotor.set(liftMotorSpeed);
            }

            if (time > 155){
                driveTrain.outTakeSet(0);
                driveTrain.armMotor.set(0);
                driveTrain.liftMotor.set(0);
                time = 0;
                currentState = "ToPlayerStation";
            }
             
            break;

            case "ToPlayerStation":
                
            if (time > 0 && time < 10){
                driveTrain.drive(0, -0.2, .3, false, false);
            }
            if (time > 10 && time < 25){
                driveTrain.drive(-0.4, 0, 0, false, false);
            }
            if (time > 25 && time < 75){
                driveTrain.drive(0, 0.5, 0, false, false);
                time = 0;
                currentState = "Align";
            }
            break;

            case "Align":
            var April_2 = driveTrain.GetAprilTagTelemotry(2);
            var April_12 = driveTrain.GetAprilTagTelemotry(12);
            
   
            if (time > 10){
                if (April_2 != null){
                    
                    if (currentTargetId == 2){
                        driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                        if(time < 100){
                            time = 0;
                            currentState = "BackToReef";
                        }
                    }
                    
                }

                if (April_12 != null){
                    
                    if (currentTargetId == 12){
                        driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                        if(time < 100){
                            time = 0;
                            currentState = "BackToReef";
                        }
                    }
                    
                }
            }
            break;

            case "BackToReef":
            var April_17 = driveTrain.GetAprilTagTelemotry(17);
            var April_8 = driveTrain.GetAprilTagTelemotry(8);
            

            if (time > 0 && time < 25 ){
                driveTrain.drive(0, 0, -0.3, false, false);
            }

            if (April_8 != null){
            
                if (time > 25 && time < 75 && currentTargetId == 8){

                    driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);

                    if (time > 100){
                        time = 0;
                        //currentState = "score";
                    }
                }
            }

            if (April_17 != null){
            
                if (time > 25 && time < 75 && currentTargetId == 17){

                    driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);

                    if (time > 100){
                        time = 0;
                        //currentState = "score";
                    }
                }
            }
            break;
        }
        time++;
        return new Object[]{currentState, time};
    }

}

