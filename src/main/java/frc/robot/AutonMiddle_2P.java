package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.driveTrain.ShortcutType;

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

    Object[] Run_2P(Object[] autoState, double getPeriod) {
        String currentState = autoState[0].toString();

        var currentTargetId = LimelightHelpers.getFiducialID("limelight");

        var rangeOutput = limelightcam.LLGetRangeOutput();
        double rotationOutput;

        var April_22 = driveTrain.GetAprilTagTelemotry(22);
        var April_10 = driveTrain.GetAprilTagTelemotry(10);

        var liftMotorPosition = driveTrain.potLift.get();
        var liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L1);
        var armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L1);

        System.out.println("cs: " + currentState);

        switch(currentState){

            case "Starting":
               if (time > 0) {
                    driveTrain.drive(0, 0, 0, false, false);
                    driveTrain.resetGyro();
                    time = 0;
                    currentState = "firstForward";
               }
            break;

            case "firstForward":
                if (time > 0 && time < 70){
                    driveTrain.driveLL(-0.5, 0, 0, false, getPeriod);
                } else {
                    driveTrain.drive(0, 0, 0, false, false);
                    time = 0;
                    currentState = "rotat";
                }
            break;

            case "rotat":
                if (driveTrain.gyro.getYaw() <= 57 || driveTrain.gyro.getYaw() >= 63){
                    rotationOutput = driveTrain.getTurnBotToAngle(60);
                    driveTrain.driveLL(0, 0, rotationOutput, false, getPeriod);
                } else if (driveTrain.gyro.getYaw() >= 57 && driveTrain.gyro.getYaw() <= 63){
                    time = 0;
                    currentState = "Drive'nForward";
                }
            break;

            case "lift":
             
                liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L1);
                armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L1);

                if (time > 1 && time < 137){
                    driveTrain.drive(0, 0, 0, false, false);
                    driveTrain.armMotor.set(armMotorSpeed);
                    driveTrain.liftMotor.set(liftMotorSpeed);
                }
                if (liftMotorPosition > 25){
                    driveTrain.drive(0, 0, 0, false, false);
                    driveTrain.outTakeSet(0);
                    driveTrain.armMotor.set(0);
                    driveTrain.liftMotor.set(0);
                    currentState = "Drive'nForward"; 
                    time = 0;
                }
            break;
                

            case "Drive'nForward":                
                if (April_22 != null){
                    rotationOutput = driveTrain.getTurnBotToAngle(60);
                    if (time < 100){
                        driveTrain.driveLL(rangeOutput, 0, rotationOutput, false, getPeriod);
                    }
                } else if (April_22 == null && time > 150){
                    driveTrain.drive(0, 0, 0, false, false);
                    time = 0;
                    currentState = "adjust1";
                }
            break;

            case "adjust1":
                rotationOutput = driveTrain.getTurnBotToAngle(60);

                if (time < 50){
                    driveTrain.driveLL(-0.4, 0, rotationOutput, false, getPeriod);
                } else {
                    driveTrain.drive(0, 0, 0, false, false);
                    time = 0;
                    currentState = "turnplayer";
                }

            break;

            case "score": 
                if (time > 0 && time < 30){
                    driveTrain.outTakeSet(0.7); 
                } else if (hasPiece() == false) {
                    driveTrain.outTakeSet(0); 
                    time = 0;
                    currentState = "turnplayer";
                }
            break;
            
            case "turnplayer":
                if (time > 0 && time < 75 ){
                    driveTrain.driveLL(0.5, 0, 0, false, getPeriod);
                } 
                if (time > 75){
                    if (driveTrain.gyro.getYaw() <= -22 || driveTrain.gyro.getYaw() >= -18){
                        rotationOutput = driveTrain.getTurnBotToAngle(-20);
                        driveTrain.driveLL(0, 0, rotationOutput, false, getPeriod);
                    } else if (driveTrain.gyro.getYaw() >= -22 && driveTrain.gyro.getYaw() <= -18){
                        time = 0;
                        currentState = "PlayerStation";
                    }  
                }
                
            break;

            case "PlayerStation":
                var April_12 = driveTrain.GetAprilTagTelemotry(12);
                
                if (April_12 != null){
                    if (currentTargetId == 12){
                        rotationOutput = driveTrain.getTurnBotToAngle(-50);
                        driveTrain.driveLL(rangeOutput*.5, 0, rotationOutput, false, getPeriod);
                    }
                } else if (April_12 == null && time > 150){
                    driveTrain.driveLL(0, 0, 0, false, getPeriod);
                    time = 0;
                    currentState = "adjust2";
                }
            break;

            case "adjust2":
                if (time > 0 && time < 70){
                    driveTrain.driveLL(-0.3, 0, 0, false, getPeriod);
                } else if (time > 75){
                    driveTrain.driveLL(0, 0, 0, false, getPeriod);
                    //time = 0;
                    //currentState = "";
                }
            break;









            /*case "Score":
                
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
        */
        }
        time++;
        return new Object[]{currentState, time};
    }

}