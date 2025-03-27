package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
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

    Object[] Run_2P(Object[] autoState, double getPeriod) {
        String currentState = autoState[0].toString();

        var rangeOutput = limelightcam.LLGetRangeOutput();
        double rotationOutput;
        var April_22 = driveTrain.GetAprilTagTelemotry(22);
        var April_10 = driveTrain.GetAprilTagTelemotry(10);
        double armMotorSpeed = 0;
        var liftMotorPosition = driveTrain.potLift.get();
        double liftMotorSpeed = 0;

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
                driveTrain.drive(-0.3, 0, 0, false, false);
            } else {
                time = 0;
                currentState = "rotat";
            }
            break;

            case "rotat":
                rotationOutput = driveTrain.getTurnBotToAngle(60);
                driveTrain.driveLL(0, 0, rotationOutput, false, getPeriod);

                if (driveTrain.gyro.getYaw() >= 57 && driveTrain.gyro.getYaw() <= 63){
                    time = 0;
                    currentState = "Drive'nForward";
                }
            break;

            case "lift":
                

            case "Drive'nForward":
                var currentTargetId = LimelightHelpers.getFiducialID("limelight");
                
                if (April_22 != null){
                    rotationOutput = driveTrain.getTurnBotToAngle(60);
                    if (time < 100){
                        driveTrain.driveLL(rangeOutput, 0, rotationOutput, false, getPeriod);
                    }
                } else if (April_22 == null && time > 50){
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
                    currentState = "PlayerStation";
                }

            break;

            case "PlayerStation":

                
        
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


    Object[] Run_ReefToPlayer(Object[] autoState, double getPeriod) {
        String currentState = autoState[0].toString();

        if(time < 20) {
        driveTrain.driveLL(0.5, 0, 0, false, getPeriod);
        }
        if(time > 20 && time < 100) {
        driveTrain.drive(0, 0, 0, false, false);
        }
        if(time > 100 && time < 200) {
        var rot = driveTrain.getTurnBotToAngle(-21);
        rot = MathUtil.clamp(rot, -0.6, 0.6);

        System.out.println("rot: " + rot);
        driveTrain.drive(0, 0, rot, false, false);
        }
        if(time > 200 && time < 300) {
        driveTrain.drive(0, 0, 0, false, false);
        }
        if(time > 300 && time < 600) {
        var April_12 = driveTrain.GetAprilTagTelemotry(12);
        var rot = driveTrain.getTurnBotToAngle(-21);

        if(April_12 != null) {
            limelightcam.LimeTargetWithRot(getPeriod, rot);
        }
        else {
            driveTrain.driveLL(-0.5, 0, rot, false, getPeriod);
        }
        }
        if(time > 600 && time < 700) {
        var rot = driveTrain.getTurnBotToAngle(135);
        driveTrain.driveLL(0, 0, rot, false, getPeriod);
        }
        if(time > 700 && time < 750) {
        var rot = driveTrain.getTurnBotToAngle(135);
        driveTrain.driveLL(0.5, 0, rot, false, getPeriod);
        }
        //Set the rotation 
        //Align to center of 17
        if(time > 750 && time < 850) {
        var rot = driveTrain.getTurnBotToAngle(135);
        double rotationOutput = limelightcam.LLGetRotation();

        driveTrain.driveLL(0, -rotationOutput, rot, false, getPeriod);
        }
        //Back up towards 12
        if(time > 850 && time < 900) {
        var April_17 = driveTrain.GetAprilTagTelemotry(17);
        var rot = driveTrain.getTurnBotToAngle(135);

        if(April_17 != null) {
            double rangeOutput = limelightcam.LLGetRangeOutput();
            
            rangeOutput = MathUtil.clamp(rangeOutput, -1, 1);

            if(rangeOutput < -3.6) { 
            rangeOutput = MathUtil.clamp(rangeOutput, -0.3, 0.3);
            }

            driveTrain.driveLL(-rangeOutput, -0.5, rot, false, getPeriod);

            if(rangeOutput < -3.8) { 
            time = 900;
            }

        }
        else {
            //driveTrain.driveLL(0.5, 0, rot, false, getPeriod());
        }
        }
        if(time > 900 && time < 1000) {
        var rot = driveTrain.getTurnBotToAngle(135);

        driveTrain.driveLL(0, -0.5, rot, false, getPeriod);
        }

        if(time > 1000) {
        driveTrain.drive(0, 0, 0, false, false);
        }
        time++;
        return new Object[]{currentState, time};

    }
}