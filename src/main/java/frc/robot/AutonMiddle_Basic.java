package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.driveTrain.ShortcutType;

public class AutonMiddle_Basic {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    LimelightHelpers Limehelp;

    boolean hasPiece() {
        return !driveTrain.proxSensor.get();
    }
    

    public AutonMiddle_Basic(driveTrain _driveTrain, limeLightTest _limelightcam, LimelightHelpers _Limehelp){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
        Limehelp = _Limehelp;
    }

    Object[] Run_2P(Object[] autoState, double getPeriod) {
        // double trackSide = limelightcam.LimeTest();
        // double trackTurn = limelightcam.ReefCenter(); 
        // double trackPush = limelightcam.ReefPush();
        String currentState = autoState[0].toString();
     
        var armPosition = 0;
        var liftMotorPosition = driveTrain.potLift.get();
        var currentTargetId = LimelightHelpers.getFiducialID("limelight");
      

        switch(currentState){

            case "Starting":
               if (time > 0) {
                    driveTrain.drive(0, 0, 0, false, false);
                    driveTrain.resetGyro();
                    time = 0;
                    currentState = "ScoreMove";
               }
            break;
        
            case "ScoreMove":
            
            var liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L1);
            var armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L1);

            if (time > 1 && time < 137){
                driveTrain.drive(0, 0, 0, false, false);
                driveTrain.armMotor.set(armMotorSpeed);
                driveTrain.liftMotor.set(liftMotorSpeed);
            }

            if (time > 137 || liftMotorPosition > 25){
                driveTrain.drive(0, 0, 0, false, false);
                driveTrain.outTakeSet(0);
                driveTrain.armMotor.set(0);
                driveTrain.liftMotor.set(0);
                time = 0;
                currentState = "Drive'nForward";
            }
             
            break;

            case "Drive'nForward":

                //var April_21 = driveTrain.GetAprilTagTelemotry(21);
                //var April_10 = driveTrain.GetAprilTagTelemotry(10);

                //var rangeOutput = limelightcam.LLGetRangeOutput();
                //var rotationOutput = limelightcam.LLGetRotation();

                if (time > 0 && time < 95){
                    driveTrain.drive(-0.3, 0, 0, false, false);
                }
                if (time > 96){
                    driveTrain.drive(0, 0, 0, false, false);
                    time = 0;
                    currentState = "Score";
                }

                /*if (April_10 != null && currentTargetId == 10){
                    if (time > 0 && time < 150){
                        driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    }
                }

                if (April_21 != null && currentTargetId == 21){
                    if (time > 0 && time < 150){
                        driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    }
                }*/
            
            break;

            case "Score":

            if (time > 0 && time < 75){
                driveTrain.outTakeMotorOuter.set(0.5);
            }
            if (time > 76){
                driveTrain.outTakeMotorOuter.set(0);
                //time = 0;
                //currentState = "1";
            }

        }
        time++;
        return new Object[]{currentState, time};
    }

}

