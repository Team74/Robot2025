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
        var liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();
        var currentTargetId = LimelightHelpers.getFiducialID("limelight");
      

        switch(currentState){

            case "Starting":
               if (time > 0) {
                    driveTrain.drive(0, 0, 0, false, false);
                    driveTrain.resetGyro();
                    time = 0;
                    currentState = "Score";
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
            }
             
            break;

            case "Drive'nForward":

                var April_21 = driveTrain.GetAprilTagTelemotry(21);
                var April_10 = driveTrain.GetAprilTagTelemotry(10);

                var rangeOutput = limelightcam.LLGetRangeOutput();
                var rotationOutput = limelightcam.LLGetRotation();

                if (April_10 != null && currentTargetId == 10){
                    if (time > 0 && time < 150){
                        driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    }
                }

                if (April_21 != null && currentTargetId == 21){
                    if (time > 0 && time < 150){
                        driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod);
                    }
                }
            
            break;

        }
        time++;
        return new Object[]{currentState, time};
    }

}

