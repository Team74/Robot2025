package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.driveTrain.ShortcutType;

public class AutonMiddle_L4 {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    LimelightHelpers Limehelp;

    boolean hasPiece() {
        return !driveTrain.proxSensor.get();
    }
    

    public AutonMiddle_L4(driveTrain _driveTrain, limeLightTest _limelightcam, LimelightHelpers _Limehelp){
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
            
            var liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L4);
            var armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L4);

            if (time > 1 && time < 200){
                driveTrain.drive(0, 0, 0, false, false);
                driveTrain.armMotor.set(armMotorSpeed);
                driveTrain.liftMotor.set(liftMotorSpeed);
            }

            if (time > 200 || liftMotorPosition > 55 && armPosition < -85){
                driveTrain.drive(0, 0, 0, false, false);
                driveTrain.outTakeSet(0);
                driveTrain.armMotor.set(0);
                driveTrain.liftMotor.set(0);
                time = 0;
                currentState = "Drive'nForward";
            }
             
            break;

            case "Drive'nForward":

                if (time > 0 && time < 95){
                    driveTrain.drive(-0.3, 0, 0, false, false);
                }
                if (time > 96){
                    driveTrain.drive(0, 0, 0, false, false);
                    time = 0;
                    currentState = "Score";
                }
            
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

