package frc.robot;

import edu.wpi.first.math.MathUtil;
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
        String currentState = autoState[0].toString();
     
        var armPosition = 0;
        var liftMotorPosition = driveTrain.potLift.get();
        var currentTargetId = LimelightHelpers.getFiducialID("limelight");
        var persistRotationOutput = limelightcam.LLGetRotation();
        var persistRangeOutput = limelightcam.LLGetRangeOutput();
        var rotationOutput = driveTrain.getTurnBotToAngle(0);
      

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

                var April_21 = driveTrain.GetAprilTagTelemotry(22); //change tag id

                if(time > 0 && time < 95) {
                    if(April_21 != null) {
                        rotationOutput = driveTrain.getTurnBotToAngle(0);
                        persistRotationOutput = limelightcam.LLGetRotation();
                        persistRangeOutput = limelightcam.LLGetRangeOutput();
                        
                        persistRotationOutput = MathUtil.clamp(persistRotationOutput, -1, 1);
                        persistRangeOutput = MathUtil.clamp(persistRangeOutput, -0.8, 0.8);
                        
                    }

                    driveTrain.driveLL(persistRangeOutput, -persistRotationOutput, rotationOutput, false, getPeriod);
                }
                if (time > 95 || April_21 == null){
                    driveTrain.drive(0, 0, 0, false, false);
                    time = 0;
                    currentState = "adjust";
                }
            
            break;

            case "adjust":
                if (time > 0 && time < 23){
                    driveTrain.driveLL(-0.25, 0.3, 0, false, getPeriod);
                } else {
                    driveTrain.driveLL(0, 0, 0, false, getPeriod);
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
            break;

        }
        time++;
        return new Object[]{currentState, time};
    }

}

