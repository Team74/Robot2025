package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.driveTrain.ShortcutType;

public class AutonMiddle_Basic {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;

    boolean hasPiece() {
        return !driveTrain.proxSensor.get();
    }
    

    public AutonMiddle_Basic(driveTrain _driveTrain, limeLightTest _limelightcam){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
    }

    Object[] Run_2P(Object[] autoState) {
        // double trackSide = limelightcam.LimeTest();
        // double trackTurn = limelightcam.ReefCenter(); 
        // double trackPush = limelightcam.ReefPush();
        String currentState = autoState[0].toString();
     
        var armPosition = 0;
        var liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();
      

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
            }
             
            break;

        }
        time++;
        return new Object[]{currentState, time};
    }

}

