package frc.robot;

import static edu.wpi.first.units.Units.Second;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.driveTrain.ShortcutType;
import frc.robot.limeLightTest;

public class AutonDriveForward {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    

    public AutonDriveForward(driveTrain _driveTrain, limeLightTest _limelightcam){
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
                currentState = "DriveForward";
            break;

            case "DriveForward":

            if(time > 10 && time <= 80) {
                driveTrain.drive(-0.6, 0, 0, false, false);
            }
            if (time > 80) {
                driveTrain.drive(0, 0, 0, false, false);

                time = 0;
                currentState = "Score11";    
            }
             break;          
        }
        time++;
        return new Object[]{currentState, time};
    }
}
