package frc.robot;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import frc.robot.LimelightHelpers.RawFiducial;

public class Auton_2P {
   int time;
   driveTrain driveTrain;
    public Auton_2P(driveTrain _driveTrain){
        driveTrain = _driveTrain;
    }
    Object[] Run_2P(Object[] autoState) {

        String currentState = autoState[0].toString();

        switch(currentState){
            case "Starting":
            driveTrain.resetGyro();
            driveTrain.drive( 0, 0, 0, false);
            time = 0;
            currentState = "Move'nToReef";
            break;

            case "Move'nToReef":
            
            if(time > 0 && time <= 90) {
                driveTrain.drive(0, .6, 0, false);
            }
            if(time > 90 && time <= 110) {
                driveTrain.drive( 0, 0, -1, false);
            }
            if(time > 110 && time <= 135) {
                driveTrain.drive(0, .6, 0, false);
            }
            if (time > 135) {
                driveTrain.drive(0, 0, 0, false);

                var April_22 = driveTrain.GetAprilTagTelemotry(22);
                var April_9 = driveTrain.GetAprilTagTelemotry(9);
                
                if (April_22 != null){
                    driveTrain.drive( 0, 0.4, 45, false);
                    if (time > 200) {
                        driveTrain.drive( 0, 0, 45, false);
                    }
                }
                
            }
            
                
            
        }
        time++;
        return new Object[]{currentState, time};
    }
}
