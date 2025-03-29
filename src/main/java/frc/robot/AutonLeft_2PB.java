package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.driveTrain.ShortcutType;

public class AutonLeft_2PB {
    int time;
    driveTrain driveTrain;
    limeLightTest limelightcam;
    LimelightHelpers Limehelp;
    double encodercount; 

    boolean hasPiece() {
        return !driveTrain.proxSensor.get();
    }
    

    public AutonLeft_2PB(driveTrain _driveTrain, limeLightTest _limelightcam, LimelightHelpers _Limehelp){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
        Limehelp = _Limehelp;
    }

    Object[] Run_2P(Object[] autoState, double getPeriod) {
        String currentState = autoState[0].toString();

        encodercount = driveTrain.leftFront.driveMotor.getEncoder().getPosition(); 
    if (time > 0 && time < 500 && encodercount < 24.42){
        driveTrain.drive (-0.3, 0, 0, false, false);
    }
    if (time > 500 || encodercount > 24.42){
        driveTrain.drive (0, 0, 0, false, false);
    }
        time++;
        return new Object[]{currentState, time};
    }

}

