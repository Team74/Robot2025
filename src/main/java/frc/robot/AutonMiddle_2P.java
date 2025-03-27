package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
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
                    if (time < 50){
                        limelightcam.LimeTargetWithRot(getPeriod, rotationOutput);
                        //driveTrain.driveLL(rangeOutput, 0, rotationOutput, false, getPeriod);
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
                    currentState = "backupReef";
                }

            break;

            case "score":
                if (time > 0 && time < 30){
                    driveTrain.outTakeSet(0.7);
                } else if (hasPiece() == false) {
                    driveTrain.outTakeSet(0);
                    time = 0;
                    currentState = "backupReef";
                }
            break;

            case "backupReef":
                if(time < 20) {
                driveTrain.driveLL(0.5, 0, 0, false, getPeriod);
                }
                if(time > 20 && time < 100) {
                driveTrain.drive(0, 0, 0, false, false);
                time = 0;
                currentState = "turntoPS";
                }
            break;

            case "turntoPS":
            if(time > 0 && time < 100) {
                var rot = driveTrain.getTurnBotToAngle(-21);
                rot = MathUtil.clamp(rot, -0.6, 0.6);

                System.out.println("rot: " + rot);
                driveTrain.drive(0, 0, rot, false, false);
                }
                if(time > 100 && time < 200) {
                driveTrain.drive(0, 0, 0, false, false);
                time = 0;
                currentState = "apriltoPS";
                }
            break;

            case "apriltoPS":
                if(time > 0 && time < 300) {
                    var April_12 = driveTrain.GetAprilTagTelemotry(12);
                    var rot = driveTrain.getTurnBotToAngle(-21);

                    if(April_12 != null) {
                        limelightcam.LimeTargetWithRot(getPeriod, rot);
                    }
                    else {
                        driveTrain.driveLL(-0.5, 0, rot, false, getPeriod);
                    }
                    if (time > 300){
                        driveTrain.driveLL(0, 0, 0, false, getPeriod);
                        time = 0;
                        currentState = "turntoAp17";
                    }
                }
            break;

            case "turntoAp17":
                if(time > 0 && time < 100) {
                    var rot = driveTrain.getTurnBotToAngle(135);
                    driveTrain.driveLL(0, 0, rot, false, getPeriod);
                    }
                    if(time > 100 && time < 150) {
                    var rot = driveTrain.getTurnBotToAngle(135);
                    driveTrain.driveLL(0.5, 0, rot, false, getPeriod);
                }
                if (time > 150){
                    driveTrain.driveLL(0, 0, 0, false, getPeriod);
                    time = 0;
                    currentState = "align17";
                }
            break;

            case "align17":
                if(time > 0 && time < 50) {
                    var rot = driveTrain.getTurnBotToAngle(135);
                    rotationOutput = limelightcam.LLGetRotation();

                    driveTrain.driveLL(0, -rotationOutput, rot, false, getPeriod);
                }
                if (time > 50){
                    driveTrain.driveLL(0, 0, 0, false, getPeriod);
                    time = 0;
                    currentState = "backupPS";
                }
            break;

            case "backupPS":
                if(time > 0 && time < 50) {
                    var April_17 = driveTrain.GetAprilTagTelemotry(17);
                    var rot = driveTrain.getTurnBotToAngle(135);

                    if(April_17 != null) {
                        rangeOutput = limelightcam.LLGetRangeOutput();

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
                        time = 0;
                        currentState = "";
                    }

                }
            break;

            case "":
            if(time > 0 && time < 100) {
                var rot = driveTrain.getTurnBotToAngle(135);
        
                driveTrain.driveLL(0, -0.5, rot, false, getPeriod);
                }
        
                if(time > 100) {
                driveTrain.drive(0, 0, 0, false, false);
            }
            break;
        }

        time++;
        return new Object[]{currentState, time};
    }

}
