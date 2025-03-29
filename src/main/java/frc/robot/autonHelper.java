package frc.robot;

import frc.robot.driveTrain.ShortcutType;

public class autonHelper {
    driveTrain driveTrain;
    limeLightTest limelightcam;
    double getPeriod = 0.02;

    public autonHelper(driveTrain _driveTrain, limeLightTest _limelightcam){
        driveTrain = _driveTrain;
        limelightcam  = _limelightcam;
    }

    public void DriveForward(double negSpeed) {
        driveTrain.driveLL(negSpeed, 0, 0, false, getPeriod);
    }
    public void DriveForward(double negSpeed, double rotationAngle) {
        var rotation = driveTrain.getTurnBotToAngle(rotationAngle);

        driveTrain.driveLL(negSpeed, 0, rotation, false, getPeriod);
    }

    public void DriveBackward(double posSpeed) {
        driveTrain.driveLL(posSpeed, 0, 0, false, getPeriod);
    }
    public void DriveBackward(double posSpeed, double rotation) {

        driveTrain.driveLL(posSpeed, 0, rotation, false, getPeriod);
    }

    public void DriveLeft(double negSpeed) {
        driveTrain.driveLL(0, negSpeed, 0, false, getPeriod);
    }
    public void DriveLeft(double negSpeed, double rotation) {

        driveTrain.driveLL(0, negSpeed, rotation, false, getPeriod);
    }

    public void DriveRight(double posSpeed) {
        driveTrain.driveLL(0, posSpeed, 0, false, getPeriod);
    }
    public void DriveRight(double posSpeed, double rotation) {

        driveTrain.driveLL(0, posSpeed, rotation, false, getPeriod);
    }

    public void Stop() {
        driveTrain.driveLL(0, 0, 0, false, getPeriod);
    }

    public void RotateBot(double targetAngle) {
        var rotationOutput = driveTrain.getTurnBotToAngle(targetAngle);

        driveTrain.driveLL(0, 0, rotationOutput, false, getPeriod);
    }

    public void MoveArmLiftToShortcut(ShortcutType shorType) {
        var liftMotorSpeed = driveTrain.ShortCutLift(shorType);
        var armMotorSpeed = driveTrain.ShortCutArm(shorType);

        driveTrain.armMotor.set(armMotorSpeed);
        driveTrain.liftMotor.set(liftMotorSpeed);
    }

    public void Intake(double negSpeed) {
        driveTrain.outTakeSet(negSpeed);
    }

    public void OutTake(double posSpeed) {
        driveTrain.outTakeSet(posSpeed);
    }

    public void AlignTargetAprilTag(boolean reverse) {
        var rotationOutput = 0.0;
        var tagId = LimelightHelpers.getFiducialID("limelight");
        var aprilTag = AprilTagLib.AprilTagFromId(tagId);

        if(aprilTag != null) {
            rotationOutput = aprilTag.Rotation;

            if(reverse) {
                rotationOutput = aprilTag.ReverseRotation;
            }

            limelightcam.LimeTargetWithRot(getPeriod, rotationOutput);
        }
    }

    public boolean between(double num1, double theNumber, double num2) {
        return theNumber > num1 && theNumber < num2;
    }

    public static class AprilTag {
        public double Rotation;
        //If backing towards an april tag
        public double ReverseRotation;

        public AprilTag(double _Rotation) {
            Rotation = _Rotation;
            ReverseRotation = (_Rotation + 180.0);
        }
        public AprilTag(double _Rotation, double _ReverseRotation) {
            Rotation = _Rotation;
            ReverseRotation = _ReverseRotation;
        }
    }
    public class AprilTagLib {
        public static AprilTag April_22 = new AprilTag(-60, 120);
        public static AprilTag April_12 = new AprilTag(54, -126);
        public static AprilTag April_17 = new AprilTag(-120, 60);

        public static AprilTag AprilTagFromId(double TagId) {
            if(TagId == 22) {
                return April_22;
            }

            if(TagId == 12) {
                return April_12;
            }

            if(TagId == 17) {
                return April_17;
            }

            return null;
        }
    }
    

}


