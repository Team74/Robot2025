package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GotoPose {
    driveTrain driveTrain;

    private double targetX;
    private double targetY;
    private double targetAngle;

    private double currentX;
    private double currentY;
    private double currentAngle;

    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;
   
    public GotoPose(driveTrain driveTrain){
        this.driveTrain = driveTrain;

        xController = new PIDController(2, 0.0, 0.005);
        xController.setIntegratorRange(-0.2, 0.2);
        yController = new PIDController(2, 0.0, 0.005);
        yController.setIntegratorRange(-0.2, 0.2);
        angleController = new PIDController(2, 0.0, 0.005);
        angleController.setIntegratorRange(-0.2, 0.2);
    }
 
    public void execute(FieldPose targetFieldPose){
        this.targetX = targetFieldPose.getPose2d().getX();
        this.targetY = targetFieldPose.getPose2d().getY();
        this.targetAngle = targetFieldPose.getPose2d().getRotation().getRadians();

        currentX = driveTrain.odometry.getEstimatedPosition().getX();
        currentY = driveTrain.odometry.getEstimatedPosition().getY();
        currentAngle = driveTrain.odometry.getEstimatedPosition().getRotation().getRadians();

        double xOutput = xController.calculate(currentX, targetX);
        double yOutput = yController.calculate(currentY, targetY);
        double angleOutput = angleController.calculate(currentAngle, targetAngle);
        xOutput = Math.min(0.3, Math.max(-0.3, xOutput));
        yOutput = Math.min(0.3, Math.max(-0.3, yOutput));
        angleOutput = Math.min(0.3, Math.max(-0.3, angleOutput));
        driveTrain.drive(xOutput, yOutput, angleOutput, false, false);
    }

    enum FieldPose {
        BLUE_18(new Pose2d(new Translation2d(3.7, 4.0), new Rotation2d(0.0))),
        BLUE_22(new Pose2d(new Translation2d(4.9, 3.3), new Rotation2d(120.0)));

        private Pose2d pose2d;
        
        FieldPose(Pose2d pose2d) {
            this.pose2d = pose2d;
        }

        public Pose2d getPose2d() {
            return pose2d;
        }
    }
}
