package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class PlayerToReef {

    driveTrain drive;
    String state = "start";
    int time;
    double turnSpeed = 0;

    PIDController pidTurn = new PIDController(0.2, 0, 0);

    public PlayerToReef(driveTrain train) {
        drive = train;
    }
    void run(){
        time++;
        switch (state) {

        case "start":
        drive.resetGyro();
        drive.gyroOffset(36);
        turnSpeed = pidTurn.calculate(drive.getGyro(), 36);
        drive.drive(0, 0, turnSpeed, false);
        // drive.outTakeSet(0);
        // drive.liftLevelSet(1);
        // drive.armSet(310);
        if (time > 0) {
            time = 0;
            state = "in";
            break; }

            case "in":
            drive.outTakeSet(-1);
            if (time > 20){
                time = 0;
                break;
            }

            case "move1":
            turnSpeed = pidTurn.calculate(drive.getGyro(), 60);
            drive.drive(0.577350538379/2, 1/2, turnSpeed, false);
            // drive.outTakeSet(0);
            // drive.liftLevelSet(1);
            if (time > 20){
            // drive.armSet(45);
            }
            if (time > 75) {
                time = 0;
                state = "score";
                break; }

                case "score":
            turnSpeed = pidTurn.calculate(drive.getGyro(), 60);
            drive.drive(0, 0, turnSpeed, false);
            // drive.liftLevelSet(4);
            // drive.armSet(45);
            if (time > 74 && time < 100) {
                // drive.outTakeSet(1);
            }
            if (time > 100) {
                // drive.outTakeSet(0);
            }
            if (time > 175) {
                time = 0;
                state = "move2";
                break; }

                case "move2":
                turnSpeed = pidTurn.calculate(drive.getGyro(), 60);
                drive.drive(-0.577350538379/2, -1/2, turnSpeed, false);
                // drive.outTakeSet(0);
                
                if (time > 20){
                // drive.armSet(310);
                // drive.liftLevelSet(1);
                }
                if (time > 75) {
                    time = 0;
                    state = "stop";
                    break; }
                
                case "stop": 
                    drive.drive(0, 0, 0, false);
                    // drive.armSet(310);
                    // drive.liftLevelSet(1);
                    // drive.outTakeSet(0);
                    time = 0;
                    drive.gyroOffset(0);
                    break;



        
        
        }   
}

}
