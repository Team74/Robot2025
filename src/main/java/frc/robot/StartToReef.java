package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.testrobotautonmovement.RobotMovement.Motor;

public class StartToReef {

    SparkMax liftMotor;
    String state = "start";
    Servo outtakeServo;
    int time;
    driveTrain driveTrain;

    public StartToReef(driveTrain driveTrain1) {
        time = 0;
        driveTrain = driveTrain1;
        System.out.println(state);
    }

    void RunS2R() {
        time++;
        switch (state) {
            // first number in if statement is time in seconds
            case "start":
            driveTrain.drive(0, 0, 0, false);
            if (time > 0*50) {
                time = 0;
                state = "move1";
                break;
            }

            case "move1":
            driveTrain.drive(0.2, 0, 0, false);   
            if (time > 3*50) {
                time = 0;
                state = "lift";
                break;
            }

            case "lift":
            driveTrain.drive(0, 0, 0, false);
            driveTrain.liftLevelSet(4);

            if (time > 1*50) {
                time = 0;
                state = "dumpy";
                break;
            }

            case "dumpy":
            driveTrain.drive(0, 0, 0, false);
            driveTrain.liftLevelSet(4);
            driveTrain.outTakeSet(1);
            if (time > .5*50) {
                time = 0;
                state = "down";
                break;
            }

            case "down":
            driveTrain.drive(0, 0, 0, false); 
            driveTrain.liftLevelSet(1);
            driveTrain.outTakeSet(0);
            if (time > 1*50) {
                time = 0;
                state = "stop";
                break;
            }

            case "stop":
            driveTrain.drive(0, 0, 0, false);
                state = "stop";
                break;
            

        }
    }

    void oldRunS2R() {
        System.out.println(state + time);
        time++;
        switch (state) {
    
            case "start":
            driveTrain.drive(0, 0, 0, false);
            if (time > 0) {
                time = 0;
                state = "move1";
                break;
            }

            case "move1":
            driveTrain.drive(0.2, 0, 0, false);
            if (time > 150) {
                time = 0;
                state = "lift";
                break;
            }

            case "lift":
            driveTrain.drive(0, 0, 0, false);
            if (time > 50) {
                time = 0;
                state = "dumpy";
                break;
            }

            case "dumpy":
            driveTrain.drive(0, 0, 0, false);
            if (time > 25) {
                time = 0;
                state = "down";
                break;
            }

            case "down":
            driveTrain.drive(0, 0, 0, false);
            if (time > 50) {
                time = 0;
                state = "stop";
                break;
            }

            case "stop":
            driveTrain.drive(0, 0, 0, false);                
            state = "stop";
                break; 
            

        }
    }
}
