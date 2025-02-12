package frc.robot;

import com.revrobotics.spark.SparkMax;

import frc.robot.testrobotautonmovement.RobotMovement.Motor;

public class PlayerToReef {
    SwerveModule rightFront;
    SwerveModule rightBack;
    SwerveModule leftFront;
    SwerveModule leftBack;
    SparkMax liftMotor;
    String state = "start";

    public PlayerToReef(SwerveModule[] List, SparkMax Lift) {
        rightFront = List[0];
        rightBack = List[1];
        leftFront = List[2];
        leftBack = List[3];
        liftMotor = Lift;
    }

    void RunP2R(int time) {
        switch (state) {
            // first number in if statement is time in seconds
            case "start":
            MotorSet(0.0,0.0,false);
            if (time > 0*50) {
                time = 0;
                state = "move1";
                break;
            }

            case "move1":
            MotorSet(0.1,0.0,false);
            if (time > 1*50) {
                time = 0;
                state = "move2";
                break;
            }



        }
    }



    void MotorSet(double speed, double angle, boolean turning){
        if(turning) {
        rightFront.turny(45);
        leftFront.turny( 135);
        rightBack.turny(135);
        leftBack.turny(45);
        rightFront.movey(speed);
        leftFront.movey(speed);
        rightBack.movey(-speed);
        leftBack.movey(-speed);
        } else

        rightFront.turny(angle);
        leftFront.turny(angle);
        rightBack.turny(angle);
        leftBack.turny(angle);
        rightFront.movey(speed);
        leftFront.movey(speed);
        rightBack.movey(speed);
        leftBack.movey(speed);


    }
}