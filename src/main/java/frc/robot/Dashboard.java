package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.studica.frc.AHRS;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Dashboard {
    DoublePublisher xPub;
    DoublePublisher Encoder1,Encoder2,Encoder3,Encoder4,Gyro;
    SendableChooser autoncode1;
    DoubleSubscriber ySub;
    Dashboard(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        xPub = table.getDoubleTopic("x").publish();
       
        Encoder1 = table.getDoubleTopic("Encoder1").publish();
        Encoder2 = table.getDoubleTopic("Encoder2").publish();
        Encoder3 = table.getDoubleTopic("Encoder3").publish();
        Encoder4 = table.getDoubleTopic("Encoder4").publish();
        autoncode1 = new SendableChooser<String>();
        //autoncode1 = table.getStringTopic("autoncode1").publish();
        autoncode1.addOption("one", null);
        autoncode1.addOption("two", null); 
        autoncode1.addOption("three", null);
        //autoncode1.addOption("fourth", null);
        Gyro = table.getDoubleTopic("Gyro").publish();

        ySub = table.getDoubleTopic("y").subscribe(0.0);
        table.getDoubleTopic("y").publish(); 
    }
    void updateDashboard(){
        xPub.set(1.0);
     //   System.out.println(ySub.get();

    }
    void updateDashboardSwerveModules(SwerveModuleState[] moduleStates){
        Encoder1.set(moduleStates[0].angle.getDegrees());
        Encoder2.set(moduleStates[1].angle.getDegrees());
        Encoder3.set(moduleStates[2].angle.getDegrees());
        Encoder4.set(moduleStates[3].angle.getDegrees());
            System.out.println(moduleStates[1].angle.getDegrees());
    }
    void updateDashboardGyro(AHRS gyro){
        Gyro.set(gyro.getAngle());
            System.out.println(gyro.getAngle());
    }

}
