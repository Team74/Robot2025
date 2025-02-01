package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class dashBoard {
    DoublePublisher xPub;
    DoubleSubscriber ySub;
    dashBoard(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        xPub = table.getDoubleTopic("x").publish();
        ySub = table.getDoubleTopic("y").subscribe(0.0);
        table.getDoubleTopic("y").publish();
    }
    void update(){
        xPub.set(1.5);
        System.out.println(ySub.get());
    }

}
