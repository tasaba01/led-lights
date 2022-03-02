package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.Drivetrain.Drive;

// import java.util.concurrent.TimeUnit;
// import ca.team3161.lib.robot.TitanBot;

public class Autonomous {

    private Drive drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Elevator elevator;
    private double wheelCircumference = Math.pow((3.14*2), 2);
    private double distance;
    private double setPoint;

    public Autonomous(Drive drivetrain, Shooter shooter, Intake intake, Elevator elevator){
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.elevator = elevator;
        this.distance = 0;
        this.setPoint = 0;
    }

    double getDistance(RelativeEncoder encoder){
        double gearRatio = 8;
        double ticksPer = encoder.getCountsPerRevolution();
        double revs = encoder.getPosition();

        distance = ((ticksPer * revs) / gearRatio) * wheelCircumference;

        return distance;
    }

    double getSetPoint(RelativeEncoder encoder){
        return getDistance(encoder) / wheelCircumference;
    }

    // void setSetPoint(double targetDistance){
    //     setPoint = targetDistance / wheelCircumference;
    // }

    void path(){
        /*
        if not at target
        target - current position
        */
    }
    
}
