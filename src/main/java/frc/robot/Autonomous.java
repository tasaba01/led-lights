package frc.robot;

import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxRelativeEncoder;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Intake.Intake;
import frc.robot.subsystems.BallPath.Shooter.Shooter;
import frc.robot.subsystems.BallPath.BallPathImpl;
// import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveImpl;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

// import java.util.concurrent.TimeUnit;
// import ca.team3161.lib.robot.TitanBot;

public class Autonomous {

    private DriveImpl drivetrain;
    // private Shooter shooter;
    // private Intake intake;
    // private Elevator elevator;
    private BallPathImpl ballPath;
    private double wheelCircumference = Math.pow((Math.PI*2), 2);
    private double distance;
    private double setPoint;
    private double max = 10;
    private double min = 0;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    private SparkMaxPIDController leftPIDController;
    private SparkMaxPIDController rightPIDController;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;





    public Autonomous(DriveImpl drivetrain, Shooter shooter, Intake intake, Elevator elevator){
        this.drivetrain = drivetrain;
        this.ballPath = new BallPathImpl(intake, elevator, shooter);
        this.distance = 0;
        this.setPoint = 0;

        this.leftEncoder = drivetrain.leftEncoder;
        this.rightEncoder = drivetrain.rightEncoder;
        this.leftPIDController = drivetrain.leftSide.getPIDController();
        this.rightPIDController = drivetrain.rightSide.getPIDController();


        leftPIDController.setOutputRange(min, max);
        rightPIDController.setOutputRange(min, max);

    }

    double calcDistance(RelativeEncoder encoder){

        // Calculate distance from encoder ticks (in inches)

        distance = calcTicks(encoder) * wheelCircumference;

        return distance;
    }

    double calcTicks(RelativeEncoder encoder){
        // Calculates ticks per revolution(shaft rotation)
        double gearRatio = 8;
        double ticksPer = encoder.getCountsPerRevolution();
        double revs = encoder.getPosition();

        double ticks = ((ticksPer * revs) / gearRatio);

        return ticks;
    }

    double getCurrentPosition(RelativeEncoder encoder){
        // return current encoder ticks for distance driven
        return calcDistance(encoder) / wheelCircumference;
    }

    // void setSetPoint(double targetDistance){
    //     setPoint = targetDistance / wheelCircumference;
    // }

    double convertIT(double distance){
        // Convert Inches to Ticks
        return distance / wheelCircumference;
    }

    double convertTI(double ticks){
        // Convert Ticks to Inches
        return ticks * wheelCircumference;
    }

    // double PIDCalc(PIDController pidController){
    //     double output;
    //     output = pidController.calculate(this.drivetrain.leftEncoder.getPosition());

    //     return output;
    // }

    void positionPIDCalc(double target){
        // PID for both sides
        leftPIDController.setReference(target, ControlType.kPosition);
        rightPIDController.setReference(target, ControlType.kPosition);
    }

    void run(double targetPosition){
        /*
        :targetPosition: distance to be driven in inches -> double

        */

        setPoint = convertIT(targetPosition);
        double revs = setPoint / leftEncoder.getCountsPerRevolution();
        positionPIDCalc(revs);
    }
    
}
