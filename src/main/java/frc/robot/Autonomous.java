package frc.robot;

import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxRelativeEncoder;

// SUBSYSTEM IMPORTS
// import frc.robot.subsystems.BallPath.Elevator.Elevator;
// import frc.robot.subsystems.BallPath.Intake.Intake;
// import frc.robot.subsystems.BallPath.Shooter.Shooter;
// import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.BallPath.BallPath.BallAction;
// import frc.robot.subsystems.BallPath.Elevator.Elevator;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;
import frc.robot.subsystems.BallPath.BallPathImpl;
// import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveImpl;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

// import edu.wpi.first.math.controller.PIDController;

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
    private double max = 1000;
    private double min = 0;

    private double kP = 0.00001;
    private double kI = 0;
    private double kD = 0;

    private SparkMaxPIDController leftPIDController;
    private SparkMaxPIDController rightPIDController;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private double maxSpeed = 1;
    // private double minSpeed = -1;
    private double zRotation = 0;





    public Autonomous(DriveImpl drivetrain, BallPathImpl ballPath){
        this.drivetrain = drivetrain;
        this.ballPath = ballPath;
        this.distance = 0;
        this.setPoint = 0;

        this.leftEncoder = drivetrain.getEncoder(1);
        this.rightEncoder = drivetrain.getEncoder(0);
        this.leftPIDController = drivetrain.getController(1).getPIDController();
        this.rightPIDController = drivetrain.getController(0).getPIDController();

        leftPIDController.setP(kP);
        leftPIDController.setI(kI);
        leftPIDController.setD(kD);

        rightPIDController.setP(kP);
        rightPIDController.setI(kI);
        rightPIDController.setD(kD);


        leftPIDController.setOutputRange(min, max);
        rightPIDController.setOutputRange(min, max);

    }

    double calcDistance(RelativeEncoder encoder){

        // Calculate distance from encoder ticks (in inches)

        distance = calcTicks(encoder) * wheelCircumference;

        return distance;
    }

    double calcDistance(double ticks){
        return ticks * wheelCircumference;
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
        rightPIDController.setReference(-target, ControlType.kPosition);
    }

    public double run(double targetPosition){
        /*
        :targetPosition: distance to be driven in inches -> double
        */

        ballPath.setAction(BallAction.FEED);

        setPoint = convertIT(targetPosition); // returns converted value from inches(targetPosition) to encoder ticks(setPpoint)

        boolean intakeLoaded = ballPath.intake.ballPrimed();
        boolean elevatorLoaded = ballPath.elevator.ballPrimed();
        boolean robotFull = intakeLoaded && elevatorLoaded;
        // boolean elevatorOnly = !intakeLoaded && elevatorLoaded;

        // Averaging left and right revolution counts
        double leftRevs = setPoint / leftEncoder.getCountsPerRevolution();
        double rightRevs = setPoint / rightEncoder.getCountsPerRevolution();
        double averageRevs = (leftRevs + rightRevs) / 2;

        positionPIDCalc(averageRevs);

        if (calcTicks(leftEncoder) < setPoint || calcTicks(rightEncoder) < setPoint){
            drivetrain.drivePidTank(maxSpeed, zRotation);
        } else {
            drivetrain.drivePidTank(0, zRotation);
        }

        
        
        if(robotFull){
            ballPath.setAction(BallAction.SHOOT);
            ballPath.shooter.setShotPosition(ShotPosition.AUTO);
        }

        double leftPosition = leftEncoder.getPosition();
        double rightPosition = rightEncoder.getPosition();
        double averagePos = (leftPosition + rightPosition) / 2;

        ballPath.setAction(BallAction.NONE);
        ballPath.shooter.setShotPosition(ShotPosition.NONE);

        return targetPosition - this.calcDistance(averagePos);
    }
    
}
