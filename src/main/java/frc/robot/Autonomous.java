package frc.robot;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.Drivetrain.Drive;

// PIDCONTROLLER IMPORTS
import edu.wpi.first.math.controller.PIDController;

// ENCODER IMPORTS
import com.revrobotics.RelativeEncoder;

public class Autonomous {

    private Drive drivetrain;
    private BallPath ballPath;
    private double wheelCircumference = Math.pow((Math.PI*2), 2);
    private double distance;
    private double setPoint;

    private double kP = 0.00001;
    private double kI = 0;
    private double kD = 0;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    
    private double zRotation = 0;
    private double targetDistance;

    private PIDController positionPIDController;





    public Autonomous(Drive drivetrain, BallPath ballPath){
        this.drivetrain = drivetrain;
        this.ballPath = ballPath;
        this.distance = 0;
        this.setPoint = 0;

        this.leftEncoder = drivetrain.getEncoder(1);
        this.rightEncoder = drivetrain.getEncoder(0);
        this.positionPIDController = new PIDController(kP, kI, kD);
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

    double positionPIDCalc(){
        // PID for both sides
        double averagePos = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
        return positionPIDController.calculate(averagePos);
    }

    public void setDriveDistance(double targetDistance) {
        this.targetDistance = convertIT(targetDistance);
        positionPIDController.setSetpoint(this.targetDistance);
    }

    public boolean drive(){
        /*
        :targetPosition: distance to be driven in inches -> double
        */

        ballPath.setAction(BallAction.FEED);

        double nextSpeed = positionPIDCalc();

        if (calcTicks(leftEncoder) < setPoint || calcTicks(rightEncoder) < setPoint){
            drivetrain.drivePidTank(nextSpeed, zRotation);
        } else {
            drivetrain.drivePidTank(0, zRotation);
        }

        return positionPIDController.atSetpoint();
    }

    void shoot(){
        boolean intakeLoaded = ballPath.getIntake().ballPrimed();
        boolean elevatorLoaded = ballPath.getElevator().ballPrimed();
        boolean robotFull = intakeLoaded && elevatorLoaded;

        if (robotFull){
            ballPath.setAction(BallAction.SHOOT);
            ballPath.getShooter().setShotPosition(ShotPosition.AUTO);
        }
    }

    void stop(){
        this.ballPath.setAction(BallAction.NONE);
        this.ballPath.getShooter().setShotPosition(ShotPosition.NONE);
    }
    
}
