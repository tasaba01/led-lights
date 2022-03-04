package frc.robot;

// SUBSYSTEM IMPORTS
import frc.robot.subsystems.BallPath.BallPath.BallAction;
import frc.robot.subsystems.BallPath.Shooter.Shooter.ShotPosition;
import frc.robot.subsystems.BallPath.BallPath;
import frc.robot.subsystems.Drivetrain.Drive;
import edu.wpi.first.math.Pair;

// PIDCONTROLLER IMPORTS
import edu.wpi.first.math.controller.PIDController;

public class Autonomous {

    private static final double DRIVE_DIST_TOLERANCE = 200; // TODO determine what this should really be
    
    private Drive drivetrain;
    private BallPath ballPath;
    private double wheelCircumference = Math.pow((Math.PI*2), 2);

    private double kP = 0.00001;
    private double kI = 0;
    private double kD = 0;

    private double zRotation = 0;
    private double targetDistance;

    private final PIDController positionPIDController;

    public Autonomous(Drive drivetrain, BallPath ballPath){
        this.drivetrain = drivetrain;
        this.ballPath = ballPath;
        this.positionPIDController = new PIDController(kP, kI, kD);
        this.positionPIDController.setTolerance(DRIVE_DIST_TOLERANCE);
    }

    double calcTicks(double encoderTicks){
        // Calculates ticks per revolution(shaft rotation)
        double gearRatio = 8;

        double ticks = encoderTicks / gearRatio; // ticksPer cancels so it's just encoderTicks / gearRatio

        return ticks;
    }

    double convertIT(double distance){
        // Convert Inches to Ticks
        return distance / wheelCircumference;
    }

    double convertTI(double ticks){
        // Convert Ticks to Inches
        return ticks * wheelCircumference;
    }

    double positionPIDCalc(Double encoderTicks){
        // PID for both sides

        return positionPIDController.calculate(encoderTicks);
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

        Pair<Double, Double> encoderTicks = this.drivetrain.getEncoderTicks();

        double averageEncoderTicks = (encoderTicks.getFirst() + encoderTicks.getSecond()) / 2;

        double nextSpeed = positionPIDCalc(averageEncoderTicks);

        drivetrain.drivePidTank(nextSpeed, zRotation);

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
