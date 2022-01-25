package frc.robot.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.motion.drivetrains.SpeedControllerGroup;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpiutil.math.Pair;

public class DriveImpl extends RepeatingPooledSubsystem implements Drive {
    
    // motor controller groups
    private final SpeedControllerGroup leftSide;
    private final SpeedControllerGroup rightSide;
    private final DifferentialDrive drivetrain;

    // encoder
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    // PID controller values

    final double kp = 0;
    final double ki = 0;
    final double kd = 0;

    // PID controllers
    
    private final PIDController leftPIDController;
    private final PIDController rightPIDController;


    public DriveImpl(SpeedControllerGroup leftSide, SpeedControllerGroup rightSide, Encoder leftEncoder, Encoder rightEncoder) {
        super(20, TimeUnit.MILLISECONDS);
        // basic drivetrain stuff
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        this.drivetrain = new DifferentialDrive(leftSide, rightSide);
        this.leftEncoder = leftEncoder; 
        this.rightEncoder = rightEncoder;

        // PID controller impl
        this.leftPIDController = new PIDController(kp, ki, kd);
        this.rightPIDController = new PIDController(kp, ki, kd);

    }

    @Override
    public void defineResources() {
        require(leftSide);
        require(rightSide);
        
        require(leftEncoder);
        require(rightEncoder);
    }

    @Override
    public void task() throws Exception {
        // TODO Auto-generated method stub
    }

    @Override
    public void driveTank(double leftSpeed, double rightSpeed) {
        this.drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void driveArcade(double speed, double rotation) {
        this.drivetrain.arcadeDrive(speed, rotation);
    }

    public void DrivePid(){
        this.drivetrain.tankDrive(this.leftPIDController.calculate(this.leftEncoder.get()), this.rightPIDController.calculate(this.rightEncoder.get()));
    }

    @Override
    public double getHeading() {
        return 0;
    }

    @Override
    public void resetEncoderTicks() {
        // Done
        this.leftEncoder.reset();
        this.rightEncoder.reset();
    }

    @Override
    public Pair<Double, Double> distanceDriven() {
        // return distance of either encoder
        return Pair.of(this.leftEncoder.getDistance(), this.rightEncoder.getDistance());
    }

    public void setSetpoint(double setpoint){
        // TODO Set setpoints for encoders
        
    }



}
