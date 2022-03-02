package frc.robot.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import ca.team3161.lib.robot.LifecycleEvent;
// import ca.team3161.lib.robot.motion.drivetrains.SpeedControllerGroup;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Pair;

public class DriveImpl extends RepeatingPooledSubsystem implements Drive {
    
    // motor controller groups
    private final CANSparkMax leftSide;
    private final CANSparkMax rightSide;
    private final DifferentialDrive drivetrain;

    // encoder
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // PID controller values

    // PID controller values

    private double kP = 6e-5; 
    private double kI = 0;
    private double kD = 0; 
    // private double kIz = 0; 
    // private double kFF = 0; 
    private double kMaxOutput = 1; 
    private double kMinOutput = -1;
    private double maxRPM = 5700;

    // PID controllers
    
    private final SparkMaxPIDController leftPIDController;
    private final SparkMaxPIDController rightPIDController;


    public DriveImpl(CANSparkMax leftSide, CANSparkMax rightSide) {
        super(20, TimeUnit.MILLISECONDS);
        // basic drivetrain stuff
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        this.drivetrain = new DifferentialDrive(leftSide, rightSide);
        this.leftEncoder = this.leftSide.getEncoder();
        this.rightEncoder = this.rightSide.getEncoder();

        // PID controller impl
        this.leftPIDController = leftSide.getPIDController();
        this.rightPIDController = rightSide.getPIDController();

        // Set PID Constants
        leftPIDController.setP(kP);
        leftPIDController.setI(kI);
        leftPIDController.setD(kD);
        // leftPIDController.setIZone(kIz);
        // leftPIDController.setFF(kFF);
        leftPIDController.setOutputRange(kMinOutput, kMaxOutput);

        rightPIDController.setP(kP);
        rightPIDController.setI(kI);
        rightPIDController.setD(kD);
        // rightPIDController.setIZone(kIz);
        // rightPIDController.setFF(kFF);
        rightPIDController.setOutputRange(kMinOutput, kMaxOutput);


        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Zone", kIz);
        // SmartDashboard.putNumber("Feed Forward", kFF);

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

    @Override
    public void  drivePidTank(double speed, double rotation){


        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        // double iz = SmartDashboard.getNumber("I Zone", 0);
        // double ff = SmartDashboard.getNumber("Feed Forward", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        
        if((p != kP)) { leftPIDController.setP(p); kP = p; rightPIDController.setP(p); kP = p; }
        if((i != kI)) { leftPIDController.setI(i); kI = i; rightPIDController.setI(i); kI = i; }
        if((d != kD)) { leftPIDController.setD(d); kD = d; rightPIDController.setD(d); kD = d; }
        // if((iz != kIz)) { leftPIDController.setIZone(iz); kIz = iz; rightPIDController.setIZone(iz); kIz = iz; }
        // if((ff != kFF)) { leftPIDController.setFF(ff); kFF = ff; rightPIDController.setFF(ff); kFF = ff; }

        leftPIDController.setOutputRange(kMinOutput, kMaxOutput); 
        rightPIDController.setOutputRange(kMinOutput, kMaxOutput); 
        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         *  com.revrobotics.CANSparkMax.ControlType.kPosition
         *  com.revrobotics.CANSparkMax.ControlType.kVelocity
         *  com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        double leftSetPoint = speed*maxRPM;
        double rotationSetPoint = rotation*maxRPM;
        leftPIDController.setReference(leftSetPoint-(Math.sqrt(rotationSetPoint)), CANSparkMax.ControlType.kVelocity);
        rightPIDController.setReference(leftSetPoint+(Math.sqrt(rotationSetPoint)), CANSparkMax.ControlType.kVelocity);
        
        SmartDashboard.putNumber("Left SetPoint", leftSetPoint);
        SmartDashboard.putNumber("Right SetPoint", leftSetPoint);
        SmartDashboard.putNumber("Left Encoder Velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Encoder Velocity", rightEncoder.getVelocity());


        
        //this.drivetrain.tankDrive(this.leftPIDController.calculate(this.leftEncoder.getVelocity(), ), this.rightPIDController.calculate(this.rightEncoder.getVelocity()));
        
    }

    @Override
    public double getHeading() {
        return 0;
    }

    @Override
    public void resetEncoderTicks() {
        // Done
        // this.leftEncoder.reset();
        // this.rightEncoder.reset();
    }

    @Override
    public Pair<Double, Double> distanceDriven() {
        // return distance of either encoder
        return Pair.of(this.leftEncoder.getPosition(), this.rightEncoder.getPosition());
    }

    public void setSetpoint(double setpoint){
        // TODO Set setpoints for encoders
        
    }
    
    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {}

}
