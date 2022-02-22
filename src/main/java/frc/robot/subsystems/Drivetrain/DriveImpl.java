package frc.robot.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

import com.revrobotics.RelativeEncoder;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.motion.drivetrains.SpeedControllerGroup;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.Pair;

public class DriveImpl extends RepeatingPooledSubsystem implements Drive {
    
    // motor controller groups
    private final SpeedControllerGroup leftSide;
    private final SpeedControllerGroup rightSide;
    private final DifferentialDrive drivetrain;

    // encoder
    
    private final RelativeEncoder leftEncoder1;
    private final RelativeEncoder leftEncoder2;
    private final RelativeEncoder rightEncoder1;
    private final RelativeEncoder rightEncoder2;

    // PID controller values

    final double kp = 0;
    final double ki = 0;
    final double kd = 0;

    // PID controllers
    
    private final PIDController leftPIDController;
    private final PIDController rightPIDController;


    public DriveImpl(SpeedControllerGroup leftSide, SpeedControllerGroup rightSide, RelativeEncoder leftEncoder1, RelativeEncoder leftEncoder2, RelativeEncoder rightEncoder1, RelativeEncoder rightEncoder2) {
        super(20, TimeUnit.MILLISECONDS);
        // basic drivetrain stuff
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        this.drivetrain = new DifferentialDrive(leftSide, rightSide);
        this.leftEncoder1 = leftEncoder1; 
        this.leftEncoder2 = leftEncoder2; 
        this.rightEncoder1 = rightEncoder1; 
        this.rightEncoder2 = rightEncoder2; 

        // PID controller impl
        this.leftPIDController = new PIDController(kp, ki, kd);
        this.rightPIDController = new PIDController(kp, ki, kd);

    }

    @Override
    public void defineResources() {
        require(leftSide);
        require(rightSide);
        
        require(leftEncoder1);
        require(rightEncoder1);
        require(leftEncoder2);
        require(rightEncoder2);
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
        this.drivetrain.tankDrive(this.leftPIDController.calculate(this.leftEncoder1.getVelocity()), this.rightPIDController.calculate(this.rightEncoder1.getVelocity()));
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
        // removing this for now as relative encoder reset on boot
    }

    @Override
    public Pair<Double, Double> distanceDriven() {
        // return distance of either encoder
        return Pair.of(this.leftEncoder1.getPosition() , this.rightEncoder1.getPosition());
    }

    public void setSetpoint(double setpoint){
        // TODO Set setpoints for encoders
        
    }
    
    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {}

}
