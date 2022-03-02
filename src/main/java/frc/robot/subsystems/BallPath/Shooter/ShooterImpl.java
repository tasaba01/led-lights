package frc.robot.subsystems.BallPath.Shooter;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
// import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends RepeatingPooledSubsystem implements Shooter {
    
    private TalonSRX turretMotor;
    private TalonFX shooterMotor;
    private TalonSRX hoodMotor;

    private double kp = 0.00035;
    private double ki = 0.000075;
    // private final double kd = -0.0002;  Value that was being tested as of February 3, 2022.
    private double kd = 0.00003;

    private double setPointHood;
    private double setPointShooter;
    private double setPointRotation;
    private double setPointShooterPID;

    private double shooterEncoderReadingPosition;
    double shooterEncoderReadingVelocity;
    private double turretHoodPosition;
    private double turretEncoderReadingPosition;
    private double turretHoodVelocity;
    private double turretEncoderReadingVelocity;

    private boolean turretReady = false;
    private boolean hoodReady = false;

    double currentOutput;

    private final PIDController shooterPid = new PIDController(kp, ki, kd);

    public ShooterImpl(TalonSRX turretMotor, TalonFX shooterMotor, TalonSRX hoodMotor) {
        super(10, TimeUnit.MILLISECONDS);
        // TODO Auto-generated constructor stub
        this.turretMotor = turretMotor;
        this.shooterMotor = shooterMotor;
        this.hoodMotor = hoodMotor;

        shooterEncoderReadingPosition = shooterMotor.getSelectedSensorPosition();
        shooterEncoderReadingVelocity = shooterMotor.getSelectedSensorVelocity();
        turretHoodPosition = hoodMotor.getSelectedSensorPosition();
        turretHoodVelocity = hoodMotor.getSelectedSensorVelocity();
        turretEncoderReadingPosition = this.turretMotor.getSelectedSensorPosition();
        turretEncoderReadingVelocity = this.turretMotor.getSelectedSensorVelocity();

        SmartDashboard.putNumber("Shooter Encoder reading position", shooterEncoderReadingPosition);
        SmartDashboard.putNumber("Shooter Encoder Reading Velocity", shooterEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Encoder Reading Position", turretEncoderReadingPosition);
        SmartDashboard.putNumber("Turret Encoder Reading Velocity", turretEncoderReadingVelocity);
        SmartDashboard.putNumber("Turret Hood Encoder reading Position", turretHoodPosition);
        SmartDashboard.putNumber("Turret Hood Encoder Reading Velocity", turretHoodVelocity);

    }

    @Override
    public void defineResources(){
        require(turretMotor);
        require(shooterMotor);
        require(hoodMotor);

        require(shooterEncoderReadingPosition);
        require(shooterEncoderReadingVelocity);
        require(turretHoodPosition);
        require(turretHoodVelocity);
        require(turretEncoderReadingPosition);
        require(turretEncoderReadingVelocity);
    }

    @Override
    public void task(){}

    @Override
    public void findAndCenterTarget() {}

    @Override
    public void centerTarget(double tx){}

    @Override
    public void getDistance(double ty, double angle1, double angle2){}

    // runs flywheel
    @Override
    public boolean readyToShoot(){
        if (turretReady && hoodReady){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public int checkBalls(){
        return 0;
    }

    @Override
    public void setHoodAngle(double setPointHood){
        if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          hoodReady = false;
        }else if(turretHoodPosition <= setPointHood - 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0.3);
          hoodReady = false;
        }else if (turretHoodPosition >= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, -0.3);
          hoodReady = false;
        }
    }

    @Override
    public void start(){}

    @Override
    public void stop(){
        System.out.println("DISABLING THE SHOOTER");
        setPointHood = 0;
        setPointRotation = 0;

        shooterMotor.set(ControlMode.PercentOutput, 0);

        if(turretEncoderReadingPosition >= setPointRotation - 30000 && turretEncoderReadingPosition <= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = false;
        }else if(turretEncoderReadingPosition <= setPointRotation - 30000){
          turretMotor.set(ControlMode.PercentOutput, 0.3);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, -0.3);
          turretReady = false;
        }
        
        if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          hoodReady = false;
        }else if(turretHoodPosition <= setPointHood - 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0.3);
          hoodReady = false;
        }else if (turretHoodPosition >= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, -0.3);
          hoodReady = false;
        }

    }

    @Override
    public void shootFender() {
        // Fender shot for teleop
        System.out.println("FENDER SHOT: ");
        setPointHood = 100000;
        // setPointShooter = 0.33;
        setPointShooterPID = 0; // tbd
        setPointRotation = 0;
        System.out.println("SETPOINT HOOD: " + setPointHood);
        System.out.println("SETPOINT SHOOTER " + setPointShooter);
        System.out.println("SETPOINT FOR THE TURRET TURNING " + setPointRotation);

        // if we dont want to use PID, use the below
        // shooterMotor.set(ControlMode.PercentOutput, setPointShooter);
        // System.out.println(setPointShooter);

        // pid for shooter
        if(setPointShooter != 0){
          currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooter);
          currentOutput = Utils.normalizePwm(currentOutput);
          System.out.println(currentOutput);
          this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
          SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooter);
          SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        }

        if(turretEncoderReadingPosition >= setPointRotation - 30000 && turretEncoderReadingPosition <= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - 30000){
          turretMotor.set(ControlMode.PercentOutput, 0.3);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, -0.3);
          turretReady = false;
        }
      
        if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          hoodReady = true;
        }else if(turretHoodPosition <= setPointHood - 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0.3);
          hoodReady = false;
        }else if (turretHoodPosition >= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, -0.3);
          hoodReady = false;
        }
    }

    @Override
    public void shootFarLaunch() {
        // TODO Auto-generated method stub
        System.out.println("FAR LAUNCHPAD SHOT: ");
        setPointHood = 300000;
        // setPointShooter = 0.65;
        setPointShooterPID = 0; // tbd
        setPointRotation = 200000;
        System.out.println("SETPOINT HOOD: " + setPointHood);
        System.out.println("SETPOINT SHOOTER " + setPointShooter);
        System.out.println("SETPOINT FOR THE TURRET TURNING " + setPointRotation);

        // Shooter without PID
        // shooterMotor.set(ControlMode.PercentOutput, setPointShooter);
        // SmartDashboard.putNumber("Shooter Output", setPointShooter);

        // shooter with PID
        // pid for shooter
        if(setPointShooter != 0){
          currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooter);
          currentOutput = Utils.normalizePwm(currentOutput);
          System.out.println(currentOutput);
          this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
          SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooter);
          SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        }

        if(turretEncoderReadingPosition >= setPointRotation - 30000 && turretEncoderReadingPosition <= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - 30000){
          turretMotor.set(ControlMode.PercentOutput, 0.3);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, -0.3);
          turretReady = false;
        }
      
        if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretHoodPosition <= setPointHood - 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0.3);
          turretReady = false;
        }else if (turretHoodPosition >= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, -0.3);
          turretReady = false;
        }

    }

    @Override
    public void shootCloseLaunch() {
        // TODO Auto-generated method stub
        System.out.println("CLOSE LAUNCHPAD SHOT: ");
        setPointHood = 0; // to be decided
        // setPointShooter = 0; // tbd
        setPointShooterPID = 0; // tbd
        setPointRotation = 0; // tbd
        System.out.println("SETPOINT HOOD: " + setPointHood);
        System.out.println("SETPOINT SHOOTER " + setPointShooter);
        System.out.println("SETPOINT FOR THE TURRET TURNING " + setPointRotation);

        // Shooter without PID
        // shooterMotor.set(ControlMode.PercentOutput, setPointShooter);
        // SmartDashboard.putNumber("Shooter Output", setPointShooter);

        // shooter with PID
        // pid for shooter
        if(setPointShooter != 0){
          currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooter);
          currentOutput = Utils.normalizePwm(currentOutput);
          System.out.println(currentOutput);
          this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
          SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooter);
          SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        }

        if(turretEncoderReadingPosition >= setPointRotation - 30000 && turretEncoderReadingPosition <= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - 30000){
          turretMotor.set(ControlMode.PercentOutput, 0.3);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, -0.3);
          turretReady = false;
        }
      
        if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretHoodPosition <= setPointHood - 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0.3);
          turretReady = false;
        }else if (turretHoodPosition >= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, -0.3);
          turretReady = false;
        }
        
    }

    @Override
    public void shootAuto() {
        // TODO Auto-generated method stub
        // TODO Auto-generated method stub
        System.out.println("CLOSE LAUNCHPAD SHOT: ");
        setPointHood = 0; // to be decided
        // setPointShooter = 0; // tbd
        setPointShooterPID = 0; // tbd
        setPointRotation = 0; // will probably still be 0 for the auto shot
        System.out.println("SETPOINT HOOD: " + setPointHood);
        System.out.println("SETPOINT SHOOTER " + setPointShooter);
        System.out.println("SETPOINT FOR THE TURRET TURNING " + setPointRotation);

        // Shooter without PID
        // shooterMotor.set(ControlMode.PercentOutput, setPointShooter);
        // SmartDashboard.putNumber("Shooter Output", setPointShooter);

        // shooter with PID
        // pid for shooter
        if(setPointShooter != 0){
          currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooter);
          currentOutput = Utils.normalizePwm(currentOutput);
          System.out.println(currentOutput);
          this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
          SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooter);
          SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        }

        if(turretEncoderReadingPosition >= setPointRotation - 30000 && turretEncoderReadingPosition <= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - 30000){
          turretMotor.set(ControlMode.PercentOutput, 0.3);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + 30000){
          turretMotor.set(ControlMode.PercentOutput, -0.3);
          turretReady = false;
        }
      
        if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretHoodPosition <= setPointHood - 20000){
          hoodMotor.set(ControlMode.PercentOutput, 0.3);
          turretReady = false;
        }else if (turretHoodPosition >= setPointHood + 20000){
          hoodMotor.set(ControlMode.PercentOutput, -0.3);
          turretReady = false;
        }
    }
}
