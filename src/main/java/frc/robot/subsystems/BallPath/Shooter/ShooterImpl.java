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

    private final double hoodBuffer = 20000;
    private final double turretBuffer = 30000;
    private final double turretSpeed = 0.3;
    private final double hoodSpeed = 0.3;


    private final PIDController shooterPid = new PIDController(kp, ki, kd);

    public ShooterImpl(TalonSRX turretMotor, TalonFX shooterMotor, TalonSRX hoodMotor) {
        super(10, TimeUnit.MILLISECONDS);
        // TODO Auto-generated constructor stub
        this.turretMotor = turretMotor;
        this.shooterMotor = shooterMotor;
        this.hoodMotor = hoodMotor;
    }

    @Override
    public void defineResources(){
        require(turretMotor);
        require(shooterMotor);
        require(hoodMotor);

    }

    @Override
    public void task(){
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
    public void findAndCenterTarget() {}

    @Override
    public void centerTarget(double tx){}

    @Override
    public void getDistance(double ty, double angle1, double angle2){}

    // runs flywheel
    @Override
    public boolean readyToShoot(){
        return turretReady && hoodReady;
    }

    @Override
    public int checkBalls(){
        return 0;
    }

    @Override
    public void setHoodAngle(double setPointHood){
        if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          hoodReady = true;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
          hoodReady = false;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
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

        if(turretEncoderReadingPosition >= setPointRotation - turretBuffer && turretEncoderReadingPosition <= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, turretSpeed);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
          turretReady = false;
        }
        
        if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          hoodReady = true;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
          hoodReady = false;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
          hoodReady = false;
        }

    }

    @Override
    public void shootFender() {
        // Fender shot for teleop
        System.out.println("FENDER SHOT: ");
        setPointHood = 100000;
        setPointShooterPID = 0; // tbd
        setPointRotation = 0;
        System.out.println("SETPOINT HOOD: " + setPointHood);
        System.out.println("SETPOINT SHOOTER " + setPointShooterPID);
        System.out.println("SETPOINT FOR THE TURRET TURNING " + setPointRotation);

        // pid for shooter
        if(setPointShooterPID != 0){
          currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
          currentOutput = Utils.normalizePwm(currentOutput);
          System.out.println(currentOutput);
          this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
          SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooterPID);
          SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        }

        if(turretEncoderReadingPosition >= setPointRotation - turretBuffer && turretEncoderReadingPosition <= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, turretSpeed);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
          turretReady = false;
        }
      
        if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          hoodReady = true;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
          hoodReady = false;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
          hoodReady = false;
        }
    }

    @Override
    public void shootFarLaunch() {
        // TODO Auto-generated method stub
        System.out.println("FAR LAUNCHPAD SHOT: ");
        setPointHood = 300000;
        setPointShooterPID = 0; // tbd
        setPointRotation = 200000;
        System.out.println("SETPOINT HOOD: " + setPointHood);
        System.out.println("SETPOINT SHOOTER " + setPointShooterPID);
  

        // shooter with PID
        // pid for shooter
        if(setPointShooterPID != 0){
          currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
          currentOutput = Utils.normalizePwm(currentOutput);
          System.out.println(currentOutput);
          this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
          SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooterPID);
          SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        }

        if(turretEncoderReadingPosition >= setPointRotation - turretBuffer && turretEncoderReadingPosition <= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, turretSpeed);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
          turretReady = false;
        }
      
        if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
          turretReady = false;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
          turretReady = false;
        }

    }

    @Override
    public void shootCloseLaunch() {
        // TODO Auto-generated method stub
        System.out.println("CLOSE LAUNCHPAD SHOT: ");
        setPointHood = 0; // to be decided
        setPointShooterPID = 0; // tbd
        setPointRotation = 0; // tbd
        System.out.println("SETPOINT HOOD: " + setPointHood);
        System.out.println("SETPOINT SHOOTER " + setPointShooterPID);
        System.out.println("SETPOINT FOR THE TURRET TURNING " + setPointRotation);


        // shooter with PID
        if(setPointShooterPID != 0){
          currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
          currentOutput = Utils.normalizePwm(currentOutput);
          System.out.println(currentOutput);
          this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
          SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooterPID);
          SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        }

        if(turretEncoderReadingPosition >= setPointRotation - turretBuffer && turretEncoderReadingPosition <= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, turretSpeed);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
          turretReady = false;
        }
      
        if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
          turretReady = false;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
          turretReady = false;
        }
        
    }

    @Override
    public void shootAuto() {
        // TODO Auto-generated method stub
        // TODO Auto-generated method stub
        System.out.println("CLOSE LAUNCHPAD SHOT: ");
        setPointHood = 0; // to be decided
        setPointShooterPID = 0; // tbd
        setPointRotation = 0; // will probably still be 0 for the auto shot
        System.out.println("SETPOINT HOOD: " + setPointHood);
        System.out.println("SETPOINT SHOOTER " + setPointShooterPID);
        System.out.println("SETPOINT FOR THE TURRET TURNING " + setPointRotation);


        // shooter with PID
        if(setPointShooterPID != 0){
          currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
          currentOutput = Utils.normalizePwm(currentOutput);
          System.out.println(currentOutput);
          this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
          SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooterPID);
          SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        }

        if(turretEncoderReadingPosition >= setPointRotation - turretBuffer && turretEncoderReadingPosition <= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, turretSpeed);
          turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + turretBuffer){
          turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
          turretReady = false;
        }
      
        if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, 0);
          turretReady = true;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
          turretReady = false;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
          hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
          turretReady = false;
        }
    }
}
