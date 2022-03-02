package frc.robot.subsystems.BallPath.Shooter;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends RepeatingPooledSubsystem implements Shooter {

    private final TalonSRX turretMotor;
    private final TalonFX shooterMotor;
    private final TalonSRX hoodMotor;

    private double kp = 0.00035;
    private double ki = 0.000075;
    private double kd = 0.00003;

    private ShotPosition requestedPosition = ShotPosition.NONE;
    private double setPointHood;
    private double setPointRotation;
    private double setPointShooterPID;

    private double shooterEncoderReadingPosition;
    private double shooterEncoderReadingVelocity;
    private double turretHoodPosition;
    private double turretEncoderReadingPosition;
    private double turretHoodVelocity;
    private double turretEncoderReadingVelocity;

    private boolean turretReady = false;
    private boolean hoodReady = false;

    private double currentOutput;

    private final double hoodBuffer = 20000;
    private final double turretBuffer = 30000;
    private final double turretSpeed = 0.3;
    private final double hoodSpeed = 0.3;

    private final PIDController shooterPid;

    public ShooterImpl(TalonSRX turretMotor, TalonFX shooterMotor, TalonSRX hoodMotor) {
        super(10, TimeUnit.MILLISECONDS);
        this.turretMotor = turretMotor;
        this.shooterMotor = shooterMotor;
        this.hoodMotor = hoodMotor;
        this.shooterPid = new PIDController(kp, ki, kd);
    }

    @Override
    public void defineResources(){
        require(turretMotor);
        require(shooterMotor);
        require(hoodMotor);
    }

    @Override
    public void setShotPosition(ShotPosition shotPosition) {
        this.requestedPosition = shotPosition;
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

        switch (requestedPosition) {
            case AUTO:
                setPointHood = 0; // to be decided
                setPointShooterPID = 0; // tbd
                setPointRotation = 0; // will probably still be 0 for the auto shot
                break;
            case FENDER:
                setPointHood = 100000;
                setPointShooterPID = 0; // tbd
                setPointRotation = 0;
                break;
            case LAUNCHPAD_CLOSE:
                setPointHood = 0; // to be decided
                setPointShooterPID = 0; // tbd
                setPointRotation = 0; // tbd
                break;
            case LAUNCHPAD_FAR:
                setPointHood = 300000;
                setPointShooterPID = 0; // tbd
                setPointRotation = 200000;
                break;
            case NONE:
                setPointShooterPID = 0;
                setPointHood = Double.NEGATIVE_INFINITY;
                setPointRotation = Double.NEGATIVE_INFINITY;
                hoodReady = false;
                turretReady = false;
            default:
                break;
        }

        if (setPointHood == Double.NEGATIVE_INFINITY) {
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodReady = false;
        } else if(turretHoodPosition >= setPointHood - hoodBuffer && turretHoodPosition <= setPointHood + hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodReady = true;
        }else if(turretHoodPosition <= setPointHood - hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, hoodSpeed);
            hoodReady = false;
        }else if (turretHoodPosition >= setPointHood + hoodBuffer){
            hoodMotor.set(ControlMode.PercentOutput, -hoodSpeed);
            hoodReady = false;
        }

        if (setPointRotation == Double.NEGATIVE_INFINITY) {
            turretMotor.set(ControlMode.PercentOutput, 0);
            turretReady = false;
        } else if(turretEncoderReadingPosition >= setPointRotation - turretBuffer && turretEncoderReadingPosition <= setPointRotation + turretBuffer){
            turretMotor.set(ControlMode.PercentOutput, 0);
            turretReady = true;
        }else if(turretEncoderReadingPosition <= setPointRotation - turretBuffer){
            turretMotor.set(ControlMode.PercentOutput, turretSpeed);
            turretReady = false;
        }else if (turretEncoderReadingPosition >= setPointRotation + turretBuffer){
            turretMotor.set(ControlMode.PercentOutput, -turretSpeed);
            turretReady = false;
        }

        if(setPointShooterPID != 0){
            currentOutput = shooterPid.calculate(shooterEncoderReadingVelocity, setPointShooterPID);
            currentOutput = Utils.normalizePwm(currentOutput);
            System.out.println(currentOutput);
            this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
            SmartDashboard.putNumber("Setpoint for the shooter is: ", setPointShooterPID);
            SmartDashboard.putNumber("Current Output is: ", shooterEncoderReadingVelocity);
        } else {
            this.shooterMotor.set(ControlMode.PercentOutput, 0);
        }
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
    public void stopMotors(){
        System.out.println("DISABLING THE SHOOTER");
        setShotPosition(ShotPosition.NONE);
    }
}
