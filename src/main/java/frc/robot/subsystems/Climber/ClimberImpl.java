package frc.robot.subsystems.Climber;

import java.lang.Thread;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.RobotMap;

public class ClimberImpl implements Climber {
        
    WPI_TalonSRX lifterMotorController;
    DoubleSolenoid climberSolenoid;

    public ClimberImpl() {
        this.lifterMotorController = new WPI_TalonSRX(RobotMap.LIFTER_TALON_PORT);
        this.climberSolenoid = new DoubleSolenoid(RobotMap.CLIMBER_SOLENOID_CHANNELS[0], RobotMap.CLIMBER_SOLENOID_CHANNELS[1]);
    }

    @Override
    public void extendOuterClimber() {
        try {
            this.lifterMotorController.set(1); // test value
            Thread.sleep(5000);
            this.lifterMotorController.set(0);
        } 
        catch(Exception e) { 
            System.out.println(e.getMessage());
        }
    }

    @Override
    public void retractOuterClimber() {
        try {
            this.lifterMotorController.set(1); // test value
            Thread.sleep(5000);
            this.lifterMotorController.set(0);
        } 
        catch(Exception e) {
            System.out.println(e.getMessage());
        }
    }

    @Override
    public void angleOuter(double angle) {}

    @Override
    public void attachInner() {
        try {
            Thread.sleep(650);
        } 
        catch(Exception e) {
            System.out.println(e.getMessage());
        }
    }

}
