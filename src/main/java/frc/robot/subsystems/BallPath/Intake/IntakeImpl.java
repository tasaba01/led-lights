package frc.robot.subsystems.BallPath.Intake;

import java.util.concurrent.TimeUnit;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

public class IntakeImpl extends RepeatingPooledSubsystem implements Intake{
    
    private WPI_TalonSRX frontIntake;
    private WPI_TalonSRX backIntake;
    private boolean frontRunning;
    private boolean backRunning;

    public IntakeImpl(WPI_TalonSRX frontintake, WPI_TalonSRX backIntake) {
        super(20, TimeUnit.MILLISECONDS);
        this.frontIntake = frontIntake;
        this.backIntake = backIntake;
    }

    @Override
    public void task() throws Exception {
        // TODO Auto-generated method stub
    }

    @Override
    public void defineResources() {
        require(frontIntake);
        require(backIntake);
    }

    public boolean checkColour(){
        return true;
    }
    // checks if ball is held by intake
    public boolean checkBall(){
        return true;
    }
    public void reverse(){

    }
    // checks if ball is at bottom of elevator
    public boolean checkIfPrime(){
        return true;
    }
    // primes a ball
    public void runInnerleft(){

    }
    public void runInnerRight(){

    }
    public void stopInner(){
        
    }
}
