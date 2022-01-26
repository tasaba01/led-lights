package frc.robot.subsystems.BallPath.Intake;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorMatch;

public class IntakeImpl extends RepeatingPooledSubsystem implements Intake {

    private final WPI_TalonSRX intake;
    private boolean primed;
    private int speed = 1;

    public IntakeImpl(WPI_TalonSRX intake) {
        super(20, TimeUnit.MILLISECONDS);
        this.intake = intake;
    }

    @Override
    public void defineResources() {
        require(intake);
    }

    @Override
    public void task() throws Exception {
        // TODO Auto-generated method stub
    }

    @Override
    public void start(){
        this.intake.set(this.speed);
    }

    @Override
    public void reverse(){
        this.intake.set(-this.speed);
    }

    @Override
    public void stop(){
        this.intake.set(0);
    }


    // checks if ball is at bottom of elevator
    @Override
    public boolean checkIfPrimed(){
        // if (ballUnderElevator){
        //     this.primed = true;
        // }

        return this.primed;
    }
    
    @Override
    public boolean checkColour(){
        return true;
    }

    // primes a ball
    // @Override
    // public void runInner(int side){
    //     // side: 0 = front
    //     // side: 1 = back
    //     if (side == 0){
    //         this.frontIntake.set(1);
    //     } else if (side == 1){
    //         this.backIntake.set(1);
    //     }
    // }

    // @Override
    // public void stopInner(int side){
    //     // side: 0 = front
    //     // side: 1 = back
    //     if (side == 0){
    //         this.frontIntake.set(0);
    //     } else if (side == 1){
    //         this.backIntake.set(0);
    //     }
    // }
}
