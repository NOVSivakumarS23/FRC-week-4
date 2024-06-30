package frc.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Drivetrain;
import frc.robot.Robot;

public class DriveStraight extends Command {
    
    Timer timer;
    Drivetrain drive;

    public DriveStraight(){
        System.out.println("started to drive straight");
        this.drive = Robot.dt;
        this.timer = new Timer();
        this.addRequirements(this.drive);
    }

    @Override
    public void initialize(){
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute(){
        this.drive.drive(0.5, 0);
        System.out.println("currently driving straight");
    }

    @Override
    public boolean isFinished(){
        if(this.timer.get()>=1){
            System.out.println("auton finished");
            return true;
        }else{return false;}
    }
}
