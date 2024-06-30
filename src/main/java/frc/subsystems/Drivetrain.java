package frc.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;

public class Drivetrain extends SubsystemBase{
    //k voltage constants
    //encoder ticks
    //wheel distance
    //what is heading and gyro
    //odometry
    //smart dashboard
    //current draw
    //static variable

    //these are like our default stuff for our drivetrain. we have 2 motors, controller, gyro, and odo
    PWMSparkMax left = new PWMSparkMax(0);
    PWMSparkMax right = new PWMSparkMax(1);
    ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    DifferentialDrive dt = new DifferentialDrive(left, right);
    DifferentialDriveOdometry odo;

    //constants
    double kticks = 1024;
    double wheel_dia = 0.15;
    double kdistperpulse = (wheel_dia*Math.PI)/kticks;
    double width = 0.69;
    boolean gyrodir = true;

    //these are constants, dont worry about them for now
    //ksvolts, kvoltsecondspermeter, voltsecondssquaredpermeter, voltsecondsperradian, voltsecondssquaredperradian

    public static final LinearSystem<N2, N2, N2> plant =
        LinearSystemId.identifyDrivetrainSystem(
            1.98,
            0.2,
            1.5,
            0.3);

    //encoders
    Encoder lefte = new Encoder(0,1,false);
    Encoder righte = new Encoder(2,3, true);

    //simulation variables
    DifferentialDrivetrainSim dtsim;
    EncoderSim leftesim;
    EncoderSim rightesim;
    Field2d fieldsim;
    ADXRS450_GyroSim gyrosim;

    //constructor
    public Drivetrain(){
        //setup the real encoders
        lefte.setDistancePerPulse(kdistperpulse);
        righte.setDistancePerPulse(kdistperpulse);
        lefte.reset();
        righte.reset();
        
        //setup real odometry
        double heading = Math.IEEEremainder(gyro.getAngle(), 360)*(gyrodir ? -1.0:1.0);
        odo = new DifferentialDriveOdometry(Rotation2d.fromDegrees(heading), lefte.getDistance(), righte.getDistance());
        
        //setup fake sensors and drivetrain
        dtsim = new DifferentialDrivetrainSim(plant, DCMotor.getFalcon500(1), 8, width, 0.075, null);
        leftesim = new EncoderSim(lefte);
        rightesim = new EncoderSim(righte);
        gyrosim = new ADXRS450_GyroSim(gyro);

        //setup smartdashboard and field
        fieldsim = new Field2d();
        SmartDashboard.putData("Field", fieldsim);
    }

    public void sim_update(){
        double vbat = RobotController.getBatteryVoltage();
        dtsim.setInputs(left.get()*vbat , right.get()*vbat);
        dtsim.update(0.020);

        leftesim.setDistance(dtsim.getLeftPositionMeters());
        leftesim.setRate(dtsim.getLeftVelocityMetersPerSecond());
        rightesim.setDistance(dtsim.getRightPositionMeters());
        rightesim.setRate(dtsim.getRightVelocityMetersPerSecond());
        gyrosim.setAngle(-dtsim.getHeading().getDegrees());
    }
    public void drive(double fwd, double rot){
        //update our dtsim
        sim_update();

        double heading = Math.IEEEremainder(gyro.getAngle(), 360)*(gyrodir ? -1.0:1.0);
        odo.update(Rotation2d.fromDegrees(heading), lefte.getDistance(), righte.getDistance());
        System.out.println(odo.getPoseMeters());
        fieldsim.setRobotPose(odo.getPoseMeters()); 
        
        dt.arcadeDrive(-fwd, -rot);
    }

}
