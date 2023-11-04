// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSubsystem extends SubsystemBase {

  //Declare motor controllers
  private final CANSparkMax m_leftleader = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax m_leftfollower = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax m_rightleader = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_rightfollower = new CANSparkMax(16, MotorType.kBrushless);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftleader, m_leftfollower);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightleader, m_rightfollower);
  private final DifferentialDrive drive = new DifferentialDrive(left, right);

  private final Encoder m_leftEncoder = new Encoder(Constants.DriveConstants.kLeftEncoderPorts[0], Constants.DriveConstants.kLeftEncoderPorts[1]);
  private final Encoder m_rightEncoder = new Encoder(Constants.DriveConstants.kRightEncoderPorts[0], Constants.DriveConstants.kRightEncoderPorts[1]);
  
  //Gyro
  private final AnalogGyro m_gyro = new AnalogGyro(1);
  
  //Simulation Stuff
  private DifferentialDriveOdometry m_odometry;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private Field2d m_fieldSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;
  

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    m_leftEncoder.setDistancePerPulse(0.1524 * Math.PI / 1024);
    m_rightEncoder.setDistancePerPulse(0.1524 * Math.PI / 1024);
        
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    //Set up robot simulation
    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      m_rightEncoder.getDistance(), 
      m_leftEncoder.getDistance(),
      new Pose2d(5.0, 5.0, new Rotation2d()));

      
    m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, 
      KitbotGearing.k10p71, 
      KitbotWheelSize.kSixInch, 
      null
    );

    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);

    //Connect the simulators with their counterparts
    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    m_gyroSim = new AnalogGyroSim(m_gyro);

    left.setInverted(true);
    right.setInverted(false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void driveArcade(double xForward, double zRotation){
    drive.arcadeDrive(xForward, zRotation);
  }

  public void driveRaw(double power){
    left.set(power);
    right.set(power);
  }

  public void turnLeft(double amount){
    left.set(-amount);
    right.set(amount);
  }

  public void turnRight(double amount){
    left.set(amount);
    right.set(-amount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update( 
      Rotation2d.fromDegrees(getHeading()),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
    );
    m_fieldSim.setRobotPose(getPose());
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
          //connect the motors to update the drivetrain                   
          m_driveTrainSim.setInputs(                                      
            left.get() * RobotController.getBatteryVoltage(),         
            right.get() * RobotController.getBatteryVoltage()         
          );                                                              
                                                                          
          //Run and update simulation                                     
          m_driveTrainSim.update(0.02);                                   
          m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
          m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
          m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());
          m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());
          m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());
  }

  public double getHeading(){
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }
  
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

}
