// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax m_leftleader = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_leftfollower = new CANSparkMax(1,  MotorType.kBrushless);
  private CANSparkMax m_rightleader = new CANSparkMax(2,  MotorType.kBrushless);
  private CANSparkMax m_rightfollower = new CANSparkMax(3,  MotorType.kBrushless);
 
  private final MotorControllerGroup left = new MotorControllerGroup(m_leftleader, m_leftfollower);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightleader, m_rightfollower);
  private final DifferentialDrive drive = new DifferentialDrive(left, right);


  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2,3);

  private final AnalogGyro m_gyro = new AnalogGyro(1);

  private DifferentialDriveOdometry m_odometry;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;
  private Field2d m_fieldSim;

//PID
  final double kP = 0.5;
  final double kI = 0.0;
  final double kD = 0.0;

  double m_setPoint = 0.0;
  double m_lastTimeStamp = 0.0;
  double m_lastError = 0.0;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    left.setInverted(true);
    right.setInverted(true);

    m_leftEncoder.setDistancePerPulse(0.1524*Math.PI/1024);
    m_rightEncoder.setDistancePerPulse(0.1524*Math.PI/1024);

    m_odometry = new DifferentialDriveOdometry( //odometry means getting position and location of the robot
      m_gyro.getRotation2d(), //initializes it
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
    );

    m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide,
      KitbotGearing.k10p71, 
      KitbotWheelSize.kSixInch, 
      null
    );

    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);
    
    m_leftEncoderSim = new EncoderSim(m_leftEncoder);//connects sim objects with physical ones
    m_rightEncoderSim = new EncoderSim(m_rightEncoder); 
    m_gyroSim = new AnalogGyroSim(m_gyro);


  }
  public void driveArcade(double xForward, double zRotation){
    drive.arcadeDrive(xForward, zRotation);
  }

  public void driveRaw(double power){
    left.set(power);
    right.set(power);
  }

  
  public void turnRight(double amount){
    left.set(-amount);
    right.set(amount);
  }

  public void turnLeft(double amount){
    left.set(amount);
    right.set(-amount);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic(){ //updates subsystem periodically in the simulation
    double position = m_rightEncoder.getDistance();
    double error = m_setPoint - position;
    double dt = Timer.getFPGATimestamp() - m_lastTimeStamp;
    
    double outputSpeed = kP*error;

    left.set(outputSpeed);
    right.set(outputSpeed);

    SmartDashboard.putNumber("position", position);
    SmartDashboard.putNumber("setpoint", m_setPoint);

    m_lastTimeStamp = Timer.getFPGATimestamp();
    m_lastError = error;






    m_driveTrainSim.setInputs(left.get()*RobotController.getBatteryVoltage(), //sets voltage to drivetrain
    right.get()*RobotController.getBatteryVoltage()
    );

    m_driveTrainSim.update(0.02); //updates every .02 seconds
    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters()); //sets distance of the encoder
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());

    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());

    m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());

    m_lastTimeStamp = Timer.getFPGATimestamp() - m_lastTimeStamp;

  }
  public double getHeading(){
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void setSetpoint(double setpoint){

  }
}