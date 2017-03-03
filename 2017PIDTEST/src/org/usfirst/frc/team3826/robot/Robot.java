package org.usfirst.frc.team3826.robot;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
//                                          2017 Bot Code
/**
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	RobotDrive ourRobot;
	Joystick xBox;//Joystick
	Relay spike;//Relay named Spike
	Victor climber;
	TalonSRX sweeper;//ROLLER AND SWEEPER ARE TALONSRXs ON COMP BOT
	Victor gear;
	TalonSRX roller;
	CANTalon shooter;

	Encoder leftSide;
	Encoder rightSide;
	
	static final int IMG_WIDTH = 640;
	static final int IMG_HEIGHT = 480;
	VisionThread visionThread;
	double centerX = 0.0;
	double centerRX = 0;
	double centerQX = 0;
	double rCenter = 0;
	double qCenter = 0;
	final Object imgLock = new Object();
	double targetRot;
	
	double setPoint;
	double autoSetPoint;
	double angleDisplace;
	double rotSpeed;
	double angleNow;
	AHRS ahrs;
	
	int autoCounter;
	int autoSetpoint;

	DigitalInput flagOne;
	DigitalInput flagTwo;
	
	int fOne;
	int fTwo;
	
	boolean open;
	boolean inTransition;
	boolean closed;
	
	private int mode = 1;
	private SendableChooser chooser;
	boolean atSpeed;
	

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    
    public void robotInit() {
    	ourRobot = new RobotDrive(0,1);//Drive Train PWM ports are 0 and 1
    	xBox = new Joystick(0);//Xbox controller has an ID of 0 on the driver station
    	spike = new Relay(1);
    	climber = new Victor(3);
    	sweeper = new TalonSRX(2);//ROLLER AND SWEEPER ARE TALONSRXs ON COMP BOT
    	gear = new Victor(5);
    	roller = new TalonSRX(4);
    	shooter = new CANTalon(0);    	
    	flagOne = new DigitalInput(0);
    	flagTwo = new DigitalInput(1);
    	leftSide = new Encoder(2,3);
    	rightSide = new Encoder(4,5);
    	leftSide.setReverseDirection(true);
    	leftSide.reset();
    	rightSide.setReverseDirection(false);
    	rightSide.reset();
    	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        camera.setExposureManual(45);
        camera.setFPS(20);
        
       
        
        visionThread = new VisionThread(camera, new VisionPipelinew(), pipeline -> {
            if (pipeline.filterContoursOutput().size() >= 2) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                Rect q = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
                synchronized (imgLock) {
                    centerRX = r.x + (r.width / 2);
                    centerQX = q.x + (q.width / 2);
                }
            }
            else if(pipeline.filterContoursOutput().size() == 1){
            	Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (imgLock) {
                    centerRX = r.x + (r.width / 2);
                    centerQX = r.x + (r.width / 2);
                }
            }
        });
        visionThread.start();
        
    	ahrs = new AHRS(SPI.Port.kMXP);
    	
    	shooter.changeControlMode(TalonControlMode.Speed);
    	shooter.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
    	shooter.reverseOutput(true);
    	shooter.setPID(.85,0.00005,0);
    	
    	chooser = new SendableChooser();
    	chooser.addDefault("No Auto", 1);
    	chooser.addObject("Right Gear", 2);
    	chooser.addObject("Left Gear", 3);
    	SmartDashboard.putData("Auto Modes", chooser);
    	
    }//
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoCounter = 0;
    	ahrs.reset();
    	leftSide.reset();
    	rightSide.reset();
    	spike.set(Relay.Value.kForward);
    	mode = (int) chooser.getSelected();
    }

    /**
     * This function is called periodically during autonomous
     */
	public void autonomousPeriodic() {
	
		
		if(Math.abs(centerRX - centerQX) > 50 || centerRX == centerQX){
			if(centerRX > centerQX){
				rCenter = centerRX;
				qCenter = centerQX;
			}
			else{
				rCenter = centerQX;
				qCenter = centerRX;
			}
	   	}
			
		centerX = (rCenter+qCenter)/2;
	    targetRot = 0.002 * (320 - centerX);
		
		
		angleNow = ahrs.getAngle();
        angleDisplace = Math.abs(angleNow - autoSetpoint);
        
        if(angleNow > autoSetpoint && angleDisplace > 15){
        	rotSpeed = 0.02 * (angleDisplace+5);
        }
        else if(angleNow > autoSetpoint && angleDisplace > 1.5){
        	rotSpeed = 0.3 + (angleDisplace/100);
        }
        else if(angleNow < autoSetpoint && angleDisplace > 15){
        	rotSpeed = -0.02 * (angleDisplace+5);
        }
        else if(angleNow < autoSetpoint && angleDisplace > 1.5){
        	rotSpeed = -0.3 - (angleDisplace/100);
        }
        else{
        	rotSpeed = 0;
        }
        
        
        switch(mode) {
        	
        
        	default:
        		
        		ourRobot.arcadeDrive(0,0);
        		System.out.println("default");
        		break;
        	
        	case 1:
        		
        		ourRobot.arcadeDrive(0,0);
        		System.out.println("case 1");
        		break;
        		
        	case 2:
        		
        		System.out.println("case 2");
            	autoSetpoint = -60;
        		
        		if(autoCounter == 0){
        			if(leftSide.get() < 500 || rightSide.get() < 500){
        				ourRobot.arcadeDrive(-.5,0);
        			}
        			else{
        				autoCounter = 1;
        			}
        		}
        		else if(autoCounter == 1){
        			ourRobot.arcadeDrive(0,0);
        			Timer.delay(1);
        			autoCounter = 2;
        		}
        		else if(autoCounter == 2){
        			if(rotSpeed != 0){
        				ourRobot.arcadeDrive(0,rotSpeed);
        			}
        			else{
        				Timer.delay(.2);
        				if(rotSpeed != 0){
        					ourRobot.arcadeDrive(0,0);
        					autoCounter = 3;
        				}
        			}
        		}
        		break;
        		
        	case 3:
        		
            	autoSetpoint = 60;
        		
        		if(autoCounter == 0){
        			if(leftSide.get() < 500 || rightSide.get() < 500){
        				ourRobot.arcadeDrive(-.5,0);
        			}
        			else{
        				autoCounter = 1;
        			}
        		}
        		else if(autoCounter == 1){
        			ourRobot.arcadeDrive(0,0);
        			Timer.delay(1);
        			autoCounter = 2;
        		}
        		else if(autoCounter == 2){
        			if(rotSpeed != 0){
        				ourRobot.arcadeDrive(0,rotSpeed);
        			}
        			else{
        				Timer.delay(.2);
        				if(rotSpeed != 0){
        					ourRobot.arcadeDrive(0,0);
        					autoCounter = 3;
        				}
        			}
        		}
        		break;
        	
        }
        
        
        
        
        SmartDashboard.putNumber("Center X",centerX);
    	SmartDashboard.putNumber("Center R",rCenter);
    	SmartDashboard.putNumber("Center Q",qCenter);
    	SmartDashboard.putNumber("Rotation to Target",targetRot);
    	SmartDashboard.putNumber("Angle",angleNow);
    	SmartDashboard.putNumber("Setpoint",setPoint);
    	SmartDashboard.putNumber("Displacement",angleDisplace);
    	SmartDashboard.putNumber("Angle Speed",rotSpeed);
    	SmartDashboard.putNumber("Auto Counter",autoCounter);
    	SmartDashboard.putNumber("Shooter Speed",shooter.getEncVelocity());
    	SmartDashboard.putNumber("FlagOne",fOne);
    	SmartDashboard.putNumber("FlagTwo",fTwo);
    	SmartDashboard.putNumber("Left Side", leftSide.get());
    	SmartDashboard.putNumber("Right Side", rightSide.get());
        
    }
    	
    
    /**
     * This function is called once each time the robot enters teleoperated mode
     */
    public void teleopInit(){
    	ahrs.reset();
    	inTransition = false;
    	leftSide.reset();
    	rightSide.reset();
    	spike.set(Relay.Value.kForward);//Sets the spike to on in the positive direction
    	closed = false;
    	open = false;
    }

    /**
     * This function is called periodically during operator control
     */
    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobot#teleopPeriodic()
     */
    public void teleopPeriodic() {
    	    	
    	//ANGLE
    	angleNow = ahrs.getAngle();
        angleDisplace = Math.abs(angleNow - setPoint);
        
        //Makeshift PID Loop Stand-in
        if(angleNow > setPoint && angleDisplace > 15){
        	rotSpeed = 0.02 * (angleDisplace+5);
        }
        else if(angleNow > setPoint && angleDisplace > 1.25){
        	rotSpeed = 0.4 + (angleDisplace/100);
        }
        else if(angleNow < setPoint && angleDisplace > 15){
        	rotSpeed = -0.02 * (angleDisplace+5);
        }
        else if(angleNow < setPoint && angleDisplace > 1.25){
        	rotSpeed = -0.4 - (angleDisplace/100);
        }
        else{
        	rotSpeed = 0;
        }

        //VISION
		if(Math.abs(centerRX - centerQX) > 50 || (centerRX == centerQX && (centerRX <= 50 || centerRX >= 590))){
			if(centerRX > centerQX){
				rCenter = centerRX;
				qCenter = centerQX;
			}
			else{
				rCenter = centerQX;
				qCenter = centerRX;
			}
    	}
		
		centerX = (rCenter+qCenter)/2;
        targetRot = 0.002 * (320 - centerX);
		
    	
    	if(xBox.getRawButton(4)){//Turn to vision target
    		if(targetRot > .125){
    			if(targetRot < .41){
    				ourRobot.arcadeDrive(0,.41);
    			}
    			else if(targetRot > .45){
    				ourRobot.arcadeDrive(0,.45);
    			}
    			else{
    				ourRobot.arcadeDrive(0,targetRot);
    			}
    		}
    		else if(targetRot < -.125){
    			if(targetRot > -.41){
    				ourRobot.arcadeDrive(0,-.41);
    			}
    			else if(targetRot < -.45){
    				ourRobot.arcadeDrive(0,-.45);
    			}
    			else{
    				ourRobot.arcadeDrive(0,targetRot);
    			}
    		}
    	}
    	else if(Math.abs(xBox.getRawAxis(1)) > .1 || Math.abs(xBox.getRawAxis(0)) > 0.15){//If stick outside deadzone
    		ourRobot.arcadeDrive(xBox.getRawAxis(1), -xBox.getRawAxis(0));//Control the bot with the left stick
    	}
    	else if(Math.abs(xBox.getRawAxis(5)) > .1 || Math.abs(xBox.getRawAxis(4)) > 0.15){//If stick outside deadzone
    		ourRobot.arcadeDrive(-xBox.getRawAxis(5), -xBox.getRawAxis(4));//Control the bot with the right stick
    	}
    	else{//If no buttons are pressed and the left and right sticks are not being moved
    		ourRobot.arcadeDrive(0,0);//Stop the bot
    	}
    	
    	
    	if(xBox.getRawButton(5)){
    		sweeper.set(-.75);
    	}
    	else if(shooter.getEncVelocity() > -17500){
    		sweeper.set(0);
    	}

    	
    	
    	
    	if(xBox.getRawButton(8)){
    		climber.set(1);
    	}
    	else{
    		climber.set(0);
    	}
    	
    	/*
    	//Old Gear Control - Alternate between Open & Closed on B button push
    	if(flagOne.get() && !open){
    		fOne = 1;
    		fTwo = 0;
    		open = true;
    		inTransition = false;
    	}
    	else if(flagTwo.get() && open){
    		fOne = 0;
    		fTwo = 1;
    		open = false;
    		inTransition = false;
    	}
    	else{
    		fOne = 0;
    		fTwo = 0;
    	}
    	
    	if(!inTransition){
    		if(open){
    			if(xBox.getRawButton(2)){
    				gear.set(-1);
    				Timer.delay(0.25);
    				gear.set(-.75);
    				inTransition = true;
    			}
    			else{
    				gear.set(0);
    			}
    		}
    		else{
    			if(xBox.getRawButton(2)){
    				gear.set(1);
    				Timer.delay(0.3);
    				gear.set(.4);
    				inTransition = true;
    			}
    			else{
    				gear.set(0);
    			}
    		}
    	}*/
    	
    	if(xBox.getRawButton(1)){
    		shooter.set(-18850);//Setpoint for PID
    	}
    	else{
    		shooter.set(0);
    	}
    	
    	
    		if(shooter.getEncVelocity() < -18750){//Speed we shoot at
    			atSpeed = true;
    		}
    		else{
    			atSpeed = false;
    		}
    	
    if(xBox.getRawButton(6)){
    	if(atSpeed){
			roller.set(-0.5);
			sweeper.set(-0.5);
			Timer.delay(.15);//How long roller spins
			roller.set(0);
			sweeper.set(0);
			Timer.delay(.05);//Time between spins
			}
    	else{
    		roller.set(0);
    	}
    }
    
    	//Gear Control - Moves to open while holding B
    	if(flagOne.get() && !open){//Right Sensor is detected (When Closed)
    		fOne = 1;//Sets variables for Troubleshooting purposes
    		fTwo = 0;
    		
    		if(!xBox.getRawButton(2)){//When button is not pressed
    			gear.set(0);//Do nothing
    			closed = true;//This prevents the other sensor from interuppting
    		}
    		else{//Button is pressed
    			gear.set(-.4);//Move the flag towards sensor two ("flagTwo")
    			inTransition = true;
    			closed = false;
    		}
    	}
    	else if(flagTwo.get() && !closed){//Left sensor is detected (When Open)
    		fTwo = 1;
    		fOne = 0;
    		
    		if(!xBox.getRawButton(2)){//Button not pressed
    			gear.set(1);//Move the flag towards closed
    			inTransition = true;
    			open = false;
    		}
    		else{//Button is pressed
    			gear.set(0);//Do nothing
    			open = true;//This prevents the other sensor from interuppting
    		}
    	}
    	else if(inTransition){//If neither sensors are detected (Transitioning between opened and closed)
    		fOne = 0;
    		fTwo = 0;
    		if(!xBox.getRawButton(2)){//Button not pressed
    			gear.set(1);//Move towards closed state
    			Timer.delay(.15);//wait
    			gear.set(0.3);//slow down
    			inTransition = false;//Setting this to false causes the motor to continue moving until
    			//either sensor detects the flag
    		}
    		else{//Button is pressed
    			gear.set(-1);//Same as above, but moves towards open
    			Timer.delay(.175);
    			gear.set(-0.5);
    			inTransition = false;
    		}
    	}
    	
    	
    	/*
    	while(open){
    		if(xBox.getRawButton(2)){
    			gear.set(0);
    		}
    		else{
    			gear.set(-1);
    		}
    	}
    	
    	while(closed){
    		if(xBox.getRawButton(2)){
    			gear.set(1);
    		}
    		else{
    			gear.set(0);
    		}
    	}
    	
    	while(inTransition){
    		if(xBox.getRawButton(2)){
    			gear.set(0.75);
    		}
    		else{
    			gear.set(-1);
    		}
    	}
    	*/
    	
    	
    	//PRINTING
    	SmartDashboard.putNumber("Center X",centerX);
    	SmartDashboard.putNumber("Center R",rCenter);
    	SmartDashboard.putNumber("Center Q",qCenter);
    	SmartDashboard.putNumber("Rotation to Target",targetRot);
    	SmartDashboard.putNumber("Angle",angleNow);
    	SmartDashboard.putNumber("Setpoint",setPoint);
    	SmartDashboard.putNumber("Displacement",angleDisplace);
    	SmartDashboard.putNumber("Angle Speed",rotSpeed);
    	SmartDashboard.putNumber("Auto Counter",autoCounter);
    	SmartDashboard.putNumber("Shooter Speed",shooter.getEncVelocity());
    	SmartDashboard.putNumber("FlagOne",fOne);
    	SmartDashboard.putNumber("FlagTwo",fTwo);
    	SmartDashboard.putNumber("Left Side", leftSide.get());
    	SmartDashboard.putNumber("Right Side", rightSide.get());
    	
    	
    	Timer.delay(.04);// wait 40ms to avoid hogging CPU cycles

    }
    
    /**
     * This function is called periodically during test mode
     */
    
    public void testPeriodic() {
    
    if(xBox.getRawButton(7)){//'Select-Back-Options' Button
		gear.set(-.25);
	}
	else{
		gear.set(0);
	}
    if(flagOne.get()){
    	fOne = 1;
    }
    else{
    	fOne = 0;
    }
    if(flagTwo.get()){
    	fTwo = 1;
    }
    else{
    	fTwo = 0;
    }
    System.out.println("FlagOne"+fOne);
	System.out.println("FlagTwo"+fTwo);
    
    
    
    }
       
       
}
