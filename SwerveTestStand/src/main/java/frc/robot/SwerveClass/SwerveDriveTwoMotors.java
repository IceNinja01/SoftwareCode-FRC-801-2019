package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities.Utils;

//This code use Ether's calculations to perform the inverse kinamatics equations for swerve RPM
//https://www.chiefdelphi.com/t/paper-4-wheel-independent-RPM-independent-steering-swerve/107383



public class SwerveDriveTwoMotors extends MotorSafety {
	//Variables to be used are set to private
	
	private MotorSafety m_safetyHelper;
	protected PIDOutput[] pidOutput = new PIDOutput[4]; 
	public static final double kDefaultExpirationTime = 0.1;
	public static final double kDefaultMaxOutput = 1.0;

	protected double temp, STR, FWD, RCW;
	protected double A,B,C,D, max;
	protected double L = 28.5; //Lenght of robot wheel base
	protected double W = 28.5; //Width of robot wheel base
	protected double R = Math.sqrt(L*L+W*W);
	protected double timeUs;
	private String motorName[] = {"FrontRight","FrontLeft","BackLeft","BackRight"};

	private double[] oldAngle = {0,0,0,0};
	private double maxRPM = 5700.0;
	private double maxTurn = 1.0;

	private int deadBand = 1; //
	private SwervePOD[] SwervePOD  = new SwervePOD[4];
    private double[] wheelAngles = new double[4];
	private double[] wheelSpeeds = new double[4];
	private double[] angleJoyStickDiff = new double[4];
	private double[] angleError = new double[4];

	private RollingAverage xavg;
	private RollingAverage yavg;
	private RollingAverage zavg;	
	
	public  SwerveDriveTwoMotors(SwervePOD FrontRightPOD,
			int avgSize) {
		
		SwervePOD[0] = FrontRightPOD;

		
		// Initializes the _avg variables to size avgSize
		xavg = new RollingAverage(avgSize);
		yavg = new RollingAverage(avgSize);
		zavg = new RollingAverage(avgSize);
		
	}
	 /**
	   * RPM method for Swerve wheeled robots.
	   *	  
	   *
	   * <p>This is designed to be directly RPMn by joystick axes.
	   *
	   * @param AxisX         	The speed that the robot should RPM in the X direction. [-1.0..1.0]
	   * @param AxisY         	The speed that the robot should RPM in the Y direction. This input is
	   *                  		inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
	   * @param rotation  		The rate of rotation for the robot that is completely independent of the
	   *                  		translation. [-1.0..1.0]
	   * @param gyroAngle 		The current angle reading from the gyro. Use this to implement field-oriented
	   *                  		controls.
	   */
	public void drive(double AxisX, double AxisY, double rotation, double gyroAngle){
		xavg.add(AxisX);
		yavg.add(AxisY);
		zavg.add(rotation);
		//Calculate Angles and Magnitudes for each motor
		FWD = -yavg.getAverage();
		STR = xavg.getAverage();
		RCW = zavg.getAverage();
		double radians = gyroAngle*Math.PI/180.00;
		temp = FWD*Math.cos(radians) + STR*Math.sin(radians);
		STR = -FWD*Math.sin(radians) + STR*Math.cos(radians);
		FWD = temp;
		//Perform the following calculations for each new set of FWD, STR, and RCW commands:
		A = STR - RCW*(L/R);
		B = STR + RCW*(L/R);
		C = FWD - RCW*(W/R);
		D = FWD + RCW*(W/R);
		
	    wheelSpeeds[0] = Math.sqrt(B*B + C*C);
	    wheelSpeeds[1] = Math.sqrt(B*B + D*D);
	    wheelSpeeds[2] = Math.sqrt(A*A + D*D);
	    wheelSpeeds[3] = Math.sqrt(A*A + C*C);
	    
	    wheelAngles[0] = Utils.wrapAngle0To360Deg(Math.atan2(B,C)*180/Math.PI);
	    wheelAngles[1] = Utils.wrapAngle0To360Deg(Math.atan2(B,D)*180/Math.PI);
	    wheelAngles[2] = Utils.wrapAngle0To360Deg(Math.atan2(A,D)*180/Math.PI);
	    wheelAngles[3] = Utils.wrapAngle0To360Deg(Math.atan2(A,C)*180/Math.PI);
		
	    //Normalize wheelSpeeds
	    //determine max motor speed
	    max=wheelSpeeds[0]; 
	    if(wheelSpeeds[1]>max){
	    	max=wheelSpeeds[1]; 
	    }
	    if(wheelSpeeds[2]>max){
	    	max=wheelSpeeds[2]; 
	    }
	    if(wheelSpeeds[3]>max){
	    	max=wheelSpeeds[3];
	    }
	    //Divide by max motor speeds
	    if(max>1){
	    	wheelSpeeds[0]/=max; 
	    	wheelSpeeds[1]/=max; 
	    	wheelSpeeds[2]/=max; 
	    	wheelSpeeds[3]/=max;
	    }

			double[] degs = new double[4];
		    for(int i=0;i<1;i++){
		    	degs[i] = SwervePOD[i].getAngle();
		    	
		    	angleJoyStickDiff[i]= wheelAngles[i]- oldAngle[i];
		    	angleError[i] = wheelAngles[i] - degs[i];

			    if(Math.abs(angleJoyStickDiff[i]) > 90){ //new angle is greater than a 90degree turn, so find shortest path
			    	//reverse translational motors 
			    	SwervePOD[i].setSpeed(maxRPM*wheelSpeeds[i]);
			    	
			    	//find new angle
			    	wheelAngles[i] -= 180.0; //subtract 180 degrees
			    	if(wheelAngles[i] < 0){ //wrap to new angle between 0-360
			    		wheelAngles[i] += 360.0;
			    	}
			    	//now the angle is set to move to the shortest path, which is just 180 degrees 
			    	//from the current heading
			    	
			    }    
			    
			    else
			    {
			    	SwervePOD[i].setSpeed(-maxRPM*wheelSpeeds[i]);
			    }
				//Turn Motors
			    if(wheelSpeeds[i]>0.1){
			    	SwervePOD[i].setAngle(wheelAngles[i]);
			    	oldAngle[i] = wheelAngles[i];
			    }
		    
		    
	    }
		// getspeed();
	    	SmartDashboard.putNumber("Angle", angleJoyStickDiff[0]);

		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
      	}

	}
	// Get the speed of the robot and angle from motor values. The angle is calculated from motors.
	// Use gyro for true angle measurements.
	private void getspeed() {
		double vel_X = 0;
		double vel_Y = 0;
		double velocity = 0;
		for(int i=0;i<1;i++){
			vel_X += SwervePOD[i].getSpeed()*Math.cos(SwervePOD[i].getAngle());
			vel_Y +=  SwervePOD[i].getSpeed()*Math.sin(SwervePOD[i].getAngle());
		}
		vel_X /= 4;
		vel_Y /= 4;
		double omega = Utils.wrapAngle0To360Deg(Math.atan2(vel_Y,vel_X)*180/Math.PI);
		velocity = Math.sqrt(vel_Y*vel_Y + vel_X*vel_X);
		SmartDashboard.putNumber("Velocity", velocity);
		SmartDashboard.putNumber("Angle_Omega", omega);

	}

	public void turnMotors(double angle_CMD) {
	    for(int i=0;i<1;i++){
	    	SwervePOD[i].setAngle(angle_CMD);
	    }
	}
	
	public void turnMotorsRPM(double angle_CMD , double speed){
	    for(int i=0;i<1;i++){
	    	SwervePOD[i].setAngle(angle_CMD);
	    	SwervePOD[i].setSpeed(-maxRPM*speed);
	    }
	}

	public void getAmps() {
		for(int i=0 ; i<1; i++) {
		   SmartDashboard.putNumber("DriveAmps_"+motorName[i], SwervePOD[i].getDriveAmps());
		   SmartDashboard.putNumber("DriveVolts_"+motorName[i], SwervePOD[i].getDriveVoltage());
		}
	}
	
	public double currentSpeed(SwervePOD motor, int num){
		double speed = motor.getSpeed();
		SmartDashboard.putNumber("Speed"+motorName[num], speed);
		return speed;
	}

	public void setMaxRPM(double maxRPM){
		this.maxRPM = maxRPM;
	}
	
	public double getLR() {
		return L/R;
	}
	
	public double getWR() {
		return W/R;
	}

	public void setWidth(double width){
		this.W = width;
		R = Math.sqrt(L*L+W*W);
	}

	public void setLength(double length){
		this.L = length;
		R = Math.sqrt(L*L+W*W);
	}

	/**
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param freeLimit The current limit at free speed (5700RPM for NEO).
	 */
	public void setDriveCurrentLimit(int stallLimit, int freeLimit) {
		/* Peak Current and Duration must be exceeded before current limit is activated.
		When activated, current will be limited to Continuous Current.
		Set Peak Current params to 0 if desired behavior is to immediately current-limit. */
		for(int i=0;i>1;i++){
			SwervePOD[i].setDriveCurrentLimit(stallLimit, freeLimit);
		}
	}

	public void setTurnCurrentLimit(int peakAmps, int durationMs, int continousAmps) {
	/* Peak Current and Duration must be exceeded before current limit is activated.
	When activated, current will be limited to Continuous Current.
	Set Peak Current params to 0 if desired behavior is to immediately current-limit. */
		for(int i=0;i>1;i++){
			SwervePOD[i].setTurnCurrentLimit(peakAmps, durationMs, continousAmps);
		}
	}

	@Override
	public void stopMotor() {
		for(int i=0;i>1;i++){
		    if (SwervePOD[i] != null) {
			  SwervePOD[i].stop();
		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}

	@Override
	public String getDescription() {
		return null;
	}
	
	public void brakeOn() {
		for(int i=0;i>1;i++){
		    if (SwervePOD[i] != null) {
			  SwervePOD[i].brakeOn();
		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}

	public void brakeOff() {
		for(int i=0;i>1;i++){
		    if (SwervePOD[i] != null) {
		    	  SwervePOD[i].brakeOff();
		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}
	
}
