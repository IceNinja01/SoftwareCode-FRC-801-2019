package frc.robot.SwerveClass;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities.Utils;

//This code use Ether's calculations to perform the inverse kinamatics equations for swerve RPM
//https://www.chiefdelphi.com/t/paper-4-wheel-independent-RPM-independent-steering-swerve/107383



public class SwerveDrive extends MotorSafety {
	//Variables to be used are set to private
	
	private MotorSafety m_safetyHelper;
	public static final double kDefaultExpirationTime = 0.1;
	public static final double kDefaultMaxOutput = 1.0;

	protected double temp, STR, FWD, RCW;
	protected double A,B,C,D, max;
	protected double L = 27.75; //Lenght of robot wheel base
	protected double W = 27.75; //Width of robot wheel base
	protected double R = Math.sqrt(L*L+W*W);
	protected double timeUs;
	private String motorName[] = {"FrontRight","FrontLeft","BackLeft","BackRight"};

	private double[] oldAngle = {0,0,0,0};
	private double maxRPM = 0.5;

	private SwervePOD[] SwervePOD  = new SwervePOD[4];
    private double[] wheelAngles = new double[4];
	private double[] wheelSpeeds = new double[4];
	private double[] angleJoyStickDiff = new double[4];
	private double[] angleError = new double[4];

	private RollingAverage xavg;
	private RollingAverage yavg;
	private RollingAverage zavg;
	private double[] position = new double[4];
	private double omega; 
	private Timer time = new Timer();
	private double old_time = 0.0;
	private double new_time = 0.0;
	private double dt;
	private double velocity;
	private double new_vel;
	private double old_vel;
	private double old_omega;
	private double new_omega;
	private double delta_vel;
	private double delta_omega;
	
	public  SwerveDrive(SwervePOD rightFrontPOD,
			frc.robot.SwerveClass.SwervePOD leftFrontPod, frc.robot.SwerveClass.SwervePOD leftBackPod, frc.robot.SwerveClass.SwervePOD rightBackPod, int avgSize) {
		
		SwervePOD[0] = rightFrontPOD;
		SwervePOD[1] = leftFrontPod;
		SwervePOD[2] = leftBackPod;
		SwervePOD[3] = rightBackPod;

		
		// Initializes the _avg variables to size avgSize
		xavg = new RollingAverage(avgSize);
		yavg = new RollingAverage(avgSize);
		zavg = new RollingAverage(avgSize);
		
	}

	//Configure each POD Drive motors
	public void configPIDDrive(double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput) {
		for(int i=0;i<4;i++){
			SwervePOD[i].configPIDDrive(kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
		}

	}

	public void currentDriveLimit(int stallLimit, int freeLimit){
		for(int i=0;i<4;i++){
			SwervePOD[i].setDriveCurrentLimit(stallLimit,freeLimit);
		}
	}

	//Configure each POD Turn motors
	public void configPIDTurn(double kP, double kI, double kD, int kIz, double kFF, double kMinOutput, double kMaxOutput, int deadBand) {
		for(int i=0;i<4;i++){
			SwervePOD[i].configPIDTurn(kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput, deadBand);
		}
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
		    for(int i=0;i<4;i++){
		    	degs[i] = SwervePOD[i].getAngleDeg();

		    	 angleError[i] = wheelAngles[i] - degs[i];
				 if(angleError[i]<0)
				 {      
				 	angleError[i] += 360;
				 }

				 angleError[i] -= 180;

				 boolean shouldInvert = true;

				if(angleError[i] > 90 )
				{
					//wraping for 350 -10 = 340, so 360 - 340 =20 
					 angleError[i] -= 180;
					 shouldInvert = false;
				}
				else if ( angleError[i] < -90 ) 
				{
					angleError[i] += 180;
					shouldInvert = false;
				}

				//Turn Motors
			    if(wheelSpeeds[i] > 0.1){
					brakeOff();
					SwervePOD[i].setAngle( Utils.wrapAngle0To360Deg( angleError[i] + degs[i]));
					oldAngle[i] =  Utils.wrapAngle0To360Deg(angleError[i] + degs[i]);
					SwervePOD[i].setSpeed(shouldInvert ? maxRPM*wheelSpeeds[i] : -maxRPM*wheelSpeeds[i]);
				}
				else{
					SwervePOD[i].disablePIDTurn();
					SwervePOD[i].stop();
					brakeOn();
				}
				

			SmartDashboard.putNumber("Angle", wheelAngles[i]);
			SmartDashboard.putNumber("Int", i);
			SwervePOD[i].getAbsAngle();
	    }
			getRobotVelocity_AngularVelocity();
	    	SmartDashboard.putNumber("JoyAngle", angleJoyStickDiff[0]);

		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
      	}

	}
	// Get the speed of the robot and angle from motor values. The angle is calculated from motors.
	// Use gyro for true angle measurements.
	private void getSpeed() {
		double vel_X = 0;
		double vel_Y = 0;
		velocity = 0;
		for(int i=0;i<4;i++){
			vel_X += SwervePOD[i].getSpeed()*Math.cos(Utils.convertDegtoRad(SwervePOD[i].getAngleDeg()));
			vel_Y += SwervePOD[i].getSpeed()*Math.sin(Utils.convertDegtoRad(SwervePOD[i].getAngleDeg()));
		}
		vel_X /= 4;
		vel_Y /= 4;
		omega = Utils.wrapAngle0To360Deg(Math.atan2(vel_Y,vel_X)*180/Math.PI);
		velocity = Math.sqrt(vel_Y*vel_Y + vel_X*vel_X);
		SmartDashboard.putNumber("Velocity", velocity);
		SmartDashboard.putNumber("Angle_Omega", omega);
	}

	private void getRobotVelocity_AngularVelocity(){
		getSpeed();
		old_time = new_time;
		new_time = Timer.getFPGATimestamp();
		dt = new_time-old_time;
		old_vel = new_vel;
		new_vel = velocity;
		old_omega = new_omega;
		new_omega = omega;
		delta_vel = new_vel - old_vel;
		delta_vel /= dt;
		delta_omega = new_omega - old_omega;
		delta_omega /=dt;
	}

	public void getRobotDisplacedPosition(){

	}

	public void getUpdate(){
		for(int i=0;i<4;i++){
		SwervePOD[i].getSpeed();
		SwervePOD[i].getAngleDeg();
		}
	}

	public void getEncoder(){
		
		for(int i=0; i < 4; i++){
			position[i] = SwervePOD[i].getPosition();
		}
	}

	public boolean isDistance(double setPoint){
		double avg = 0;
		getEncoder();
		for(int i=0; i < 4; i++){
			avg += position[i];
		}
		avg /=4;
		if(avg>=setPoint){
			return true;
		}
		else{
			return false;
		}
	}

	public void turnMotors(double angle_CMD) {
	    for(int i=0;i<4;i++){
	    	SwervePOD[i].setAngle(angle_CMD);
	    }
	}

	//Used driving by Command, velocity and angle
	public void turnMotorsRPM(double angle_CMD , double speed){
	    for(int i=0;i<4;i++){
	    	SwervePOD[i].setAngle(angle_CMD);
	    	SwervePOD[i].setSpeed(-maxRPM*speed);
	    }
	}

	public void getAmps() {
		for(int i=0; i<4; i++) {
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
		for(int i=0;i<4;i++){
			SwervePOD[i].setDriveCurrentLimit(stallLimit, freeLimit);
		}
	}

	public void setTurnCurrentLimit(int peakAmps, int durationMs, int continousAmps) {
	/* Peak Current and Duration must be exceeded before current limit is activated.
	When activated, current will be limited to Continuous Current.
	Set Peak Current params to 0 if desired behavior is to immediately current-limit. */
		for(int i=0;i<4;i++){
			SwervePOD[i].setTurnCurrentLimit(peakAmps, durationMs, continousAmps);
		}
	}

	@Override
	public void stopMotor() {
		for(int i=0;i<4;i++){
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
		for(int i=0;i<4;i++){
		    if (SwervePOD[i] != null) {
			  SwervePOD[i].brakeOn();
		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}

	public void brakeOff() {
		for(int i=0;i<4;i++){
		    if (SwervePOD[i] != null) {
		    	  SwervePOD[i].brakeOff();
		    }
		}
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	}

	public void getDriveVoltage() {
		for(int i=0;i<4;i++){
			SwervePOD[i].getDriveVoltage();
		}
	}

	public void getTurnVoltage() {
	}

	
}
