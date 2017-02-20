#include "WPILib.h"
#include "MyJoystick.h"

#define ANALOG0 0
#define ANALOG1 1
#define ANALOG2 2

#define USB0 0
#define USB1 1
#define USB2 2

#define PWM0 0
#define PWM1 1
#define PWM2 2
#define PWM3 3
#define PWM4 4
#define PWM5 5

#define PCM0 0
#define PCH0 0
#define PCH1 1
#define PCH2 2
#define PCH3 3
#define PCH4 4
#define PCH5 5

#define DIO0 0
#define DIO1 1
#define DIO2 2
#define DIO3 3
#define DIO4 4
#define DIO5 5
#define DIO6 6
#define DIO7 7
#define DIO8 8
#define DIO9 9

#define PULSEPERIN 0
#define PULSEPERCM 0

class Robot : public IterativeRobot {
public:
	Joystick left;
	Joystick right;
	Joystick controller;

	Spark leftDriveA;
	Spark leftDriveB;
	Spark rightDriveA;
	Spark rightDriveB;
	Spark ropeClimber;
	Spark ballShooter;

	DoubleSolenoid leftGearBox;
	DoubleSolenoid rightGearBox;
	DoubleSolenoid gearPusher;

	Compressor compressor;

	MyJoystick* rightJoystick;
	MyJoystick* leftJoystick;
	MyJoystick* handheld;

	DigitalInput* ropeLimit;
	AnalogInput leftUltrasonic;
	AnalogInput rightUltrasonic;
	AnalogInput centerUltrasonic;

	Encoder leftEncoder;
	Encoder rightEncoder;

	std::shared_ptr<NetworkTable> table;
	std::vector<double> xCoords;
	double xCoordA;
	double xCoordB;

	bool autoAiming;
	bool driveSwapped;
	bool enabled;
	bool pressureStatus;
	float current;

	bool ultrasonicAligning;
	bool ultrasonicApproaching;
	double minDistance;
	double valueToInches;

	enum Gears {
		up,
		down
	} leftGear, rightGear;

	enum AutoState {
		step0,
		step1,
		step2,
		step3,
		done
	} autoStep;

	Robot() :
		left(USB0),
		right(USB1),
		controller(USB2),
		leftDriveA(PWM0),
		leftDriveB(PWM1),
		rightDriveA(PWM2),
		rightDriveB(PWM3),
		ropeClimber(PWM4),
		ballShooter(PWM5),
		leftGearBox(PCM0, PCH2, PCH3),
		rightGearBox(PCM0, PCH4, PCH5),
		gearPusher(PCM0, PCH0, PCH1),
		compressor(PCM0),
		leftUltrasonic(ANALOG0),
		rightUltrasonic(ANALOG1),
		centerUltrasonic(ANALOG2),
		leftEncoder(DIO6, DIO7),
		rightEncoder(DIO8, DIO9)
	{
		rightJoystick = new MyJoystick();
		leftJoystick = new MyJoystick();
		handheld = new MyJoystick();

		table = NetworkTable::GetTable("GRIP/myContoursReport");
		xCoords = table->GetNumberArray("xCoords", llvm::ArrayRef<double>());
		if (xCoords.size() > 1)
		{
			xCoordA = xCoords[0];
			xCoordB = xCoords[1];
		}
		autoAiming = false;
		current = 0;
		enabled = false;
		pressureStatus = 0;

		minDistance = 14;
		valueToInches = 0.125;
		ultrasonicAligning = false;
		ultrasonicApproaching = false;

		leftDriveA.SetExpiration(0.1);
		leftDriveB.SetExpiration(0.1);
		rightDriveA.SetExpiration(0.1);
		rightDriveB.SetExpiration(0.1);

		driveSwapped = false;
		leftGear = down;
		rightGear = down;

		ropeLimit = new DigitalInput(DIO0);

		autoStep = step0;
	}

	~Robot()
	{
		delete rightJoystick;
		delete leftJoystick;
		delete handheld;
	}

	void RobotInit()
	{
		CameraServer::GetInstance()->StartAutomaticCapture("cam0", 0);
		enabled = compressor.Enabled();
		pressureStatus = compressor.GetPressureSwitchValue();
		current = compressor.GetCompressorCurrent();
		compressor.SetClosedLoopControl(true);
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{
//		if (autoStep == step0)
//		{
//			ResetEverything();
//			autoStep = step1;
//		}
//		else if (autoStep == step1)
//		{
//			MoveForward(1.0, 12, PULSEPERIN);
//		}

//		if (autoStep == step0)
//		{
//			ResetEverything();
//			autoStep = step1;
//		}
//		else if (autoStep == step1)
//		{
//			AlignWithGear(leftUltrasonic.GetValue() - rightUltrasonic.GetValue(), step2);
//		}
//		else if (autoStep == step2)
//		{
//			ApproachGear(centerUltrasonic.GetValue() * valueToInches, done);
//		}
//		else if (autoStep == done)
//		{
//			ResetEverything();
//		}

		ultrasonicAligning = true;
		AlignWithGear(leftUltrasonic.GetValue() - rightUltrasonic.GetValue(), done);
	}

	void ResetEverything()
	{
		leftEncoder.Reset();
		rightEncoder.Reset();
		//GYRO RESET CODE HERE
	}
	void MoveForward(double speed, double distance, double pulsemod)
	{
		if (leftEncoder.Get() < distance * pulsemod)
		{
			SetMotors(speed, speed);
		}
		else
		{
			SetMotors(0, 0);
		}
	}

	void TurnStandalone (bool direction, double speed)
	{
		if (direction) //TURN RIGHT
		{
			SetMotors(-speed, speed);
		}
		else
		{
			SetMotors(speed, -speed);
		}
	}

	void TurnGyro(bool direction, double angle, double speed)
	{
		if (direction) //TURN RIGHT
		{
			/* if () //FOR GYRO CODE
			 * {
			 * 	SetMotors(speed, speed);
			 * }
			 * else
			 * {
			 * 	SetMotors(0, 0);
			 * }
			 */
		}
		else //TURN LEFT
		{
			/* if () //FOR GYRO CODE
			 * {
			 * 	SetMotors(-speed, -speed);
			 * }
			 * else
			 * {
			 * 	SetMotors(0, 0);
			 * }
			 */
		}
	}

	void AlignWithGear (double difference, AutoState nextStep)
	{
		if (ultrasonicAligning)
		{
			int turning = 0;
			ShiftDown();
			if (difference > 50)
			{
				turning = 1;
				TurnStandalone(false, 0.33); //TURN RIGHT
			}
			else if (difference < -50)
			{
				turning = 2;
				TurnStandalone(true, 0.33); //TURN LEFT
			}
			else
			{
				turning = 0;
				SetMotors(0, 0);
				ultrasonicAligning = false;
				autoStep = nextStep;
			}

			SmartDashboard::PutNumber("L=1 R=2", turning);
		}
	}

	void ApproachGear (double distance, AutoState nextStep)
	{
		if (ultrasonicApproaching)
		{
			ShiftDown();
			if (distance > minDistance)
			{
				SetMotors(0.45, 0.45);
			}
			else
			{
				SetMotors(0, 0);
				ultrasonicApproaching = false;
				autoStep = nextStep;
			}
		}
	}
	void TeleopInit()
	{
		leftJoystick->init(&left);
		rightJoystick->init(&right);
		handheld->init(&controller);

		driveSwapped = false;
	}

	void TeleopPeriodic()
	{
		leftJoystick->readJoystick();
		rightJoystick->readJoystick();
		handheld->readJoystick();

		ManualGearAim(handheld->readButton(1));
		SwapDrive(rightJoystick->readButton(1), leftJoystick->readButton(1));
		SmartDashboard::PutBoolean("Drive Swapped?", driveSwapped);
		DualTankDrive();
		ManualShiftGears(rightJoystick->readButton(6),rightJoystick->readButton(4));

		PushGear(handheld->readButton(6), handheld->readButton(8));
		ClimbRope(handheld->readButton(4), handheld->readButton(2));
		BallShooter(handheld->readButton(5), handheld->readButton(7));

		SmartDashboard::PutNumber("Left Ultrasonic", leftUltrasonic.GetValue());
		SmartDashboard::PutNumber("Right Ultrasonic", rightUltrasonic.GetValue());
		SmartDashboard::PutNumber("Difference", leftUltrasonic.GetValue() - rightUltrasonic.GetValue());
	}

	void SetMotors(double left, double right){
		leftDriveA.Set(left);
		leftDriveB.Set(left);
		rightDriveA.Set(right);
		rightDriveB.Set(right);
	}

	void ManualGearAim(bool aimButton){
		ShiftDown();
		std::vector<double> xCoords = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
		SmartDashboard::PutNumberArray("X Coords", xCoords);
	//	std::vector<double> areas = table->GetNumberArray("area", llvm::ArrayRef<double>());
		if (xCoords.size() > 1 && aimButton == true){
			SmartDashboard::PutBoolean("Is Autoaiming?", true);
			double xCoordA = xCoords[0];
			double xCoordB = xCoords[1];
			SmartDashboard::PutNumber("xCoordA", xCoordA);
			SmartDashboard::PutNumber("xCoordB", xCoordB);
			double averageXCoord = (xCoordB + xCoordA)/2;
			SmartDashboard::PutNumber("Average X Coord", averageXCoord);
			bool isLeft = false;
			bool isRight = false;
			bool isCenter = false;
			if (averageXCoord < 260){
				autoAiming = true;
				isLeft = true;
				isRight = false;
				isCenter = false;
				leftDriveA.Set(-0.45);
				leftDriveB.Set(-0.45);
				rightDriveA.Set(-0.45);
				rightDriveB.Set(-0.45);
			}
			else if (averageXCoord > 380){
				autoAiming = true;
				isLeft = false;
				isRight = true;
				isCenter = false;
				rightDriveA.Set(0.45);
				rightDriveB.Set(0.45);
				leftDriveA.Set(0.45);
				leftDriveB.Set(0.45);
			}
			else
			{
				isLeft = false;
				isRight = false;
				isCenter = true;
				rightDriveA.Set(0.0);
				rightDriveB.Set(0.0);
				leftDriveA.Set(0.0);
				leftDriveB.Set(0.0);
				autoAiming = false;
			}
			SmartDashboard::PutBoolean("Is Left?", isLeft);
			SmartDashboard::PutBoolean("Is Right?", isRight);
			SmartDashboard::PutBoolean("Is Center?", isCenter);
		}
		else
		{
			autoAiming = false;
		}
	}

	void SwapDrive(bool right, bool left) {
		if (right)
		{
			if (driveSwapped)
			{
				driveSwapped = false;
			}
		}
		if (left)
		{
			if (!driveSwapped)
			{
				driveSwapped = true;
			}
		}
	}

	void DualTankDrive()
	{
		if (!driveSwapped)
		{
			if(fabs(left.GetRawAxis(1)) > 0.3) {
				leftDriveA.Set(left.GetRawAxis(1) * -0.65);
				leftDriveB.Set(left.GetRawAxis(1) * -0.65);
			}
			else if(!autoAiming)
			{
				leftDriveA.Set(0);
				leftDriveB.Set(0);
			}

			if(fabs(right.GetRawAxis(1)) > 0.3) {
				rightDriveA.Set(right.GetRawAxis(1) * -0.65);
				rightDriveB.Set(right.GetRawAxis(1) * -0.65);
			}
			else if(!autoAiming)
			{
				rightDriveA.Set(0);
				rightDriveB.Set(0);
			}
		}
		else if (driveSwapped)
		{
			if(fabs(left.GetRawAxis(1)) > 0.3) {
				rightDriveA.Set(left.GetRawAxis(1) * 0.65);
				rightDriveB.Set(left.GetRawAxis(1) * 0.65);
			}
			else if(!autoAiming)
			{
				SmartDashboard::PutString("Right", "Inactive");
				rightDriveA.Set(0);
				rightDriveB.Set(0);
			}

			if(fabs(right.GetRawAxis(1)) > 0.3) {
				leftDriveA.Set(right.GetRawAxis(1) * 0.65);
				leftDriveB.Set(right.GetRawAxis(1) * 0.65);
			}
			else if(!autoAiming)
			{
				leftDriveA.Set(0);
				leftDriveB.Set(0);
			}
		}
	}

	void ShiftDown()
	{
		if(leftGear == up && rightGear == up)
		{
			leftGearBox.Set(DoubleSolenoid::kReverse);
			rightGearBox.Set(DoubleSolenoid::kReverse);
			leftGear = down;
			rightGear = down;
		}
	}

	void ShiftUp()
	{
		if(leftGear == down && rightGear == down)
		{
			leftGearBox.Set(DoubleSolenoid::kForward);
			rightGearBox.Set(DoubleSolenoid::kForward);
			leftGear = up;
			rightGear = up;
		}
	}
	void ManualShiftGears(bool upButton, bool downButton)
	{
		if(upButton)
		{
			ShiftUp();
		}

		if(downButton)
		{
			ShiftDown();
		}
	}


	void PushGear(bool pushButton, bool retractButton)
	{
		if (pushButton)
		{
			gearPusher.Set(DoubleSolenoid::kForward);
		}

		if (retractButton)
		{
			gearPusher.Set(DoubleSolenoid::kReverse);
		}
	}

	void ClimbRope (bool climbButton, bool descendButton)
	{
		if (climbButton && !descendButton && ropeLimit->Get())
		{
			ropeClimber.Set(1.0);
		}
		else if (!climbButton && descendButton)
		{
			ropeClimber.Set(-1.0);
		}
		else
		{
			ropeClimber.Set(0.0);
		}
	}

	void BallShooter (bool fireButton, bool collectButton)
	{
		if (fireButton && !collectButton)
		{
			ballShooter.Set(1.0);
		}
		else if (!fireButton && collectButton)
		{
			ballShooter.Set(-1.0);
		}
		else
		{
			ballShooter.Set(0.0);
		}
	}
};

START_ROBOT_CLASS(Robot);
