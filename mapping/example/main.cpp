#include "Aria.h"
#include "ArMapa.cpp"

int main(int argc, char **argv)
{
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobot robot;
	ArRobotConnector robotConnector(&parser, &robot);
	ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "teleopActionsExample: Could not connect to the robot");
		if (parser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
		}
	}
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}
	ArLog::log(ArLog::Normal, "teleopActionsExample: Connected");
	if (!laserConnector.connectLasers())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}
	robot.runAsync(true);
	robot.enableMotors();
	ArUtil::sleep(3000);
	ArActionLimiterForwards limiter("speed limiter near", 300, 600, 250);
	ArActionLimiterForwards limiterFar("speed limiter far", 300, 1100, 400);
	ArActionLimiterTableSensor tableLimiter;
	ArActionLimiterBackwards backwardsLimiter;
	ArActionJoydrive joydriveAct;
	ArActionKeydrive keydriveAct;
	ArKeyHandler *keyHandler = Aria::getKeyHandler();
	if (keyHandler == NULL)
	{
		keyHandler = new ArKeyHandler;
		Aria::setKeyHandler(keyHandler);
		robot.attachKeyHandler(keyHandler);
	}
	ArGlobalFunctor escapeCB(&escape);
	keyHandler->addKeyHandler('a', &escapeCB);	//when a pressed, interrupt loop
	keyHandler->addKeyHandler('A', &escapeCB);
	keydriveAct.setSpeeds(2000, 20);
	keydriveAct.setIncrements(1000, 50);
	cout << "This program will allow you to use a joystick or keyboard to control the robot" << endl;
	if (!joydriveAct.joystickInited()) cout << "Do not have a joystick, only the arrow keys on the keyboard will work" << endl; // if we don't have a joystick, let 'em know
	//robot.addAction(&limiter, 4);
	//robot.addAction(&limiterFar, 3);
	robot.addAction(&joydriveAct, 2);
	robot.addAction(&keydriveAct, 1);
	joydriveAct.setStopIfNoButtonPressed(true);
	robot.run(true);
	ArMapa mapa;
	ArPose position(3000,1410,90);
	robot.moveTo(position);
	while(fl == 0)
	{
		mapa.mapupdate(robot);
		ArUtil::sleep(200);
	}
	mapa.finalload();
	mapa.save();
	Aria::exit(0);
	return(0);
}
