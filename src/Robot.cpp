#include "Robot.hpp"

#include "Client.hpp"
#include "CommunicationService.hpp"
#include "Goal.hpp"
#include "LaserDistanceSensor.hpp"
#include "Logger.hpp"
#include "MainApplication.hpp"
#include "MathUtils.hpp"
#include "Message.hpp"
#include "MessageTypes.hpp"
#include "RobotWorld.hpp"
#include "Server.hpp"
#include "Shape2DUtils.hpp"
#include "Wall.hpp"
#include "WayPoint.hpp"

#include <chrono>
#include <ctime>
#include <sstream>
#include <thread>
#include <random>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <math.h>
#include <cmath>

	struct believePos {
		float posX;
		float posY;
	};

namespace Model {
/**
 *
 */
Robot::Robot() :
		name(""), size( DefaultSize), position( DefaultPosition), front(0, 0), speed(
				0.0), acting(false), driving(false), communicating(false) {
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
	currentFilter = 0;
	getInformationMap();
	createParticlesDone = false;
}
/**
 *
 */
Robot::Robot(const std::string &aName) :
		name(aName), size( DefaultSize), position( DefaultPosition), front(0,
				0), speed(0.0), acting(false), driving(false), communicating(
				false) {
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
	currentFilter = 0;
	getInformationMap();
	createParticlesDone = false;
}
/**
 *
 */
Robot::Robot(const std::string &aName, const Point &aPosition) :
		name(aName), size( DefaultSize), position(aPosition), front(0, 0), speed(
				0.0), acting(false), driving(false), communicating(false) {
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
	currentFilter = 0;
	getInformationMap();
	createParticlesDone = false;
}
/**
 *
 */
Robot::~Robot() {
	if (driving) {
		stopDriving();
	}
	if (acting) {
		stopActing();
	}
	if (communicating) {
		stopCommunicating();
	}
}

void Robot::getInformationMap() {
	wayPoints = RobotWorld::getRobotWorld().getWayPoints();
	goals = RobotWorld::getRobotWorld().getGoals();
	walls = RobotWorld::getRobotWorld().getWalls();
}

/**
 *
 */
void Robot::setName(const std::string &aName,
		bool aNotifyObservers /*= true*/) {
	name = aName;
	if (aNotifyObservers == true) {
		notifyObservers();
	}

}
/**
 *
 */
Size Robot::getSize() const {
	return size;
}
/**
 *
 */
void Robot::setSize(const Size &aSize, bool aNotifyObservers /*= true*/) {
	size = aSize;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
void Robot::setPosition(const Point &aPosition,
		bool aNotifyObservers /*= true*/) {
	position = aPosition;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
BoundedVector Robot::getFront() const {
	return front;
}
/**
 *
 */
void Robot::setFront(const BoundedVector &aVector,
		bool aNotifyObservers /*= true*/) {
	front = aVector;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
float Robot::getSpeed() const {
	return speed;
}
/**
 *
 */
void Robot::setSpeed(float aNewSpeed, bool aNotifyObservers /*= true*/) {
	speed = aNewSpeed;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
void Robot::startActing() {
	acting = true;
	std::thread newRobotThread([this] {
		startDriving();
	});
	robotThread.swap(newRobotThread);
}
/**
 *
 */
void Robot::stopActing() {
	acting = false;
	driving = false;
	robotThread.join();
}
/**
 *
 */
void Robot::startDriving() {
	driving = true;

	goal = RobotWorld::getRobotWorld().getGoal("Goal");
	calculateRoute(goal);

	drive();
}
/**
 *
 */
void Robot::stopDriving() {
	driving = false;
}
/**
 *
 */
void Robot::startCommunicating() {
	if (!communicating) {
		communicating = true;

		std::string localPort = "12345";
		if (Application::MainApplication::isArgGiven("-local_port")) {
			localPort =
					Application::MainApplication::getArg("-local_port").value;
		}

		if (Messaging::CommunicationService::getCommunicationService().isStopped()) {
			TRACE_DEVELOP( "Restarting the Communication service");
			Messaging::CommunicationService::getCommunicationService().restart();
		}

		server = std::make_shared < Messaging::Server
				> (static_cast<unsigned short>(std::stoi(localPort)), toPtr<
						Robot>());
		Messaging::CommunicationService::getCommunicationService().registerServer(
				server);
	}
}
/**
 *
 */
void Robot::stopCommunicating() {
	if (communicating) {
		communicating = false;

		std::string localPort = "12345";
		if (Application::MainApplication::isArgGiven("-local_port")) {
			localPort =
					Application::MainApplication::getArg("-local_port").value;
		}

		Messaging::Client c1ient("localhost",
				static_cast<unsigned short>(std::stoi(localPort)),
				toPtr<Robot>());
		Messaging::Message message(Messaging::StopCommunicatingRequest, "stop");
		c1ient.dispatchMessage(message);
	}
}
/**
 *
 */
Region Robot::getRegion() const {
	Point translatedPoints[] = { getFrontRight(), getFrontLeft(), getBackLeft(),
			getBackRight() };
	return Region(4, translatedPoints);
}
/**
 *
 */
bool Robot::intersects(const Region &aRegion) const {
	Region region = getRegion();
	region.Intersect(aRegion);
	return !region.IsEmpty();
}
/**
 *
 */
Point Robot::getFrontLeft() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalFrontLeft(x, y);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point frontLeft(
			static_cast<int>((originalFrontLeft.x - position.x)
					* std::cos(angle)
					- (originalFrontLeft.y - position.y) * std::sin(angle)
					+ position.x),
			static_cast<int>((originalFrontLeft.y - position.y)
					* std::cos(angle)
					+ (originalFrontLeft.x - position.x) * std::sin(angle)
					+ position.y));

	return frontLeft;
}
/**
 *
 */
Point Robot::getFrontRight() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalFrontRight(x + size.x, y);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point frontRight(
			static_cast<int>((originalFrontRight.x - position.x)
					* std::cos(angle)
					- (originalFrontRight.y - position.y) * std::sin(angle)
					+ position.x),
			static_cast<int>((originalFrontRight.y - position.y)
					* std::cos(angle)
					+ (originalFrontRight.x - position.x) * std::sin(angle)
					+ position.y));

	return frontRight;
}
/**
 *
 */
Point Robot::getBackLeft() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalBackLeft(x, y + size.y);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point backLeft(
			static_cast<int>((originalBackLeft.x - position.x) * std::cos(angle)
					- (originalBackLeft.y - position.y) * std::sin(angle)
					+ position.x),
			static_cast<int>((originalBackLeft.y - position.y) * std::cos(angle)
					+ (originalBackLeft.x - position.x) * std::sin(angle)
					+ position.y));

	return backLeft;

}
/**
 *
 */
Point Robot::getBackRight() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalBackRight(x + size.x, y + size.y);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point backRight(
			static_cast<int>((originalBackRight.x - position.x)
					* std::cos(angle)
					- (originalBackRight.y - position.y) * std::sin(angle)
					+ position.x),
			static_cast<int>((originalBackRight.y - position.y)
					* std::cos(angle)
					+ (originalBackRight.x - position.x) * std::sin(angle)
					+ position.y));

	return backRight;
}
/**
 *
 */
void Robot::handleNotification() {
	//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

	static int update = 0;
	if ((++update % 200) == 0) {
		notifyObservers();
	}
}
/**
 *
 */
void Robot::handleRequest(Messaging::Message &aMessage) {
	FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

	switch (aMessage.getMessageType()) {
	case Messaging::StopCommunicatingRequest: {
		aMessage.setMessageType(Messaging::StopCommunicatingResponse);
		aMessage.setBody("StopCommunicatingResponse");
		// Handle the request. In the limited context of this works. I am not sure
		// whether this works OK in a real application because the handling is time sensitive,
		// i.e. 2 async timers are involved:
		// see CommunicationService::stopServer and Server::stopHandlingRequests
		Messaging::CommunicationService::getCommunicationService().stopServer(
				12345, true);

		break;
	}
	case Messaging::EchoRequest: {
		aMessage.setMessageType(Messaging::EchoResponse);
		aMessage.setBody("Messaging::EchoResponse: " + aMessage.asString());
		break;
	}
	default: {
		TRACE_DEVELOP(__PRETTY_FUNCTION__ + std::string(": default not implemented"));
		break;
	}
	}
}
/**
 *
 */
void Robot::handleResponse(const Messaging::Message &aMessage) {
	FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

	switch (aMessage.getMessageType()) {
	case Messaging::StopCommunicatingResponse: {
		//Messaging::CommunicationService::getCommunicationService().stop();
		break;
	}
	case Messaging::EchoResponse: {
		break;
	}
	default: {
		TRACE_DEVELOP(__PRETTY_FUNCTION__ + std::string( ": default not implemented, ") + aMessage.asString());
		break;
	}
	}
}
/**
 *
 */
std::string Robot::asString() const {
	std::ostringstream os;

	os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

	return os.str();
}
/**
 *
 */
std::string Robot::asDebugString() const {
	std::ostringstream os;

	os << "Robot:\n";
	os << AbstractAgent::asDebugString();
	os << "Robot " << name << " at (" << position.x << "," << position.y
			<< ")\n";

	return os.str();
}
/**
 *
 */
void Robot::drive() {
	try {
		for (std::shared_ptr<AbstractSensor> sensor : sensors) {
			//sensor->setOn();
		}

		if (speed == 0.0) {
			speed = 10.0;
		}

		unsigned pathPoint = 0;
		while (position.x > 0 && position.x < 1024 && position.y > 0 && position.y < 1024 && pathPoint < path.size()) {
			const PathAlgorithm::Vertex &vertex = path[pathPoint +=	static_cast<int>(speed)];
			front = BoundedVector(vertex.asPoint(), position);
			std::vector<int> newPosition = {position.x, position.y};
			werkelijkeRoute.push_back(newPosition);
			float oldPositionX = position.x;
			float oldPositionY = position.y;
			position.x = vertex.x;
			position.y = vertex.y;

			// control update
			std::vector<int> newPos = controlUpdate();
			position.x += newPos.at(0);
			position.y += newPos.at(1);

			float angle = calculateAngle(getBackRight().x, getBackRight().y, getFrontRight().x, getFrontRight().y);
			compassRobot(angle, 1);

			updateOdometer(oldPositionX, oldPositionY, position.x, position.y, 10);

			if (currentFilter == 0) {
				robotLidar = lidar(position.x, position.y);
				if (createParticlesDone) {
					moveParticles(position.x - oldPositionX, position.y - oldPositionY);
				}
				updateParticleFilter();
			}

			if (arrived(goal) || collision()) {
				Application::Logger::log(
						__PRETTY_FUNCTION__
								+ std::string(": arrived or collision"));
				notifyObservers();
				break;
			}

			notifyObservers();

			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			// this should be the last thing in the loop
			if (driving == false) {
				return;
			}
		} // while

		for (std::shared_ptr<AbstractSensor> sensor : sensors) {
			//sensor->setOff();
		}
	} catch (std::exception &e) {
		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": ") + e.what());
		std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
	} catch (...) {
		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": unknown exception"));
		std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
	}
}
/**
 *
 */
void Robot::calculateRoute(GoalPtr aGoal) {
	path.clear();
	if (aGoal) {
		// Turn off logging if not debugging AStar
		Application::Logger::setDisable();

		front = BoundedVector(aGoal->getPosition(), position);
		//handleNotificationsFor( astar);
		path = astar.search(position, aGoal->getPosition(), size);
		//stopHandlingNotificationsFor( astar);

		Application::Logger::setDisable(false);
	}
}
/**
 *
 */
bool Robot::arrived(GoalPtr aGoal) {
	if (aGoal && intersects(aGoal->getRegion())) {
		return true;
	}
	return false;
}
/**
 *
 */
bool Robot::collision() {
	Point frontLeft = getFrontLeft();
	Point frontRight = getFrontRight();
	Point backLeft = getBackLeft();
	Point backRight = getBackRight();

	const std::vector<WallPtr> &walls = RobotWorld::getRobotWorld().getWalls();
	for (WallPtr wall : walls) {
		if (Utils::Shape2DUtils::intersect(frontLeft, frontRight,
				wall->getPoint1(), wall->getPoint2())
				|| Utils::Shape2DUtils::intersect(frontLeft, backLeft,
						wall->getPoint1(), wall->getPoint2())
				|| Utils::Shape2DUtils::intersect(frontRight, backRight,
						wall->getPoint1(), wall->getPoint2())) {
			return true;
		}
	}
	const std::vector<RobotPtr> &robots =
			RobotWorld::getRobotWorld().getRobots();
	for (RobotPtr robot : robots) {
		if (getObjectId() == robot->getObjectId()) {
			continue;
		}
		if (intersects(robot->getRegion())) {
			return true;
		}
	}
	return false;
}

std::vector<int> Robot::controlUpdate(){
	// control update
	std::vector<int> newPos;
	srand((unsigned int)time(NULL));
	float max = 10.0;
	float randomValue = (((float) rand()/RAND_MAX) * max);
	if(randomValue >= 0 && randomValue <= 7){
		max = 3.0;
		randomValue = (((float) rand()/RAND_MAX) * max);
		int value = randomValue;
		switch (value){
		case 0: newPos.push_back(-1);
		case 1: newPos.push_back(0);
		case 2: newPos.push_back(1);
		}
		max = 3.0;
		randomValue = (((float) rand()/RAND_MAX) * max);
		value = randomValue;
		switch (value){
		case 0: newPos.push_back(-1);
		case 1: newPos.push_back(0);
		case 2: newPos.push_back(1);
		}
	}
	else if(randomValue > 7 && randomValue <= 10){
		max = 5.0;
		randomValue = (((float) rand()/RAND_MAX) * max);
		int value = randomValue;
		switch (value){
		case 0: newPos.push_back(-2);
		case 1: newPos.push_back(-1);
		case 2: newPos.push_back(0);
		case 3: newPos.push_back(1);
		case 4: newPos.push_back(2);
		}
		max = 5.0;
		randomValue = (((float) rand()/RAND_MAX) * max);
		value = randomValue;
		switch (value){
		case 0: newPos.push_back(-2);
		case 1: newPos.push_back(-1);
		case 2: newPos.push_back(0);
		case 3: newPos.push_back(1);
		case 4: newPos.push_back(2);
		}
	}
	return newPos;
}

float Robot::compassRobot(float input, float standaardDeviatie) {
	std::random_device random { };
	std::mt19937 gen { random() };

	std::normal_distribution<> normalDistribution { input, standaardDeviatie };
	float value = normalDistribution(gen);
	if (value > 360) {
		value = 360 - value;
	} else if (value < 0) {
		value = 360 + value;
	}
	Application::Logger::log(std::string(": angle is "+ std::to_string(value)));
	return value;
}

float Robot::calculateAngle(float x1, float y1, float x2,
		float y2) {
float angleDegree = 0;
x1 = x1 -1;
	if(x1 < x2 && y1 >y2){ // 0-90
		float m = (y2-y1) / (x2-x1);
		float radians = atan(m);
		angleDegree = radians * (180.0/3.141592653589793238463);
	}
	else if(x1<x2 && y1<y2){ // 90-180
		float m = (y2-y1) / (x2-x1);
		float radians = atan(m);
		angleDegree = radians * (180.0/3.141592653589793238463) + 90;
	}
	else if(x1>x2 && y1<y2){ // 180-270
		float m = (y2-y1) / (x2-x1);
		float radians = atan(m);
		angleDegree = radians * (180.0/3.141592653589793238463) + 180;
	}
	else if(x1>x2 && y1>y2){ // 270-0
		float m = (y2-y1) / (x2-x1);
		float radians = atan(m);
		angleDegree = radians * (180.0/3.141592653589793238463) + 270;
	}
	else if(x1 == x2 && y1 > y2){
		angleDegree = 0; // 0 graden
	}
	else if(x1 < x2 && y1 == y2){
		angleDegree = 90; // 90 graden
	}
	else if(x1 == x2 && y1 < y2){
		angleDegree = 180; // 180 graden
	}
	else if(x1 > x2 && y1 == y2){
		angleDegree = 270; // 270 graden
	}
	return angleDegree;
}

void Robot::switchFilter() {
	if (currentFilter == true) {
		currentFilter = false;
		Application::Logger::log(
				__PRETTY_FUNCTION__
						+ std::string(": Particle filter is active"));
		notifyObservers();
	} else {
		currentFilter = true;
		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": Kalman filter is active"));
		notifyObservers();
	}
}

std::vector<std::vector<float>> Robot::lidar(float posX, float posY) {
	std::vector<std::vector<float>> AlldataPerDegree;
			double startDegree = 1.0;
			unsigned short maxTimes = 180;
			double pi = 3.14159265359;

			for (unsigned short i = 0; i < maxTimes; ++i) {
				double currentRadian = startDegree * (pi/180);
				float particleX = posX;
				float particleY = posY;
				float particleHelling = tan(currentRadian);
				float particleB = particleY - (particleHelling * particleX);

				std::vector<float> distanceWalls;
				//----------------------Muur----------------------------------------------------------------------
				for (unsigned short j = 0; j< walls.size(); ++j) {
					float wallDeltaX = walls[j]->getPoint2().x - walls[j]->getPoint1().x;
					float wallDeltaY = walls[j]->getPoint2().y - walls[j]->getPoint1().y;
					float wallHelling = (wallDeltaY /  wallDeltaX);
					float wallB = walls[j]->getPoint2().y - (wallHelling * walls[j]->getPoint2().x);
				//----------------------Muur-----------------------------------------------------------------------
				// ax+b = ax+b
					// Bereken afstand
					float deltaAfstandX = 0;
					float deltaAfstandY = 0;

					// a2 + b2 = c2
					float afstand = 0;
					if(particleHelling != wallHelling){
						float helling = wallHelling - particleHelling;
						float b = wallB - particleB;
						float x = b / helling; // X-Coordinaat
						float y = wallHelling * x + wallB;

						if(particleX < x && particleY > y && startDegree > 0 && startDegree < 90){ // 0-90
							if(x <= walls[j]->getPoint2().x && x >= walls[j]->getPoint1().x && y <= walls[j]->getPoint2().y && y >= walls[j]->getPoint1().y){
								deltaAfstandX = particleX - x;
								deltaAfstandY = particleY - y;
//								// a2 + b2 = c2
								afstand = std::sqrt(std::pow(deltaAfstandX,2) + std::pow(deltaAfstandY,2));
							}
						}
						else if(particleX < x && particleY < y && startDegree > 90 && startDegree < 180){ // 90-180
							if(x <= walls[j]->getPoint2().x && x >= walls[j]->getPoint1().x && y <= walls[j]->getPoint2().y && y >= walls[j]->getPoint1().y){
								deltaAfstandX = particleX - x;
								deltaAfstandY = particleY - y;
//								// a2 + b2 = c2
								afstand = std::sqrt(std::pow(deltaAfstandX,2) + std::pow(deltaAfstandY,2));
							}
						}
						else if(particleX > x && particleY < y && startDegree > 180 && startDegree < 270){ // 180-270
							if(x <= walls[j]->getPoint2().x && x >= walls[j]->getPoint1().x && y <= walls[j]->getPoint2().y && y >= walls[j]->getPoint1().y){
								deltaAfstandX = particleX - x;
								deltaAfstandY = particleY - y;
								// a2 + b2 = c2
								afstand = std::sqrt(std::pow(deltaAfstandX,2) + std::pow(deltaAfstandY,2));
							}
						}
						else if(particleX > x && particleY > y && startDegree > 270 && startDegree < 360){ // 270-360
							if(x <= walls[j]->getPoint2().x && x >= walls[j]->getPoint1().x && y <= walls[j]->getPoint2().y && y >= walls[j]->getPoint1().y){
								deltaAfstandX = particleX - x;
								deltaAfstandY = particleY - y;
								// a2 + b2 = c2
								afstand = std::sqrt(std::pow(deltaAfstandX,2) + std::pow(deltaAfstandY,2));
							}
							}
					}
					if(afstand > 0){
					}
					distanceWalls.push_back(afstand);
				}
				AlldataPerDegree.push_back(distanceWalls);
			startDegree += 2;
			}
	return AlldataPerDegree;
}

void Robot::createParticles() {
	srand((unsigned int)time(NULL));
	 float maxValue = 1024.0; // Max size map
	 float posX;
	 float posY;
	 for (int i=0;i<200;i++) {
		 posX = (float(rand())/float((RAND_MAX)) * maxValue);
		 posY = (float(rand())/float((RAND_MAX)) * maxValue);
		 Particle p;
		 p.posX = posX;
		 p.posY = posY;
		 p.weight = 0.0;
		 particles.push_back(p);
	 }
	 Application::Logger::log("Particles created");
	 	notifyObservers();
	 createParticlesDone = true;
	 }

void Robot::updateParticleFilter() {
	if (!createParticlesDone) {
		createParticles();
	}
	updateWeights();
	resampleParticles();
}

void Robot::updateWeights() {
	for (unsigned short particle = 0; particle < particles.size(); ++particle) {
				float weight = 0;
				particles[particle].lidar = lidar(particles[particle].posX, particles[particle].posY);
				for (unsigned short graden = 0; graden < particles.at(particle).lidar.size(); ++graden) {
					for(unsigned short walls = 0; walls < particles.at(particle).lidar.at(graden).size(); ++walls){
					float diff = sqrt(pow(robotLidar.at(graden).at(walls) - particles.at(particle).lidar.at(graden).at(walls), 2));
					weight += diff;
					}
				}
				particles[particle].weight = weight;
			}
}

void Robot::resampleParticles() {
	std::sort(particles.begin(), particles.end());
	std::vector<int> predictedPosition = {particles[0].posX, particles[0].posY};
	particleRoute.push_back(predictedPosition);
	Application::Logger::log("Voorspelde locatie: X: " + std::to_string(particles[0].posX) + " Y: " + std::to_string(particles[0].posY));

	for (unsigned short i = 4; i < particles.size(); ++i) {
					unsigned short randomNumber = rand() % 4;
					std::random_device rd{};
					std::mt19937 gen{rd()};

					switch(randomNumber){
					case 0:{
						std::normal_distribution<> d{particles.at(0).posX,10};
						std::normal_distribution<> d2{particles.at(0).posY,10};
						particles.at(i).posX =	d(gen);
						particles.at(i).posY =	d2(gen);
					break;
					}
					case 1:{
						std::normal_distribution<> d{particles.at(1).posX,10};
						std::normal_distribution<> d2{particles.at(1).posY,10};
						particles.at(i).posX =	d(gen);
						particles.at(i).posY =	d2(gen);
						break;
					}
					case 2:{
						std::normal_distribution<> d{particles.at(2).posX,10};
						std::normal_distribution<> d2{particles.at(2).posY,10};
						particles.at(i).posX =	d(gen);
						particles.at(i).posY =	d2(gen);
						break;
					}
					case 3:{
						std::normal_distribution<> d{particles.at(3).posX,10};
						std::normal_distribution<> d2{particles.at(3).posY,10};
						particles.at(i).posX =	d(gen);
						particles.at(i).posY =	d2(gen);
						break;
					}
					}
				}
}

void Robot::moveParticles(float x, float y) {
	for (unsigned short i = 0; i < particles.size(); ++i) {
		std::random_device random{};
		std::mt19937 generate{random()};
		std::normal_distribution<> nd1{x,5};
		std::normal_distribution<> nd2{y,5};
		particles.at(i).posX +=	nd1(generate);
		particles.at(i).posY +=	nd2(generate);
	}
}

void Robot::updateOdometer(float oldX, float oldY, float newX, float newY, float standaardDeviatie){
	float verschilX = newX - oldX;
	float verschilY = newY - oldY;
	// a2 + b2 = c2
	float langeZijde = std::sqrt(std::pow(verschilX,2) + std::pow(verschilY ,2));
	odometer += langeZijde;
	std::random_device rd{};
	std::mt19937 gen{rd()};
	std::normal_distribution<> d{odometer,standaardDeviatie};
	odometer = d(gen);
	Application::Logger::log("Odometer: " + std::to_string(odometer));
}



float Robot::getOdometer() {
	return odometer;
}

bool Robot::getCurrentFilter() {
		return currentFilter;
	}

} // namespace Model
