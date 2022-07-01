#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "Config.hpp"

#include "AbstractAgent.hpp"
#include "AStar.hpp"
#include "BoundedVector.hpp"
#include "Message.hpp"
#include "MessageHandler.hpp"
#include "Observer.hpp"
#include "Point.hpp"
#include "Size.hpp"
#include "Region.hpp"
#include "RobotWorld.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <cmath>
#include <bits/stdc++.h>
#include <algorithm>

namespace Messaging {
class Message;
class Server;
typedef std::shared_ptr<Server> ServerPtr;
}

namespace Model {
class Robot;
typedef std::shared_ptr<Robot> RobotPtr;

class Goal;
typedef std::shared_ptr<Goal> GoalPtr;

class Robot: public AbstractAgent,
		public Messaging::MessageHandler,
		public Base::Observer {
public:
	/**
	 *
	 */
	Robot();
	/**
	 *
	 */
	explicit Robot(const std::string &aName);
	/**
	 *
	 */
	Robot(const std::string &aName, const Point &aPosition);
	/**
	 *
	 */
	virtual ~Robot();
	/**
	 *
	 */
	std::string getName() const {
		return name;
	}
	/**
	 *
	 */
	void setName(const std::string &aName, bool aNotifyObservers = true);
	/**
	 *
	 */
	Size getSize() const;
	/**
	 *
	 */
	void setSize(const Size &aSize, bool aNotifyObservers = true);
	/**
	 *
	 */
	Point getPosition() const {
		return position;
	}
	/**
	 *
	 */
	void setPosition(const Point &aPosition, bool aNotifyObservers = true);
	/**
	 *
	 */
	BoundedVector getFront() const;
	/**
	 *
	 */
	void setFront(const BoundedVector &aVector, bool aNotifyObservers = true);
	/**
	 *
	 */
	float getSpeed() const;
	/**
	 *
	 */
	void setSpeed(float aNewSpeed, bool aNotifyObservers = true);
	/**
	 *
	 * @return true if the robot is acting, i.e. either planning or driving
	 */
	bool isActing() const {
		return acting;
	}
	/**
	 *
	 */
	virtual void startActing() override;
	/**
	 *
	 */
	virtual void stopActing() override;
	/**
	 *
	 * @return true if the robot is driving
	 */
	bool isDriving() const {
		return driving;
	}
	/**
	 *
	 */
	virtual void startDriving();
	/**
	 *
	 */
	virtual void stopDriving();
	/**
	 *
	 * @return true if the robot is communicating, i.e. listens with an active ServerConnection
	 */
	bool isCommunicating() const {
		return communicating;
	}
	/**
	 * Starts a ServerConnection that listens at port 12345 unless given
	 * an other port by specifying a command line argument -local_port=port
	 */
	void startCommunicating();
	/**
	 * Connects to the ServerConnection that listens at port 12345 unless given
	 * an other port by specifying a command line argument -local_port=port
	 * and sends a message with messageType "1" and a body with "stop"
	 *
	 */
	void stopCommunicating();
	/**
	 *
	 */
	Region getRegion() const;
	/**
	 *
	 */
	bool intersects(const Region &aRegion) const;
	/**
	 *
	 */
	Point getFrontLeft() const;
	/**
	 *
	 */
	Point getFrontRight() const;
	/**
	 *
	 */
	Point getBackLeft() const;
	/**
	 *
	 */
	Point getBackRight() const;
	/**
	 * @name Observer functions
	 */
	//@{
	/**
	 * A Notifier will call this function if this Observer will handle the notifications of that
	 * Notifier. It is the responsibility of the Observer to filter any events it is interested in.
	 *
	 */
	virtual void handleNotification() override;
	//@}
	/**
	 *
	 */
	PathAlgorithm::OpenSet getOpenSet() const {
		return astar.getOpenSet();
	}
	/**
	 *
	 */
	PathAlgorithm::Path getPath() const {
		return path;
	}
	/**
	 * @name Messaging::MessageHandler functions
	 */
	//@{
	/**
	 * This function is called by a ServerSesssion whenever a message is received. If the request is handled,
	 * any response *must* be set in the Message argument. The message argument is then echoed back to the
	 * requester, probably a ClientSession.
	 *
	 * @see Messaging::RequestHandler::handleRequest( Messaging::Message& aMessage)
	 */
	virtual void handleRequest(Messaging::Message &aMessage);
	/**
	 * This function is called by a ClientSession whenever a response to a previous request is received.
	 *
	 * @see Messaging::ResponseHandler::handleResponse( const Messaging::Message& aMessage)
	 */
	virtual void handleResponse(const Messaging::Message &aMessage);
	//@}
	/**
	 * @name Debug functions
	 */
	//@{
	/**
	 * Returns a 1-line description of the object
	 */
	virtual std::string asString() const override;
	/**
	 * Returns a description of the object with all data of the object usable for debugging
	 */
	virtual std::string asDebugString() const override;
	//@}

	float compassRobot(float input, float standaardDeviatie);

	void switchFilter();

	void getInformationMap();

	std::vector<std::vector<float>> lidar(float x, float y);

	struct Particle {
		float posX;
		float posY;
		float weight;
		unsigned id;
		std::vector<std::vector<float>> lidar;
		bool operator < (const Particle& rhs) const {
		        return weight < rhs.weight;
		    }
	};

	struct believePos {
		float posX;
		float posY;
	};

	void createParticles();
	void updateWeights();
	void resampleParticles();
	void moveParticles(float x, float y);
	float getOdometer();
	float calculateAngle(float currentX, float currentY, float newX, float newY);
	void updateParticleFilter();
	bool getCurrentFilter();
	void updateOdometer(float oldX, float oldY, float newX, float newY, float standaardDeviatie);
	std::vector<int> controlUpdate();
	std::vector<Particle> getParticles() {
		return particles;
	}
		bool getcreateParticlesDone() {
			return createParticlesDone;
		}
		std::vector<std::vector<int>> getparticleRoute() {
			return particleRoute;
		}
		std::vector<std::vector<int>> getwerkelijkeRoute() {
			return werkelijkeRoute;
		}
protected:
	/**
	 *
	 */
	void drive();
	/**
	 *
	 */
	void calculateRoute(GoalPtr aGoal);
	/**
	 *
	 */
	bool arrived(GoalPtr aGoal);
	/**
	 *
	 */
	bool collision();
private:
	/**
	 *
	 */
	std::string name;
	/**
	 *
	 */
	Size size;
	/**
	 *
	 */
	Point position;
	/**
	 *
	 */
	BoundedVector front;
	/**
	 *
	 */
	float speed;
	/**
	 *
	 */
	GoalPtr goal;
	/**
	 *
	 */
	PathAlgorithm::AStar astar;
	/**
	 *
	 */
	PathAlgorithm::Path path;
	/**
	 *
	 */
	bool acting;
	/**
	 *
	 */
	bool driving;
	/**
	 *
	 */
	bool communicating;
	/**
	 *
	 */
	std::thread robotThread;
	/**
	 *
	 */
	mutable std::recursive_mutex robotMutex;
	/**
	 *
	 */
	Messaging::ServerPtr server;

	bool currentFilter; //0 particle - 1 kalman
	/**
	 * The vectors are mutable to allow for lazy instantiation
	 */
	mutable std::vector< WayPointPtr > wayPoints;
	mutable std::vector< GoalPtr > goals;
	mutable std::vector< WallPtr > walls;



	bool createParticlesDone;
	std::vector<std::vector<float>> robotLidar;
	std::vector<Particle> particles;
	std::vector<std::vector<int>> particleRoute;
	std::vector<std::vector<int>> werkelijkeRoute;
	float currentAngle;
	float odometer;

};
} // namespace Model
#endif // ROBOT_HPP_
