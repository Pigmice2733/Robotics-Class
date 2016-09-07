

#include "newping_distance_sensor.h"
#include "moving_average.h"

#define RUN_TIME 30                     /**< seconds the robot will run */
#define TOO_CLOSE 25                    /**< distance to obstacle in centimeters */
#define MAX_DISTANCE (TOO_CLOSE * 10)   /**< maximum distance to track with sensor */
#define RANDOM_ANALOG_PIN 5             /**< unused analog pin to use as random seed */
#define DISTANCE_SENSOR_INIT 2,3,MAX_DISTANCE

 class PigmiceAutonomousRobot
    {
    public:
      
        PigmiceAutonomousRobot() :            
              distanceSensor(DISTANCE_SENSOR_INIT),
              distanceAverage(TOO_CLOSE * 10)
        {
            initialize();
        }

        void SetDriveTrain(PigmiceDriveTrain& pmDT)
        {
          _pmDT = pmDT;
        }
     
        void initialize()
        {
            randomSeed(analogRead(RANDOM_ANALOG_PIN));
            endTime = millis() + RUN_TIME * 1000;
            move();
        }
        
      
        void run()
        {
            if (stopped())
                return;

            unsigned long currentTime = millis();
            int distance = distanceAverage.add(distanceSensor.getDistance());
      
            
            if (doneRunning(currentTime))
                stop();
            else if (moving()) {
                if (obstacleAhead(distance))
                    turn(currentTime);
            }
            else if (turning()) {
                if (doneTurning(currentTime, distance))
                    move();
            }
        }

    protected:
        void move()
        {
           _pmDT.setMotor1Speed(84);
           _pmDT.setMotor2Speed(84);
            state = stateMoving;
        }
        
        void stop()
        {
           _pmDT.setMotor1Speed(64);
           _pmDT.setMotor2Speed(64);
            state = stateStopped;
        }
        
        bool doneRunning(unsigned long currentTime)
        {
            return (currentTime >= endTime);
        }
        
        bool obstacleAhead(unsigned int distance)
        {
            return (distance <= TOO_CLOSE);
        }
        
        bool turn(unsigned long currentTime)
        {
            if (random(2) == 0) {
                _pmDT.setMotor1Speed(28);
                _pmDT.setMotor2Speed(92);
            }
            else {
                _pmDT.setMotor1Speed(92);
                _pmDT.setMotor2Speed(28);
            }
            state = stateTurning;
            endStateTime = currentTime + random(500, 1000);
        }
        
        bool doneTurning(unsigned long currentTime, unsigned int distance)
        {
            if (currentTime >= endStateTime)
                return (distance > TOO_CLOSE);
            return false;
        }
        
        bool moving() { return (state == stateMoving); }
        bool turning() { return (state == stateTurning); }
        bool stopped() { return (state == stateStopped); }

    private:
        PigmiceDriveTrain _pmDT;
        DistanceSensor distanceSensor;
        MovingAverage<unsigned int, 3> distanceAverage;
        enum state_t { stateStopped, stateMoving, stateTurning };
        state_t state;
        unsigned long endStateTime;
        unsigned long endTime;
    };
