#pragma once
#include <cmath>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <PosGen.hpp>
#include <MotionPlanner.hpp>
#include <algorithm>

#define R_STEP_PIN 33
#define R_DIR_PIN 25
#define T_STEP_PIN 32
#define T_DIR_PIN 22
#define RX_PIN 27
#define TX_PIN 26
#define R_ADDR 1
#define RC_ADDR 0
#define T_ADDR 2


class PolarControl {
  enum State_t : uint8_t {
    UNINITIALIZED,
    INITIALIZED,
    IDLE,
    RUNNING,
    PAUSED
  };

  private:
     static constexpr double TIME_RESOLUTION = .1;
     static constexpr double R_MAX =  450.00;
     static constexpr double T_MAX_VELOCITY =  (2*PI)/10;
     static constexpr double R_MAX_VELOCITY =  (R_MAX)/40;
     static constexpr double T_MAX_ACCEL =  T_MAX_VELOCITY/TIME_RESOLUTION;
     static constexpr double R_MAX_ACCEL =  R_MAX_VELOCITY/TIME_RESOLUTION;
     static constexpr int    STEPS_PER_RADIAN = (400/(2*PI))*(60/16);
     static constexpr int    STEPS_PER_MM =  100;

     static constexpr int    T_MICROSTEPS =  2;
     static constexpr int    T_CURRENT =  800;
     static constexpr double T_MAX_STEP_VELOCITY =  STEPS_PER_RADIAN*T_MAX_VELOCITY;
     static constexpr double T_MAX_STEP_ACCEL =  STEPS_PER_RADIAN * T_MAX_ACCEL;

     static constexpr int    R_MICROSTEPS =  2;
     static constexpr int    R_CURRENT =  500;
     static constexpr double R_MAX_STEP_VELOCITY =  STEPS_PER_MM*R_MAX_VELOCITY;
     static constexpr double R_MAX_STEP_ACCEL =  STEPS_PER_RADIAN * R_MAX_ACCEL;

  public:
    ~PolarControl();
    PolarControl();

    void home();
    void test();
    void setupDrivers();
    bool processNextMove();
    bool start(PosGen *posGen);
    State_t getState();
    bool pause();
    bool resume();
    bool stop();
    void setSpeed(uint8_t speed);
    uint8_t getSpeed() const { return m_speed; }
    PolarCord_t getCurrentPosition() const { return m_currPos; }
    double getMaxRho() const { return R_MAX; }
    void testUartControl();
    void testStepControl();
    void verifyDriverConfig();
    bool loadAndRunFile(String filePath);
    bool loadAndRunFile(String filePath, double maxRho);

  private:
    // Look-ahead buffer for motion planning
    struct PathBuffer {
      static constexpr int BUFFER_SIZE = 10;
      PolarCord_t points[BUFFER_SIZE];
      int head;
      int tail;
      int count;

      PathBuffer() : head(0), tail(0), count(0) {}

      bool isEmpty() const { return count == 0; }
      bool isFull() const { return count == BUFFER_SIZE; }

      void push(PolarCord_t pos);
      PolarCord_t peek(int offset) const;
      PolarCord_t pop();
      void clear();
    };

    // Curvature analysis for speed adaptation
    struct PathCurvature {
      double radius;         // Radius of curvature (mm or rad)
      double angle;          // Angle change (rad)
      double maxSafeSpeed;   // Speed limit for this curve
    };

    State_t                      m_state;
    FastAccelStepperEngine       m_stepperEngine;
    FastAccelStepper             *m_tStepper;
    FastAccelStepper             *m_rStepper;
    TMC2209Stepper               m_tDriver;
    TMC2209Stepper               m_rDriver;
    TMC2209Stepper               m_rCDriver;

    uint8_t                      m_speed = 5; // 1-10
    PolarCord_t                  m_velocity;

    PolarCord_t                  m_dStepMax;
    uint8_t                      m_lastTDir;
    uint8_t                      m_lastRDir;

    PolarCord_t                  m_currPos = {0.0, 0.0};
    PolarCord_t                  m_nextPos = {0.0, 0.0};
    PosGen *                     m_posGen;

    double                       m_currentVelR = 0.0;
    double                       m_currentVelT = 0.0;

    // Look-ahead buffer and motion planning state
    PathBuffer                   m_buffer;
    bool                         m_useScurve = true;
    double                       m_maxJerkT = 60.0;      // rad/s³
    double                       m_maxJerkR = 1000.0;    // mm/s³
    double                       m_maxCentripetalAccel = 50.0;  // mm/s²
    double                       m_minCurveRadius = 5.0;        // mm
    double                       m_centerThreshold = 10.0;      // mm

    // Motion planning methods
    bool calculateMotionProfile(
        double distance,
        double startVel,
        double endVel,
        double maxVel,
        double maxAccel,
        MotionProfile &profile);

    bool synchronizeProfiles(
        MotionProfile &profile1,
        MotionProfile &profile2,
        double targetTime = 0.0);

    bool adjustProfileToTime(MotionProfile &profile, double targetTime);

    double calculateMaxEndVelocity(
        double currentDist,
        double nextDist,
        double currentAngle,
        double nextAngle,
        double maxVel,
        double maxAccel);

    // Look-ahead buffer methods
    void preloadBuffer();

    // Curvature-based speed adaptation
    PathCurvature calculateCurvature(int bufferIndex);
    double adaptSpeedForCurvature(double baseSpeed, const PathCurvature &curve, bool isRadial);

    // S-curve motion profile calculation
    bool calculateSCurveProfile(
        double distance,
        double startVel,
        double endVel,
        double maxVel,
        double maxAccel,
        double maxJerk,
        SMotionProfile &profile);

    // Path smoothing
    PolarCord_t interpolateSpline(double t, PolarCord_t p0, PolarCord_t p1,
                                   PolarCord_t p2, PolarCord_t p3);
    void smoothDirectionChange(PolarCord_t &target);

    // Dynamic time resolution
    double calculateDynamicTimeResolution(double distance);

    // Driver and control methods
    static void commonSetupDriver(TMC2209Stepper &driver);
    static void home(TMC2209Stepper &driver, int speed);
    static void home(TMC2209Stepper &driver);
    int getNumTR(PolarCord_t movement);
    uint8_t move();
    void forceStop();
    uint8_t moveTo(PolarCord_t pos);
};
