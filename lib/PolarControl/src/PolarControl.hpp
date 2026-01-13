#pragma once
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <PosGen.hpp>
#include <MotionPlanner.hpp>

// Pin definitions
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
public:
  enum State_t : uint8_t {
    UNINITIALIZED,
    INITIALIZED,
    IDLE,
    RUNNING,
    PAUSED
  };

  PolarControl();
  ~PolarControl();

  // Lifecycle
  void setupDrivers();
  void home();

  // Pattern control
  bool start(PosGen *posGen);
  bool loadAndRunFile(String filePath);
  bool loadAndRunFile(String filePath, double maxRho);
  bool pause();
  bool resume();
  bool stop();

  // Main processing loop - call from motor task
  bool processNextMove();

  // Getters/Setters
  State_t getState();
  void setSpeed(uint8_t speed);
  uint8_t getSpeed() const { return m_speed; }
  PolarCord_t getCurrentPosition() const { return m_currPos; }
  double getMaxRho() const { return R_MAX; }

private:
  // Hardware constants
  static constexpr double R_MAX = 450.0;
  static constexpr double T_MAX_VELOCITY = (2 * PI) / 10;      // rad/s
  static constexpr double R_MAX_VELOCITY = R_MAX / 40;          // mm/s
  static constexpr double T_MAX_ACCEL = T_MAX_VELOCITY / 0.1;   // rad/s²
  static constexpr double R_MAX_ACCEL = R_MAX_VELOCITY / 0.1;   // mm/s²
  static constexpr int STEPS_PER_RADIAN = (400 / (2 * PI)) * (60 / 16);
  static constexpr int STEPS_PER_MM = 100;

  static constexpr int T_MICROSTEPS = 2;
  static constexpr int T_CURRENT = 800;
  static constexpr double T_MAX_STEP_VELOCITY = STEPS_PER_RADIAN * T_MAX_VELOCITY;
  static constexpr double T_MAX_STEP_ACCEL = STEPS_PER_RADIAN * T_MAX_ACCEL;

  static constexpr int R_MICROSTEPS = 2;
  static constexpr int R_CURRENT = 500;
  static constexpr double R_MAX_STEP_VELOCITY = STEPS_PER_MM * R_MAX_VELOCITY;
  static constexpr double R_MAX_STEP_ACCEL = STEPS_PER_RADIAN * R_MAX_ACCEL;

  // Look-ahead buffer
  struct PathBuffer {
    static constexpr int BUFFER_SIZE = 10;
    PolarCord_t points[BUFFER_SIZE];
    int head = 0;
    int tail = 0;
    int count = 0;

    bool isEmpty() const { return count == 0; }
    bool isFull() const { return count == BUFFER_SIZE; }
    void push(PolarCord_t pos);
    PolarCord_t peek(int offset) const;
    PolarCord_t pop();
    void clear();
  };

  // Curvature analysis
  struct PathCurvature {
    double radius;
    double angle;
    double maxSafeSpeed;
  };

  // Hardware
  FastAccelStepperEngine m_stepperEngine;
  FastAccelStepper *m_tStepper;
  FastAccelStepper *m_rStepper;
  TMC2209Stepper m_tDriver;
  TMC2209Stepper m_rDriver;
  TMC2209Stepper m_rCDriver;

  // State
  State_t m_state;
  PosGen *m_posGen = nullptr;
  PathBuffer m_buffer;
  PolarCord_t m_currPos = {0.0, 0.0};
  double m_currentVelR = 0.0;
  double m_currentVelT = 0.0;
  int8_t m_lastTDir = 1;
  int8_t m_lastRDir = 1;
  uint8_t m_speed = 5;

  // Motion planning parameters
  bool m_useScurve = true;
  double m_maxJerkT = 60.0;
  double m_maxJerkR = 1000.0;
  double m_maxCentripetalAccel = 50.0;
  double m_minCurveRadius = 5.0;
  double m_centerThreshold = 10.0;

  // Buffer management
  void fillBuffer();

  // Motion planning
  bool planAndQueueNextMove();
  bool handleDirectionChange(PolarCord_t target, bool rChange, bool tChange);
  PolarCord_t calculateSmoothedTarget();
  double calculateCartesianDistance(PolarCord_t from, PolarCord_t to);

  // Move execution
  bool queueCoordinatedMove(PolarCord_t target, double maxVelR, double maxVelT,
                            double endVelR, double endVelT);
  void forceStop();

  // Profile calculation
  bool calculateMotionProfile(double distance, double startVel, double endVel,
                              double maxVel, double maxAccel, MotionProfile &profile);
  bool synchronizeProfiles(MotionProfile &p1, MotionProfile &p2, double targetTime = 0.0);
  bool adjustProfileToTime(MotionProfile &profile, double targetTime);

  // Speed adaptation
  PathCurvature calculateCurvature(int bufferIndex);
  double adaptSpeedForCurvature(double baseSpeed, const PathCurvature &curve, bool isRadial);
  double calculateDynamicTimeResolution(double distance);

  // Path smoothing
  PolarCord_t interpolateSpline(double t, PolarCord_t p0, PolarCord_t p1,
                                 PolarCord_t p2, PolarCord_t p3);

  // Driver setup
  static void commonSetupDriver(TMC2209Stepper &driver);
  static void home(TMC2209Stepper &driver, int speed);
  static void home(TMC2209Stepper &driver);
};
