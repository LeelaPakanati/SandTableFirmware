#pragma once
#include <TMCStepper.h>
#include <MultiStepperLite.h>
#include <PosGen.hpp>
#include <freertos/semphr.h>

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
    PAUSED,
    STOPPING  // Waiting for queued moves to complete
  };

  PolarControl();
  ~PolarControl();

  // Lifecycle
  void begin();
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
  PolarCord_t getCurrentPosition() const { return m_currPos; }  // Queued/planned position
  PolarCord_t getActualPosition() const;  // Actual stepper position
  double getMaxRho() const { return R_MAX; }

private:
  // Physical constants
  static constexpr double R_MAX = 450.0;
  static constexpr int T_MICROSTEPS = 2;
  static constexpr int R_MICROSTEPS = 2;

  static constexpr int STEPS_PER_MM = 50 * R_MICROSTEPS;
  static constexpr int STEPS_PER_RADIAN = (int)((200.0 * T_MICROSTEPS / (2.0 * PI)) * (60.0 / 16.0));

  // Motor limits (these clamp the maximum motor speeds)
  // R: 450mm in 20s = 22.5 mm/s max
  // T: 2*PI rad in 20s = 0.314 rad/s max
  static constexpr double R_MAX_VELOCITY = 22.5;                // mm/s (20s edge-to-edge)
  static constexpr double T_MAX_VELOCITY = 0.314;               // rad/s (20s per rotation)
  static constexpr double R_MAX_ACCEL = 50.0;                   // mm/s² (gentler accel)
  static constexpr double T_MAX_ACCEL = 1.0;                    // rad/s² (gentler accel)

  // Driver settings
  static constexpr int T_CURRENT = 800;
  static constexpr int R_CURRENT = 500;

  // Look-ahead buffer
  static constexpr int BUFFER_SIZE = 10;
  struct PathBuffer {
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

  // Hardware
  MultiStepperLite m_stepper; // Index 0: R, 1: T
  TMC2209Stepper m_tDriver;
  TMC2209Stepper m_rDriver;
  TMC2209Stepper m_rCDriver;

  int32_t m_rPos = 0;
  int32_t m_tPos = 0;
  int32_t m_rTargetPos = 0;
  int32_t m_tTargetPos = 0;
  int8_t m_rDir = 1;
  int8_t m_tDir = 1;

  SemaphoreHandle_t m_mutex = NULL;

  State_t m_state = UNINITIALIZED;
  PosGen *m_posGen = nullptr;
  PathBuffer m_buffer;

  // Current position and velocity
  PolarCord_t m_currPos = {0.0, 0.0};
  double m_currVelR = 0.0;    // Current R velocity (mm/s)
  double m_currVelT = 0.0;    // Current T velocity (rad/s)

  // Interpolation state
  static constexpr double MAX_SEGMENT_LEN = 2.0; // mm (max length of a single move command)
  PolarCord_t m_pendingTarget = {NAN, NAN};      // The full target point we are interpolating towards
  bool m_hasPendingTarget = false;

  // Speed setting: speed 10 = 20mm/s Cartesian velocity
  uint8_t m_speed = 5;

  // Buffer management
  void fillBuffer();

  // Motion planning - the core of the new system
  bool planNextMove();
  double calcLookAheadEndVel(int axis);  // 0=R, 1=T

  // Execute a move with trapezoidal velocity profile
  bool executeMove(double rDist, double tDist,
                   double startVelR, double startVelT,
                   double endVelR, double endVelT,
                   double maxVelR, double maxVelT);

  // Helpers
  static double normalizeThetaDiff(double diff);
  double calcCartesianDist(PolarCord_t from, PolarCord_t to);
  void forceStop();

  // Driver setup and homing
  static void commonSetupDriver(TMC2209Stepper &driver);
  void homeDriver(TMC2209Stepper &driver, int speed);
  void homeDriver(TMC2209Stepper &driver);
};
