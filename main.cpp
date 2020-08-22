#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <EEPROMex.h>
#include "pidf_controller.h"

// Numerical values #defined here so as to avoid taking up
// SRAM space

/**
 * Mission clock task stack size.
 */
#define MISSION_CLOCK_STACK 70
/**
 * Guidance control task stack size.
 */
#define GUIDANCE_CTL_STACK 300
/**
 * Low priority tasks task stack size.
 */
#define LOW_PRIORITY_TASKS_STACK 160

/**
 * The initial countdown (T minus AUTO_MISSION_TIME_SEC)
 * after the BEGIN command is sent in which the system is
 * in "autonomous" mode.
 */
#define AUTO_MISSION_TIME_SEC -10

/**
 * The trajectory tunnel radius outside which would trigger
 * the FTS, in meters.
 */
#define TUNNEL_RAD_M 500
/**
 * The maximum velocity error above which the FTS would be
 * triggered, in m/s.
 */
#define MAX_V_ERROR 10

/**
 * The pin used for the LED indicating the system health.
 */
#define HEALTHY_LED_PIN 2
/**
 * The pin used for the LED indicating an error.
 */
#define ERROR_LED_PIN 3

/**
 * Constant name helper to reduce RAM usage for the name of
 * the mission_clock task.
 */
static const char mission_clock_name[] PROGMEM = "mission_clock";
/**
 * Constant name helper to reduce RAM usage for the name of
 * the guidance_ctl task.
 */
static const char guidance_ctl_name[] PROGMEM = "guidance_ctl";
/**
 * Constant name helper to reduce RAM usage for the name of
 * the low_priority_tasks task.
 */
static const char low_priority_tasks_name[] PROGMEM = "low_priority_tasks";

/**
 * Whether or not the mission has begun and that the
 * mission clock should start counting and is valid.
 */
static bool mission_began = false;

/**
 * Whether the healthy LED is currently on.
 */
static bool healthy_led_on = false;

/**
 * The incomplete line read from the last call to the
 * read_line method which will be prepended to the next
 * call to avoid missing any incomplete lines.
 */
static String last_buf;

/**
 * A struct representing the position and velocity vector.
 */
struct state_vector {
    /**
     * Position X, in meters.
     */
    double x;
    /**
     * Position Y, in meters.
     */
    double y;
    /**
     * Position Z, in meters.
     */
    double z;
    /**
     * Velocity X, in m/s.
     */
    double vx;
    /**
     * Velocity Y, in m/s.
     */
    double vy;
    /**
     * Velocity Z, in m/s.
     */
    double vz;
};

/**
 * The number of elements in the trajectory array.
 */
static const int TRAJECTORY_LEN = 10;

/**
 * Counter representing the mission time, in seconds.
 */
static int mission_clock_sec;
/**
 * The cached value of the target state vector for the
 * current mission clock time.
 */
static struct state_vector target_state;
/**
 * The current state vector based on the last sensor read.
 */
static struct state_vector sensed_state;

/**
 * Simulated throttle for the X velocity.
 */
static double vx_throttle;
/**
 * Simulated throttle for the Y velocity.
 */
static double vy_throttle;
/**
 * Simulated throttle for the Z velocity.
 */
static double vz_throttle;

/**
 * Converts the given number of seconds into ticks. These
 * are not represented as static constants so as to allow
 * the trajectory to be stored to be stored in the program
 * memory instead.
 *
 * @param seconds the number of seconds
 * @return the number of equivalent ticks at the configured
 * tick frequency
 */
static TickType_t sec_to_ticks(double seconds) {
    return seconds * configTICK_RATE_HZ;
}

/**
 * Computes the address at which to store and retrieve the
 * target trajectory at the given time index.
 * 
 * @param index the time index
 * @return the EEPROM address
 */
static int get_trajectory_addr(int index) {
    return index * sizeof(struct state_vector);
}

/**
 * "Raises" an error, which prints the given error message
 * to the serial port and turns on the error LED.
 *
 * @param error the message to print to the serial port
 */
static void raise_error(const String &error) {
    Serial.print(F("ERROR: '"));
    Serial.print(error);
    Serial.println(F("'"));

    digitalWrite(ERROR_LED_PIN, HIGH);
    vTaskEndScheduler();
}

/**
 * Attempts to read an LF-terminated line from the serial
 * port.
 *
 * @param result the full String, if true is returned
 * @return true only if a full string is returned, false if
 * the line is incomplete or if there is no data
 */
static bool read_line(String *result) {
    while (Serial.available()) {
        int in = Serial.read();
        if (in == '\n') {
            *result = last_buf;
            last_buf = F("");

            return true;
        }

        last_buf += static_cast<char>(in);
    }

    return false;
}

/**
 * Performs non-task related setup such as initializing
 * pins and the serial port.
 */
static void setup0() {
    Serial.begin(9600);

    Serial.print(F("configTICK_RATE_HZ = "));
    Serial.println(configTICK_RATE_HZ);

    Serial.print(F("TRAJECTORY_LEN = "));
    Serial.println(TRAJECTORY_LEN);

    pinMode(HEALTHY_LED_PIN, OUTPUT);
    pinMode(ERROR_LED_PIN, OUTPUT);
}

/**
 * Determines the magnitude of the given 3D vector.
 * 
 * @param a the first component
 * @param b the second component
 * @param c the third component
 * @return the magnitude of the vector of the given 3
 * components
 */
static double magnitude(double a, double b, double c) {
    return sqrt(a * a + b * b + c * c);
}

/**
 * Simulated "flight termination system" (FTS) subroutine
 * to check whether the guidance parameters are being met.
 */
static void fts_check() {
    double goal_pos_vec = magnitude(target_state.x, target_state.y, target_state.z);
    double cur_pos_vec = magnitude(sensed_state.x, sensed_state.y, sensed_state.z);
    if (abs(goal_pos_vec - cur_pos_vec) > TUNNEL_RAD_M) {
        raise_error(F("Tunnel bounds exceeded"));
    }

    double goal_vel_vec = magnitude(target_state.vx, target_state.vy, target_state.vz);
    double cur_vel_vec = magnitude(sensed_state.vx, sensed_state.vy, sensed_state.vz);
    if (abs(goal_vel_vec - cur_vel_vec) > MAX_V_ERROR) {
        raise_error(F("Velocity error exceeded"));
    }
}

/**
 * The task for performing guidance.
 *
 * @param args unused
 */
[[noreturn]] void guidance_ctl(void *args) {
    pidf_controller x_pidf{1.0, 0.1, 0.0, 0.0, 0.0};
    pidf_controller y_pidf{1.0, 0.1, 0.0, 0.0, 0.0};
    pidf_controller z_pidf{1.0, 0.1, 0.0, 0.0, 0.0};
    pidf_controller vx_pidf{1.0, 0.1, 0.0, 0.0, 0.0};
    pidf_controller vy_pidf{1.0, 0.1, 0.0, 0.0, 0.0};
    pidf_controller vz_pidf{1.0, 0.1, 0.0, 0.0, 0.0};

    TickType_t prev_time = xTaskGetTickCount();

    TickType_t last_guidance_time = prev_time;

    while (true) {
        vTaskDelayUntil(&prev_time, sec_to_ticks(0.1));

        fts_check();

        TickType_t cur_guidance_time = xTaskGetTickCount();
        if (cur_guidance_time - last_guidance_time > sec_to_ticks(1.0)) {
            // Update setpoint for first step
            x_pidf.set_setpoint(target_state.x);
            y_pidf.set_setpoint(target_state.y);
            z_pidf.set_setpoint(target_state.z);

            // Update last state for first step
            x_pidf.set_last_state(sensed_state.x);
            y_pidf.set_last_state(sensed_state.y);
            z_pidf.set_last_state(sensed_state.z);

            // Now update the cascade with the new values
            vx_pidf.set_setpoint(x_pidf.compute_pidf());
            vy_pidf.set_setpoint(y_pidf.compute_pidf());
            vz_pidf.set_setpoint(z_pidf.compute_pidf());

            // Update the velocity current state
            vx_pidf.set_last_state(sensed_state.vx);
            vy_pidf.set_last_state(sensed_state.vy);
            vz_pidf.set_last_state(sensed_state.vz);

            // Obtain the final output values for telemetry
            vx_throttle = vx_pidf.compute_pidf();
            vy_throttle = vy_pidf.compute_pidf();
            vz_throttle = vz_pidf.compute_pidf();
        }
    }
}

/**
 * Called at T-0 seconds to start any flight-related tasks.
 */
static void liftoff() {
    xTaskCreate(guidance_ctl, guidance_ctl_name, GUIDANCE_CTL_STACK, nullptr, 2, nullptr);
}

/**
 * Called when the timer indicates that the mission is
 * over.
 */
static void end_mission() {
    digitalWrite(HEALTHY_LED_PIN, HIGH);
    vTaskEndScheduler();
}

/**
 * The task for running the mission clock.
 *
 * @param args unused
 */
[[noreturn]] void mission_clock(void *args) {
    TickType_t prev_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&prev_time, sec_to_ticks(1.0));
        if (!mission_began) {
            continue;
        }

        mission_clock_sec++;
        
        if (mission_clock_sec > 0) {
            int addr = get_trajectory_addr(mission_clock_sec);
            EEPROM.readBlock(addr, &target_state);
        }

        if (mission_clock_sec == 0) {
            liftoff();
        }

        if (mission_clock_sec > TRAJECTORY_LEN) {
            end_mission();
        }
    }
}

/**
 * Handles a command to add the given trajectory arguments
 * to memory.
 *
 * @param args the trajectory command arguments to process
 */
static void handle_cmd_trajectory(const String &args) {
    int idx_end_idx = args.indexOf(F(" "));
    int x_end_idx = args.indexOf(F(" "), idx_end_idx);
    int y_end_idx = args.indexOf(F(" "), x_end_idx);
    int z_end_idx = args.indexOf(F(" "), y_end_idx);
    int vx_end_idx = args.indexOf(F(" "), z_end_idx);
    int vy_end_idx = args.indexOf(F(" "), vx_end_idx);

    struct state_vector v;
    v.x = args.substring(idx_end_idx + 1, x_end_idx).toDouble();
    v.y = args.substring(x_end_idx + 1, y_end_idx).toDouble();
    v.z = args.substring(y_end_idx + 1, z_end_idx).toDouble();
    v.vx = args.substring(z_end_idx + 1, vx_end_idx).toDouble();
    v.vy = args.substring(vx_end_idx + 1, vy_end_idx).toDouble();
    v.vz = args.substring(vy_end_idx + 1).toDouble();

    int idx = args.substring(0, idx_end_idx).toInt();
    int addr = get_trajectory_addr(idx);
    EEPROM.writeBlock(addr, v);
}

/**
 * Handles the BEGIN command to start the launch sequence.
 */
static void handle_cmd_begin() {
    mission_began = true;
    mission_clock_sec = AUTO_MISSION_TIME_SEC;
}

/**
 * Handles a command that simulates sensor data being
 * received.
 *
 * @param args the trajectory command arguments to process
 */
static void handle_cmd_sensor(const String &args) {
    int x_end_idx = args.indexOf(F(" "));
    int y_end_idx = args.indexOf(F(" "), x_end_idx);
    int z_end_idx = args.indexOf(F(" "), y_end_idx);
    int vx_end_idx = args.indexOf(F(" "), z_end_idx);
    int vy_end_idx = args.indexOf(F(" "), vx_end_idx);

    sensed_state.x = args.substring(0, x_end_idx).toDouble();
    sensed_state.y = args.substring(x_end_idx + 1, y_end_idx).toDouble();
    sensed_state.z = args.substring(y_end_idx + 1, z_end_idx).toDouble();
    sensed_state.vx = args.substring(z_end_idx + 1, vx_end_idx).toDouble();
    sensed_state.vy = args.substring(vx_end_idx + 1, vy_end_idx).toDouble();
    sensed_state.vz = args.substring(vy_end_idx + 1).toDouble();
}

/**
 * Command processor function.
 *
 * @param cmd the command to process read from the serial
 * port
 */
static void handle_cmd(const String &cmd) {
    int type_end_idx = cmd.indexOf(' ');
    if (type_end_idx < 0) {
        Serial.print(F("ERROR: Unrecognized command: '"));
        Serial.print(cmd);
        Serial.println(F("'"));
        return;
    }

    const String cmd_type = cmd.substring(0, type_end_idx);
    if (cmd_type.equals(F("TRAJECTORY"))) {
        const String &args = cmd.substring(type_end_idx + 1);
        handle_cmd_trajectory(args);
    } else if (cmd_type.equals(F("BEGIN"))) {
        handle_cmd_begin();
    } else if (cmd_type.equals(F("SENSOR"))) {
        const String &args = cmd.substring(type_end_idx + 1);
        handle_cmd_sensor(args);
    } else {
        Serial.print(F("ERROR: Unrecognized command: '"));
        Serial.print(cmd);
        Serial.println(F("'"));
    }
}

/**
 * Sends the telemetry information to the simulated
 * downlink on the serial port.
 */
static void send_telem() {
    Serial.print(F("TELEMETRY "));
    Serial.print(mission_clock_sec);
    Serial.print(F(" "));
    Serial.print(vx_throttle);
    Serial.print(F(" "));
    Serial.print(vy_throttle);
    Serial.print(F(" "));
    Serial.println(vz_throttle);
}

/**
 * Causes the healthy LED to blink and toggles its stored
 * state accordingly.
 */
static void blink_led() {
    healthy_led_on = !healthy_led_on;
    digitalWrite(HEALTHY_LED_PIN, healthy_led_on ? HIGH : LOW);
}

/**
 * Tasks grouped together as the "low-priority" relative to
 * the other tasks to reduce RAM usage. This includes the
 * command processing, telemetry downlink and status LED
 * blink tasks.
 *
 * @param args unused
 */
[[noreturn]] void low_priority_tasks(void *args) {
    TickType_t last_cycle_time = xTaskGetTickCount();

    TickType_t last_telem_time = last_cycle_time;
    TickType_t last_led_time = last_cycle_time;

    while (true) {
        vTaskDelayUntil(&last_cycle_time, sec_to_ticks(0.1));

        String command;
        while (read_line(&command)) {
            handle_cmd(command);
        }

        TickType_t cur_telem_time = xTaskGetTickCount();
        if (cur_telem_time - last_telem_time >= sec_to_ticks(1.0)) {
            send_telem();
            last_telem_time = cur_telem_time;
        }

        TickType_t cur_led_time = xTaskGetTickCount();
        if (cur_led_time - last_led_time >= sec_to_ticks(0.5)) {
            blink_led();
            last_led_time = cur_led_time;
        }
    }
}

/**
 * The Arduino setup task, running the miscellaneous setup
 * in addition to the task setup.
 */
void setup() {
    setup0();

    xTaskCreate(mission_clock, mission_clock_name, MISSION_CLOCK_STACK, nullptr, 3, nullptr);
    xTaskCreate(low_priority_tasks, low_priority_tasks_name, LOW_PRIORITY_TASKS_STACK, nullptr, 1, nullptr);
}

/**
 * Unused.
 */
void loop() {
}

/**
 * The custom heap allocation failure hook, which raises an
 * error instead.
 */
void vApplicationMallocFailedHook() {
    raise_error(F("Heap exceeded"));
}

/**
 * The custom stack overflow failure hook, which raises an
 * error instead.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    raise_error(F("Stack overflowed"));
}
