#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

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
 * The trajectory loaded by the TRAJECTORY command,
 * containing one state vector per second of mission time.
 */
static struct state_vector trajectory[TRAJECTORY_LEN];

/**
 * Counter representing the mission time, in seconds.
 */
static int mission_clock_sec;
/**
 * The current state vector based on the last sensor read.
 */
static struct state_vector sensed_state;

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
 * "Raises" an error, which prints the given error message
 * to the serial port and turns on the error LED.
 *
 * @param error the message to print to the serial port
 */
static void raise_error(const String &error) {
    Serial.print("ERROR: '");
    Serial.print(error);
    Serial.println("'");

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
            last_buf = "";

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

    Serial.print("configTICK_RATE_HZ = ");
    Serial.println(configTICK_RATE_HZ);

    Serial.print("TRAJECTORY_LEN = ");
    Serial.println(TRAJECTORY_LEN);

    pinMode(HEALTHY_LED_PIN, OUTPUT);
    pinMode(ERROR_LED_PIN, OUTPUT);
}

/**
 * Handles a command to add the given trajectory arguments
 * to memory.
 *
 * @param args the trajectory command arguments to process
 */
static void handle_cmd_trajectory(const String &args) {
    int idx_end_idx = args.indexOf(" ");
    int x_end_idx = args.indexOf(" ", idx_end_idx);
    int y_end_idx = args.indexOf(" ", x_end_idx);
    int z_end_idx = args.indexOf(" ", y_end_idx);
    int vx_end_idx = args.indexOf(" ", z_end_idx);
    int vy_end_idx = args.indexOf(" ", vx_end_idx);

    int idx = args.substring(0, idx_end_idx).toInt();
    state_vector &v = trajectory[idx];

    v.x = args.substring(idx_end_idx + 1, x_end_idx).toDouble();
    v.y = args.substring(x_end_idx + 1, y_end_idx).toDouble();
    v.z = args.substring(y_end_idx + 1, z_end_idx).toDouble();
    v.vx = args.substring(z_end_idx + 1, vx_end_idx).toDouble();
    v.vy = args.substring(vx_end_idx + 1, vy_end_idx).toDouble();
    v.vz = args.substring(vy_end_idx + 1).toDouble();
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
 * The task for performing guidance.
 *
 * @param args unused
 */
[[noreturn]] void guidance_ctl(void *args) {
    TickType_t prev_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&prev_time, sec_to_ticks(0.1));

        state_vector &goal = trajectory[mission_clock_sec];
        double goal_pos_vec = magnitude(goal.x, goal.y, goal.z);
        double cur_pos_vec = magnitude(sensed_state.x, sensed_state.y, sensed_state.z);
        if (abs(goal_pos_vec - cur_pos_vec) > TUNNEL_RAD_M) {
            raise_error("Tunnel bounds exceeded");
        }

        double goal_vel_vec = magnitude(goal.vx, goal.vy, goal.vz);
        double cur_vel_vec = magnitude(sensed_state.vx, sensed_state.vy, sensed_state.vz);
        if (abs(goal_vel_vec - cur_vel_vec) > MAX_V_ERROR) {
            raise_error("Velocity error exceeded");
        }
    }
}

/**
 * Called at T-0 seconds to start any flight-related tasks.
 */
static void liftoff() {
    xTaskCreate(guidance_ctl, "guidance_ctl", 128, nullptr, 2, nullptr);
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

        if (mission_clock_sec == 0) {
            liftoff();
        }

        if (mission_clock_sec > TRAJECTORY_LEN) {
            end_mission();
        }
    }
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
    int x_end_idx = args.indexOf(" ");
    int y_end_idx = args.indexOf(" ", x_end_idx);
    int z_end_idx = args.indexOf(" ", y_end_idx);
    int vx_end_idx = args.indexOf(" ", z_end_idx);
    int vy_end_idx = args.indexOf(" ", vx_end_idx);

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
        Serial.print("ERROR: Unrecognized command: '");
        Serial.print(cmd);
        Serial.println("'");
        return;
    }

    const String cmd_type = cmd.substring(0, type_end_idx);
    if (cmd_type.equals("TRAJECTORY")) {
        const String &args = cmd.substring(type_end_idx + 1);
        handle_cmd_trajectory(args);
    } else if (cmd_type.equals("BEGIN")) {
        handle_cmd_begin();
    } else {
        Serial.print("ERROR: Unrecognized command: '");
        Serial.print(cmd);
        Serial.println("'");
    }
}

/**
 * Sends the telemetry information to the simulated
 * downlink on the serial port.
 */
static void send_telem() {
    Serial.print("TELEMETRY ");
    Serial.print("mission_clock_sec = ");
    Serial.println(mission_clock_sec);
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

    xTaskCreate(mission_clock, "mission_clock", 67, nullptr, 3, nullptr);
    xTaskCreate(low_priority_tasks, "low_priority_tasks", 128, nullptr, 1, nullptr);
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
    raise_error("Heap exceeded");
}

/**
 * The custom stack overflow failure hook, which raises an
 * error instead.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    raise_error("Stack overflowed");
}
