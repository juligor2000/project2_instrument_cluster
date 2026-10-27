#include <stdio.h>
#include <stdint.h>
#include "tpl_os.h"

/* =====================================
   OSEK DECLARATIONS
   ===================================== */

DeclareResource(ICM_Resource);
DeclareEvent(Event_CAN_M1);
DeclareEvent(Event_CAN_M2);

/* =====================================
   CONFIGURATION
   ===================================== */

/* Nominal OS tick = 10 ms */
#define CAN_M1_TIMEOUT_TICKS 150U
#define CAN_M2_TIMEOUT_TICKS 300U

/* Simulate CAN M1 loss after 5 seconds */
#define ENABLE_CAN_M1_FAULT_TEST 1
#define CAN_M1_FAULT_START_TICK 500U

/* =====================================
   SHARED VEHICLE DATA
   ===================================== */

typedef struct {
    uint16_t speed_kmh;
    uint16_t rpm;
    uint8_t fuel_percent;
    uint8_t coolant_temp;
    uint8_t engine_warning;

    uint8_t can_m1_timeout;
    uint8_t can_m2_timeout;
} ICM_Data;

static ICM_Data vehicle = {
    .speed_kmh = 0,
    .rpm = 800,
    .fuel_percent = 80,
    .coolant_temp = 85,
    .engine_warning = 0,
    .can_m1_timeout = 0,
    .can_m2_timeout = 0
};

/* Shared diagnostic timestamps */
static uint32_t system_ticks = 0;
static uint32_t last_can_m1_tick = 0;
static uint32_t last_can_m2_tick = 0;

/* Logging file */
static FILE *log_file = NULL;

/* =====================================
   MAIN
   ===================================== */

int main(void)
{
    log_file = fopen("instrument_cluster_log.csv", "w");

    if (log_file == NULL) {
        printf("[LOG] ERROR: Cannot create CSV file\r\n");
        return 1;
    }

    fprintf(
        log_file,
        "tick,speed_kmh,rpm,fuel_percent,"
        "coolant_temp,engine_warning,"
        "can_m1_timeout,can_m2_timeout\n"
    );

    fflush(log_file);

    printf("[LOG] CSV file created successfully\r\n");

    StartOS(OSDEFAULTAPPMODE);

    /* Normally ShutdownOS does not return */
    if (log_file != NULL) {
        fclose(log_file);
        log_file = NULL;
    }

    return 0;
}

/* =====================================
   TASK CAN RX - EVENT DRIVEN
   ===================================== */

TASK(Task_CAN_RX)
{
    EventMaskType events;

    static uint16_t simulated_rpm = 800;
    static uint16_t simulated_speed = 0;

    while (1) {

        WaitEvent(Event_CAN_M1 | Event_CAN_M2);

        GetEvent(Task_CAN_RX, &events);

        ClearEvent(events & (Event_CAN_M1 | Event_CAN_M2));

        if (events & Event_CAN_M1) {

            uint32_t now;

            GetResource(ICM_Resource);
            now = system_ticks;
            ReleaseResource(ICM_Resource);

            /*
             * Simulate lost CAN M1 frames after
             * tick 500, without discarding M2.
             */
#if ENABLE_CAN_M1_FAULT_TEST
            if (now < CAN_M1_FAULT_START_TICK)
#endif
            {
                if (simulated_rpm < 8000) {
                    simulated_rpm += 200;

                    if (simulated_rpm > 8000) {
                        simulated_rpm = 8000;
                    }
                }

                GetResource(ICM_Resource);

                vehicle.rpm = simulated_rpm;
                last_can_m1_tick = system_ticks;

                ReleaseResource(ICM_Resource);

                printf(
                    "[CAN M1] RPM received: %u\r\n",
                    (unsigned)simulated_rpm
                );
            }
        }

        if (events & Event_CAN_M2) {

            if (simulated_speed < 250) {
                simulated_speed += 10;

                if (simulated_speed > 250) {
                    simulated_speed = 250;
                }
            }

            GetResource(ICM_Resource);

            vehicle.speed_kmh = simulated_speed;
            last_can_m2_tick = system_ticks;

            ReleaseResource(ICM_Resource);

            printf(
                "[CAN M2] Speed received: %u km/h\r\n",
                (unsigned)simulated_speed
            );
        }
    }
}

/* =====================================
   TASK ACQUISITION - 10 ms
   ===================================== */

TASK(Task_Acquisition)
{
    static unsigned int acquisition_count = 0;

    acquisition_count++;

    GetResource(ICM_Resource);

    system_ticks++;

    /* Simulate coolant temperature sensor */
    if (acquisition_count % 100 == 0) {
        if (vehicle.coolant_temp < 120) {
            vehicle.coolant_temp++;
        }
    }

    ReleaseResource(ICM_Resource);

    TerminateTask();
}

/* =====================================
   TASK PROCESSING - 20 ms
   ===================================== */

TASK(Task_Processing)
{
    GetResource(ICM_Resource);

    vehicle.engine_warning =
        (vehicle.coolant_temp > 110) ? 1 : 0;

    ReleaseResource(ICM_Resource);

    TerminateTask();
}

/* =====================================
   TASK DIAGNOSTICS - 100 ms
   ===================================== */

TASK(Task_Diagnostics)
{
    uint32_t now;
    uint32_t age_m1;
    uint32_t age_m2;

    uint8_t previous_m1;
    uint8_t previous_m2;
    uint8_t current_m1;
    uint8_t current_m2;

    GetResource(ICM_Resource);

    now = system_ticks;

    age_m1 = now - last_can_m1_tick;
    age_m2 = now - last_can_m2_tick;

    previous_m1 = vehicle.can_m1_timeout;
    previous_m2 = vehicle.can_m2_timeout;

    vehicle.can_m1_timeout =
        (age_m1 > CAN_M1_TIMEOUT_TICKS) ? 1 : 0;

    vehicle.can_m2_timeout =
        (age_m2 > CAN_M2_TIMEOUT_TICKS) ? 1 : 0;

    current_m1 = vehicle.can_m1_timeout;
    current_m2 = vehicle.can_m2_timeout;

    ReleaseResource(ICM_Resource);

    /* Report diagnostic state transitions */
    if (current_m1 != previous_m1) {

        if (current_m1) {
            printf(
                "[DIAG] CAN M1 TIMEOUT! Age: %u ticks\r\n",
                (unsigned)age_m1
            );
        } else {
            printf(
                "[DIAG] CAN M1 communication restored\r\n"
            );
        }
    }

    if (current_m2 != previous_m2) {

        if (current_m2) {
            printf(
                "[DIAG] CAN M2 TIMEOUT! Age: %u ticks\r\n",
                (unsigned)age_m2
            );
        } else {
            printf(
                "[DIAG] CAN M2 communication restored\r\n"
            );
        }
    }

    TerminateTask();
}

/* =====================================
   TASK DISPLAY - 50 ms
   ===================================== */

TASK(Task_Display)
{
    static uint8_t display_counter = 0;
    ICM_Data snapshot;

    display_counter++;

    /*
     * Task executes every 50 ms,
     * but prints every 500 ms.
     */
    if (display_counter >= 10) {

        display_counter = 0;

        GetResource(ICM_Resource);
        snapshot = vehicle;
        ReleaseResource(ICM_Resource);

        printf(
            "\r\n"
            "========== INSTRUMENT CLUSTER ==========\r\n"
            "Speed       : %u km/h\r\n"
            "Engine RPM  : %u rpm\r\n"
            "Fuel        : %u %%\r\n"
            "Temperature : %u C\r\n"
            "Engine Warn : %s\r\n"
            "CAN M1      : %s\r\n"
            "CAN M2      : %s\r\n"
            "========================================\r\n",
            (unsigned)snapshot.speed_kmh,
            (unsigned)snapshot.rpm,
            (unsigned)snapshot.fuel_percent,
            (unsigned)snapshot.coolant_temp,
            snapshot.engine_warning ? "ACTIVE" : "OK",
            snapshot.can_m1_timeout ? "TIMEOUT" : "OK",
            snapshot.can_m2_timeout ? "TIMEOUT" : "OK"
        );
    }

    TerminateTask();
}

/* =====================================
   TASK LOGGING - 1000 ms
   ===================================== */

TASK(Task_Logging)
{
    ICM_Data snapshot;
    uint32_t timestamp;

    /*
     * Copy shared data while holding
     * the OSEK resource.
     */
    GetResource(ICM_Resource);

    snapshot = vehicle;
    timestamp = system_ticks;

    ReleaseResource(ICM_Resource);

    /*
     * File I/O occurs outside
     * the critical section.
     */
    if (log_file != NULL) {

        fprintf(
            log_file,
            "%u,%u,%u,%u,%u,%u,%u,%u\n",
            (unsigned)timestamp,
            (unsigned)snapshot.speed_kmh,
            (unsigned)snapshot.rpm,
            (unsigned)snapshot.fuel_percent,
            (unsigned)snapshot.coolant_temp,
            (unsigned)snapshot.engine_warning,
            (unsigned)snapshot.can_m1_timeout,
            (unsigned)snapshot.can_m2_timeout
        );

        fflush(log_file);

        printf(
            "[LOG] Vehicle data saved at tick %u\r\n",
            (unsigned)timestamp
        );
    }

    TerminateTask();
}

/* =====================================
   TASK STOP
   ===================================== */

TASK(Task_Stop)
{
    printf("\r\nInstrument Cluster: Shutdown\r\n");

    /*
     * Close CSV before shutting down
     * the virtual RTOS.
     */
    if (log_file != NULL) {
        fflush(log_file);
        fclose(log_file);
        log_file = NULL;
    }

    printf("[LOG] CSV file closed\r\n");

    ShutdownOS(E_OK);

    TerminateTask();
}