#ifndef MECHANICAL_DIAGNOSTICS_H
#define MECHANICAL_DIAGNOSTICS_H
#include "actuator_control.h"
#include "servo_bus.h"
/* Writer receives <=95 ASCII characters. Called only outside motion loops. */
typedef bool (*MechanicalLogWriter)(void *context, const char *text);
bool mechanical_log_capture(ServoBus *bus, bool moving, const char *label,
                            MechanicalLogWriter writer, void *context);
bool mechanical_log_gait(const ActuatorDiagnostics *diagnostics, bool moving,
                         MechanicalLogWriter writer, void *context);
#endif
