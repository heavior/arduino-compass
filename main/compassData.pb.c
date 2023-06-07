/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.4.8-dev */

#include "compassData.pb.h"
#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

PB_BIND(compass_Coordinate, compass_Coordinate, AUTO)


PB_BIND(compass_MapPoint, compass_MapPoint, 2)


PB_BIND(compass_CompassState, compass_CompassState, 2)


PB_BIND(compass_CompassConfig, compass_CompassConfig, AUTO)


PB_BIND(compass_CalibrationData, compass_CalibrationData, AUTO)



#ifndef PB_CONVERT_DOUBLE_FLOAT
/* On some platforms (such as AVR), double is really float.
 * To be able to encode/decode double on these platforms, you need.
 * to define PB_CONVERT_DOUBLE_FLOAT in pb.h or compiler command line.
 */
PB_STATIC_ASSERT(sizeof(double) == 8, DOUBLE_MUST_BE_8_BYTES)
#endif

