#pragma once

// --- Standard library includes ---
#include <functional>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// --- Forward declarations, to break circular dependencies ---
class BeeBusProtocol;
class BeeBusOperations;
class BeeBusModbusDataMap;

// --- Core BeeBus library headers, in dependency order ---
//
// This is the one header an application includes. Transport implementations are deliberately absent: the application picks one
// from src/transport/ and passes it to begin(), which keeps the core free of
// any platform or medium specific dependency.
#include "BeeBusPlatform.h"       // Platform abstraction (time, target detection)
#include "BeeBusTypes.h"          // Basic types and constants
#include "BeeBusConfig.h"         // Per-instance protocol configuration
#include "BeeBusTiming.h"         // Measured rotation and hop timing
#include "BeeBusTransport.h"      // Transport layer interface
#include "BeeBusModbusDataMap.h"   // Local data storage
#include "BeeBusTopicFrame.h"     // Topic PDU codec
#include "BeeBusTopicMap.h"       // Topic storage
#include "BeeBusStreams.h"        // Reliable streams, sessions, serve / bind records
#include "BeeBusModbusFrame.h"     // Modbus PDU building and parsing
#include "BeeBusOperations.h"     // Operation queue management
#include "BeeBusModbusHandler.h"   // Modbus request processing
#include "BeeBusFrame.h"          // BeeBus frame handling
#include "BeeBusIO.h"             // IO operations
#include "BeeBusProtocol.h"       // Main protocol class
#include "BeeBusAPI.h"            // High-level API
#include "BeeBusDebug.h"          // Debugging utilities
