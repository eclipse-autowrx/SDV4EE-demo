/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef DEBUG_CFG_H
#define DEBUG_CFG_H

// Define für Debug-Level
#define DEBUG_LEVEL_INFO 3
#define DEBUG_LEVEL_WARN 2
#define DEBUG_LEVEL_ERR  1
#define DEBUG_LEVEL_NONE 0 // Neues Level, das alle Debug-Nachrichten deaktiviert

// Standard Debug-Level festlegen (INFO)
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL DEBUG_LEVEL_ERR
#endif


#if (DEBUG_LEVEL > DEBUG_LEVEL_INFO) || (DEBUG_LEVEL < DEBUG_LEVEL_NONE)
#error "Invalid DEBUG_LEVEL defined!"
#elif (DEBUG_LEVEL != DEBUG_LEVEL_NONE)
#include <stdio.h>
#endif


// Übergeordnete Bedingung, um Debug-Nachrichten vollständig zu deaktivieren
#if (DEBUG_LEVEL == DEBUG_LEVEL_NONE)

    // Macros für deaktivierte Debug-Nachrichten
    #define DEBUG_INFO(fmt, ...) ((void)0)
    #define DEBUG_WARNING(fmt, ...) ((void)0)
    #define DEBUG_ERROR(fmt, ...) ((void)0)
    #define DEBUG_INFO_CONDITIONAL(fmt, ...) ((void)0)
    #define DEBUG_WARNING_CONDITIONAL(fmt, ...) ((void)0)
    #define DEBUG_ERROR_CONDITIONAL(fmt, ...) ((void)0)

#else

    // Macros für Debug-Nachrichten auf verschiedenen Leveln
    #define DEBUG_INFO(fmt, ...) \
        do { \
            if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO) { \
                fprintf(stderr, "[INFO] " fmt "\n", ##__VA_ARGS__); \
            } \
        } while (0)

    #define DEBUG_WARNING(fmt, ...) \
        do { \
            if (DEBUG_LEVEL >= DEBUG_LEVEL_WARN) { \
                fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__); \
            } \
        } while (0)

    #define DEBUG_ERROR(fmt, ...) \
        do { \
            if (DEBUG_LEVEL >= DEBUG_LEVEL_ERR) { \
                fprintf(stderr, "[ERR] " fmt "\n", ##__VA_ARGS__); \
            } \
        } while (0)

    
    #define DEBUG_INFO_CONDITIONAL(condition, message, ...) \
        do { \
            if ((condition)) { \
                DEBUG_INFO(message, ##__VA_ARGS__); \
            } \
        } while (0)
    
    #define DEBUG_WARNING_CONDITIONAL(condition, message, ...) \
        do { \
            if ((condition)) { \
                DEBUG_WARNING(message, ##__VA_ARGS__); \
            } \
        } while (0)

    #define DEBUG_ERROR_CONDITIONAL(condition, message, ...) \
        do { \
            if ((condition)) { \
                DEBUG_ERROR(message, ##__VA_ARGS__); \
            } \
        } while (0)

#endif

#endif // DEBUG_CFG_H