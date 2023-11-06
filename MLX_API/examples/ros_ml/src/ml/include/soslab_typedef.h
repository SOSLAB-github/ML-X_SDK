/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#ifndef SOSLAB_TYPEDEF_H_
#define SOSLAB_TYPEDEF_H_

#include <iostream>
#include <sstream>
#include <vector>
#include <deque>
#include <condition_variable>
#include <fstream>
#include <map> //C++03
#include <iomanip>
#include <functional>
#include <thread>
#include <cstring>

/* Macros */
#ifdef SOSLABAPI_EXPORTS
	#if defined(_MSC_VER)
		#define SOSLAB_EXPORTS __declspec(dllexport)
	#elif defined(__GNUC__)
		#define SOSLABAPI_EXPORTS __attribute__((visibility("default")))
	#endif
#endif

#ifndef SOSLAB_EXPORTS
	#define SOSLAB_EXPORTS
#endif


/* un-used */
#define UNUSED(x) (void)(x)

/** @brief default value for 1 Giga */
#define CONST_ONE_GIGA  1000000000.0
/** @brief default value for 1 nano */
#define CONST_ONE_NANO  (1.0 / CONST_ONE_GIGA)
/** @brief default value for 20 nano */
#define CONST_20_NANO   (20.0 * CONST_ONE_NANO)
/** @brief default value for DEG2RAD */
#define DEG2RAD         (M_PI / 180.0)
/** @brief default value for RAD2DEG */
#define RAD2DEG         (180.0 / M_PI)

/* common type definitions */
namespace SOSLAB
{
    /** @brief The structure of a single point for the cartesian coordinate. */
    typedef struct _POINT_T
    {
        float x;          /**< x value (unit : mm) */
        float y;          /**< y value (unit : mm) */
        float z;          /**< z value (unit : mm) */
    } point_t;

    /** @brief IP communication address */
    typedef struct IP_ADDRESS_T {
        std::string ip_address;
        int port_number;
    } ip_settings_t;

	typedef struct _PCD_ {
		int64_t x : 21;
		int64_t y : 21;
		int64_t z : 21;
		int64_t rsvd : 1;
	}point_cloud_t;

	#pragma pack(push, 1)
		typedef struct _ML_LIDAR_PACKET_ {
			char header[8];
			uint64_t timestamp;
			uint64_t status;
			union {
				struct {
					uint8_t multi_echo : 1;
					uint8_t rsvd : 3;
					uint8_t depth_completion : 1;
					uint8_t ambient_disable : 1;
					uint8_t depth_disable : 1;
					uint8_t intensity_disable : 1;
				}type;
				uint8_t packet_type;
			};
			uint8_t frame_id;
			uint8_t row_number;
			uint8_t rsvd[5];
		} ml_lidar_packet_t;
	#pragma pack(pop)

} /* namespace SOSLAB */


#endif /* SOSLAB_TYPEDEF_H_ */



