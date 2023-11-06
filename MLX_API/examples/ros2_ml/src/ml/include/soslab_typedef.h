/*
 * soslab_typdef.h
 *
 *  Created on: Nov 16, 2018
 *      Author: gnohead
 */

#ifndef SOSLAB_TYPEDEF_H_
#define SOSLAB_TYPEDEF_H_

#include <cstdint>
#include <string>
#include <vector>
#include <utility>
#include <cmath>
#include <cstring>
#include <system_error>
#include <limits>

#define _USE_MATH_DEFINES
#include <math.h>

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
    /** @brief integer */
    typedef int32_t INT_T;

    /** @brief unsigned int */
    typedef uint32_t UINT_T;

    /** @brief floating point */
    typedef double FLOAT_T;

    /** @brief data arryay */
    typedef std::vector<uint8_t> hex_array_t;

    /** @brief The structure of a single point for the cartesian coordinate. */
    typedef struct _POINT_T
    {
        float x;          /**< x value (unit : mm) */
        float y;          /**< y value (unit : mm) */
        float z;          /**< z value (unit : mm) */
    } point_t;

    /** @brief Structure of the pointcloud of SOSLAB LiDAR */
    typedef std::vector<point_t> pointcloud_t;

    /** @brief IP communication address */
    typedef struct IP_ADDRESS_T {
        std::string ip_address;
        int port_number;
    } ip_settings_t;
} /* namespace SOSLAB */


#endif /* SOSLAB_TYPEDEF_H_ */



