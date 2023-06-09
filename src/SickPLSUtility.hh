/*!
 * \file SickPLSUtility.hh
 * \brief Defines simple utility functions for working with the
 *        Sick LMS 2xx laser range finder units.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

/* Auto-generated header */
#include "SickConfig.hh"

/**
 * \def REVERSE_BYTE_ORDER_16
 * \brief Reverses the byte order of the given 16 bit unsigned integer
 */
#define REVERSE_BYTE_ORDER_16(y) ( ( ( y & 0x00FF ) << 8 ) | ( ( y & 0xFF00 ) >> 8 ) )

/**
 * \def REVERSE_BYTE_ORDER_32
 * \brief Reverses the byte order of the given 32 bit unsigned integer
 */
#define REVERSE_BYTE_ORDER_32(y) ( ( ( y & 0x000000FF ) << 24 ) | ( ( y & 0x0000FF00 ) << 8 ) | ( ( y & 0x00FF0000 ) >> 8 ) | ( ( y & 0xFF000000 ) >> 24 ) )

/* Associate the namespace */
namespace sickpls {

#ifndef WORDS_BIGENDIAN

/* NOTE: The following functions are necessary since the Sick LD doesn't adopt the
 *       convention of converting from network byte order.
 */

/**
 * \brief Converts host byte order (little-endian) to Sick LMS byte order (little-endian)
 * \param value The 2-byte value to convert to little-endian
 * \return Value in Sick LMS byte order (little-endian)
 */
    inline uint16_t host_to_sick_pls_byte_order(uint16_t value) {
        return value;
    }

/**
 * \brief Converts host byte order (little-endian) to Sick LMS byte order (little-endian)
 * \param value The 4-byte value to convert to little-endian
 * \return Value in Sick LMS byte order (little-endian)
 */
    inline uint32_t host_to_sick_pls_byte_order(uint32_t value) {
        return value;
    }

/**
 * \brief Converts Sick LMS byte order (little-endian) to host byte order (little-endian)
 * \param value The 2-byte value to convert to little-endian
 * \return Value in host byte order (little-endian)
 */
    inline uint16_t sick_pls_to_host_byte_order(uint16_t value) {
        return value;
    }

/**
 * \brief Converts Sick LMS byte order (little-endian) to host byte order (little-endian)
 * \param value The 4-byte value to convert to little-endian
 * \return Value in host byte order (little-endian)
 */
    inline uint32_t sick_pls_to_host_byte_order(uint32_t value) {
        return value;
    }

#else // The host has a big-endian architecture

    /**
     * \brief Converts host byte order (big-endian) to Sick LMS byte order (little-endian)
     * \param value The 2-byte value to convert to little-endian
     * \return Value in Sick LMS byte order (little-endian)
     */
    inline uint16_t host_to_sick_pls_byte_order( uint16_t value ) {
      return REVERSE_BYTE_ORDER_16(value);
    }

    /**
     * \brief Converts host byte order (big-endian) to Sick LMS byte order (little-endian)
     * \param value The 4-byte value to convertto little-endian
     * \return Value in Sick LMS byte order (little-endian)
     */
    inline uint32_t host_to_sick_pls_byte_order( uint32_t value ) {
      return REVERSE_BYTE_ORDER_32(value);
    }

    /**
     * \brief Converts Sick LMS byte order (little-endian) to host byte order (big-endian)
     * \param value The 2-byte value to convert to big-endian
     * \return Value in host byte order (big-endian)
     */
    inline uint16_t sick_pls_to_host_byte_order( uint16_t value ) {
      return REVERSE_BYTE_ORDER_16(value);
    }

    /**
     * \brief Converts Sick LMS byte order (little-endian) to host byte order (big-endian)
     * \param value The 4-byte value to convert to big-endian
     * \return Value in host byte order (big-endian)
     */
    inline uint32_t sick_pls_to_host_byte_order( uint32_t value ) {
      return REVERSE_BYTE_ORDER_32(value);
    }

#endif /* WORDS_BIGENDIAN */

/*
 * NOTE: Other utility functions can be defined here
 */

} //namespace sickpls
