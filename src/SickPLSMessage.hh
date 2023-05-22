/*!
 * \file SickPLSMessage.hh
 * \brief Definition of class SickPLSMessage.
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

#ifndef SICK_PLS_MESSAGE_HH
#define SICK_PLS_MESSAGE_HH

/* Definition dependencies */
#include <cstring>
#include <netinet/in.h>   
#include "SickMessage.hh"
#include "SickException.hh"

#define CRC16_GEN_POL 0x8005                        ///< Used to compute CRCs

/** Makes a "short" in little endian */
#define MKSHORT(a,b) ((unsigned short) (a) | ((unsigned short)(b) << 8))

#define SICK_PLS_MSG_HEADER_LEN            (4)  ///< Sick LMS message length in bytes
#define SICK_PLS_MSG_PAYLOAD_MAX_LEN     (812)  ///< Sick LMS max payload length in bytes
#define SICK_PLS_MSG_TRAILER_LEN           (2)  ///< Sick LMS message trailer length in bytes 

/* Associate the namespace */
namespace sickpls {

    /**
     * \brief A class to represent all messages sent to and from the Sick PLS
     *
     * This class helps to construct messages to be sent to the Sick. It also
     * provides a container for received messages to be parsed into.
     */
  class SickPLSMessage : public SickMessage< SICK_PLS_MSG_HEADER_LEN, SICK_PLS_MSG_PAYLOAD_MAX_LEN, SICK_PLS_MSG_TRAILER_LEN >
  {

  public:
  
    /** Default constructor. Constructs an empty message (not well-formed!). */
    SickPLSMessage( );

    /** Constructs a frame by using BuildMessage(). */
    SickPLSMessage( uint8_t dest_address, const uint8_t * payload_buffer, unsigned int payload_length );

    /** Constructs a frame using ParseMessage(). */
    explicit SickPLSMessage( uint8_t * message_buffer );

    /** Constructs a well-formed raw frame from input fields. */
    void BuildMessage( uint8_t dest_address, const uint8_t * payload_buffer,
		       unsigned int payload_length );
    
    /** Populates fields from a (well-formed) raw frame. */
    void ParseMessage( const uint8_t * message_buffer ) override;

    /** Gets the address of the frame. */
    [[nodiscard]] uint8_t GetDestAddress( ) const { return _message_buffer[1]; }

    /** Gets the command code associated with the message */
    [[nodiscard]] uint8_t GetCommandCode( ) const { return _message_buffer[MESSAGE_HEADER_LENGTH]; }

    /** Gets the status byte from an LMS response message (NOTE: only applies to Sick LMS response telegrams!) */
    [[nodiscard]] uint8_t GetStatusByte( ) const { return _message_buffer[MESSAGE_HEADER_LENGTH+_payload_length-1]; }
    
    /** Gets the checksum for the message. */
    [[nodiscard]] uint16_t GetChecksum( ) const { return _checksum; }
    
    /** Reset the data associated with this message (for initialization purposes) */
    void Clear( ) override;
    
    /** A debugging function that prints the contents of the message. */
    void Print( ) const override;

    /** Destructor */
    ~SickPLSMessage() override;

  protected:

    /** The checksum (CRC16) */
    uint16_t _checksum{};
    
  private:

    /** Computes the checksum of the frame. */
    static uint16_t _computeCRC( uint8_t * data, unsigned int data_length ) ;

  };

} /* namespace sickpls */
  
#endif //SICK_PLS_MESSAGE_HH
