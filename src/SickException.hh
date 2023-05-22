/*!
 * \file SickException.hh
 * \brief Contains some simple exception classes.
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

#ifndef SICK_EXCEPTION
#define SICK_EXCEPTION

/* Definition dependencies */
#include <string>
#include <exception>

/* Associate the namespace */
namespace sickpls {

    /** \class SickException
     *  \brief Provides a base exception class from
     *         which to derive other Sick exceptions
     */
    class SickException : std::exception {

    public:

        /**
         * \brief A standard constructor
         * \param general_str A descriptive "general" string
         */
        explicit SickException(const std::string& general_str) {
            _detailed_msg = general_str;
        }

        /**
         * \brief A standard constructor
         * \param general_str A descriptive "general" string
         * \param detailed_str A more detailed description
         */
        SickException(const std::string& general_str, const std::string& detailed_str) {
            _detailed_msg = general_str + " " + detailed_str;
        }

        /**
         * \brief From the standard exception library
         */
        [[nodiscard]] const char* what() const noexcept override {
            return _detailed_msg.c_str();
        }

        /**
         * \brief A destructor
         */
        ~SickException() noexcept override = default;

    private:

        /** The string identifier */
        std::string _detailed_msg;

    };

    /**
     * \class SickTimeoutException
     * \brief Makes handling timeouts much easier
     */
    class SickTimeoutException : public SickException {

    public:

        /**
         * \brief A constructor
         */
        SickTimeoutException() :
                SickException("A Timeout Occurred!") {}

        /**
         * \brief A constructor
         * \param detailed_str A more detailed description
         */
        explicit SickTimeoutException(const std::string& detailed_str) :
                SickException("A Timeout Occurred -", detailed_str) {}

        /**
         * \brief A destructor
         */
        ~SickTimeoutException() noexcept override = default;

    };

    /**
     * \class SickIOException
     * \brief Thrown instance where the driver can't
     *        read,write,drain,flush,... the buffers
     */
    class SickIOException : public SickException {

    public:

        /**
         * \brief A constructor
         */
        SickIOException() :
                SickException("ERROR: I/O exception!") {}

        /**
         * \brief Another constructor
         * \param detailed_str A more detailed description
         */
        explicit SickIOException(const std::string& detailed_str) :
                SickException("ERROR: I/O exception -", detailed_str) {}

        /**
         * \brief A destructor
         */
        ~SickIOException() noexcept override = default;

    };

    /**
     * \class SickBadChecksumException
     * \brief Thrown when a received message has an
     *        invalid checksum
     */
    class SickBadChecksumException : public SickException {

    public:

        /**
         * \brief A constructor
         */
        SickBadChecksumException() :
                SickException("ERROR: Bad Checksum!") {}

        /**
         * \brief Another constructor
         * \param detailed_str A more detailed description
         */
        explicit SickBadChecksumException(const std::string& detailed_str) :
                SickException("ERROR: Bad Checksum -", detailed_str) {}

        /**
         * \brief A destructor
         */
        ~SickBadChecksumException() noexcept override = default;

    };

    /**
     * \class SickThreadException
     * \brief Thrown when error occurs during thread
     *        initialization, and uninitialization
     */
    class SickThreadException : public SickException {

    public:

        /**
         * \brief A constructor
         */
        SickThreadException() :
                SickException("ERROR: Sick thread exception!") {}

        /**
         * \brief Another constructor
         * \param detailed_str A more detailed description
         */
        explicit SickThreadException(const std::string& detailed_str) :
                SickException("ERROR: Sick thread exception -", detailed_str) {}

        /**
         * \brief A destructor
         */
        ~SickThreadException() noexcept override = default;

    };

    /**
     * \class SickConfigException
     * \brief Thrown when the driver detects (or the Sick reports)
     *        an invalid config
     */
    class SickConfigException : public SickException {

    public:

        /**
         * \brief A constructor
         */
        SickConfigException() :
                SickException("ERROR: Config exception!") {}

        /**
         * \brief Another constructor
         * \param detailed_str A more detailed description
         */
        explicit SickConfigException(const std::string& detailed_str) :
                SickException("ERROR: Config exception -", detailed_str) {}

        /**
         * \brief A destructor
         */
        ~SickConfigException() noexcept override = default;

    };

    /**
     * \class SickErrorException
     * \brief Thrown when Sick returns an error code
     *        or an unexpected response.
     */
    class SickErrorException : public SickException {

    public:

        /**
         * \brief A constructor
         */
        SickErrorException() :
                SickException("ERROR: Sick returned error code!") {};

        /**
         * \brief Another constructor
         * \param detailed_str A more detailed description
         */
        explicit SickErrorException(const std::string& detailed_str) :
                SickException("ERROR: Sick error -", detailed_str) {}

        /**
         * \brief A destructor
         */
        ~SickErrorException() noexcept override = default;

    };
} /* namespace sickpls */

#endif /* SICK_EXCEPTION */
