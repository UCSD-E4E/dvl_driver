#ifndef IODRIVERS_BASE_STATUS_HPP
#define IODRIVERS_BASE_STATUS_HPP

#include "BaseTime.hpp"

namespace iodrivers_base {
    /** This structure holds IO statistics */
    struct Status
    {
        base::Time stamp;

	unsigned int tx; //! count of bytes received
	unsigned int good_rx; //! count of bytes received and accepted
	unsigned int bad_rx; //! count of bytes received and rejected

	Status()
	    : tx(0), good_rx(0), bad_rx(0) {}
    };
}

#endif

