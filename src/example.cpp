#include <iostream>

#include "SickPLS.hh"

using namespace std;
using namespace sickpls;

int main(int argc, char *argv[]) {


    string device_str;
    SickPLS::sick_pls_baud_t desired_baud = SickPLS::SICK_BAUD_38400;

    unsigned int values[SickPLS::SICK_MAX_NUM_MEASUREMENTS] = {0}; // Uses macro defined in SickPLS.hh
    unsigned int num_values = 0;                                   // Holds the number of measurements returned

    /* Check for a device path.  If it's not present, print a usage statement. */
    if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1], "--help") == 0)) {
        cout << "Usage: app PATH [BAUD RATE]" << endl
             << "Ex: app /dev/ttyUSB0 9600" << endl;
        return -1;
    }

    /* Only device path is given */
    if (argc == 2) {
        device_str = argv[1];
    }

    /* Device path and baud are given */
    if (argc == 3) {
        device_str = argv[1];
        if ((desired_baud = SickPLS::StringToSickBaud(argv[2])) == SickPLS::SICK_BAUD_UNKNOWN) {
            cerr << "Invalid baud value! Valid values are: 9600, 19200, 38400, and 500000" << endl;
            return -1;
        }
    }

    /*
     * Instantiate an instance
     */
    SickPLS sick_pls(device_str);

    /*
     * Initialize the Sick LMS 2xx
     */
    try {
        sick_pls.Initialize(desired_baud);
    }

    catch (...) {
        cerr << "Initialize failed! Are you using the correct device path?" << endl;
        return -1;
    }

    /*
     * Acquire a few scans from the Sick LMS
     */
    try {
        for (unsigned int i = 0; i < 10; i++) {
            sick_pls.GetSickScan(values, num_values);
            cout << "\t  Num. Values: " << num_values << endl;
            for (unsigned int val: values) {
                cout << "\t  val: " << val << endl;
            }
        }

    }

        /* Catch anything else and exit */
    catch (...) {
        cerr << "An error occurred!" << endl;
    }

    /*
     * Uninitialize the device
     */
    try {
        sick_pls.Uninitialize();
    }
    catch (...) {
        cerr << "Uninitialize failed!" << endl;
        return -1;
    }

    /* Success! */
    return 0;

    return 0;
}


