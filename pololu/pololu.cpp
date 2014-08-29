/******************************************************************************************
* Simple pololu commands via serial to Arduino
* Nick Rypkema
* August 2014
******************************************************************************************/

/******************************************************************************************
* g++ pololu.cpp -o pololu -lserial
******************************************************************************************/

#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <sstream>

using namespace std;
using namespace LibSerial ;

// This function returns queued data on port, returns empty string if there is no data
// does not block
string read(SerialStream& serial_port)
{
    string result;
    while( serial_port.rdbuf()->in_avail() )
    {
        char next_byte;
        serial_port.get(next_byte);  
        result += next_byte;
    }
    return result;
}

// This function blocks for upto timeout usec and waits for data
// to be available on the port. 
std::string readBlocking(SerialStream& serial_port, int timeout)
{
    while( serial_port.rdbuf()->in_avail() == 0 && timeout > 0 )
    {
        timeout -= 100;
        usleep(100);
    }
    if(timeout < 0)
        return std::string();
    return read(serial_port);
}

void write(SerialStream& serial_port, const std::string& data)
{
    serial_port.write(data.c_str(), data.size());
}

int main(int argc, char** argv)
{
    //
    // Open the serial port.
    //
    SerialStream serial_port ;

    serial_port.Open( "/dev/ttymxc3" ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Set the baud rate of the serial port.
    //
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_9600 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the baud rate." <<  std::endl;
        exit(1) ;
    }
    //
    // Set the number of data bits.
    //
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the character size." << std::endl;
        exit(1) ;
    }
    //
    // Disable parity.
    //
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not disable the parity." << std::endl;
        exit(1) ;
    }
    //
    // Set the number of stop bits.
    //
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the number of stop bits." << std::endl;
        exit(1) ;
    }
    //
    // Turn off hardware flow control.
    //
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not use hardware flow control." << std::endl;
        exit(1) ;
    }
    
    unsigned int input;
    stringstream datawrite;
    cout << "Input servo value between 500 and 5500, or input 0 to exit" << endl;
    cin >> input;
    while (input != 0) {
        datawrite << input << ',' << input << ',' << input << ',' << input << ',' << input << ',' << input << '\n';
        write(serial_port, datawrite.str());
        datawrite.str("");
        datawrite.clear();
        cout << "Input servo value between 500 and 5500, or input 0 to exit" << endl;
        cin >> input;
    }
}
