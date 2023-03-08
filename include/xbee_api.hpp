#include <stdint.h>
#include <string.h>
#include <json.hpp>
/* json class license information information
The class is licensed under the MIT License:

Copyright � 2013-2022 Niels Lohmann

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the �Software�), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED �AS IS�, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

using json = nlohmann::json;

////////////////////////////////////////////////////////
//          TRANSMISSION/FRAME CONSTANTS              //
////////////////////////////////////////////////////////
//static const uint8_t START_DELIMITER = 0x7E;                                                           // delimiter at the start of every xbee frame
//static const uint8_t COORDINATOR_ADDRESS_64[] = "0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00";      // coordinator address
//static const uint8_t BROADCAST_ADDRESS_64[] = "0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF";        // PAN broadcast address
//static const uint8_t UNKNOWN_ADDRESS_64[] = "0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF";          // unknown address
//static const uint8_t COORDINATOR_ADDRESS_16[] = "0x00, 0x00";                                          // coordinator address
//static const uint8_t BROADCAST_ADDRESS_16[] = "0xFF, 0xFE";                                            // PAN broadcast address

//static const std::string AT_COMMAND_LIST = {};

////////////////////////////////////////////////////////
//                      AT COMMANDS                   //
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
//                      FUNCTIONS                     //
////////////////////////////////////////////////////////

// increment global frame counter
void frameIDIncrement();

// determine if option is hex or ascii
// return size and then option following it in the same array
uint8_t* parseOptions(std::string options);

// calculate xbee frame checksum per xbee documentation
uint8_t calculateCHKSM(uint8_t* framePtr, int frameLength);

////////////////////////////////////////////////////////
//                TRANSMIT DATA FRAMES                //
////////////////////////////////////////////////////////

/* LOCAL AT COMMAND REQUEST
 * query or set AT parameter on LOCAL xbee module
 *
 *      /-----------------------------------------------------------------------------------------------\
 *      |   0  |     1      |      2     |    3   |     4    |    5    |    6    |    7 - N    |   EOF  |
 *      |------|------------|------------|--------|----------|---------|---------|-------------|--------|
 *      | 0x7E | length MSB | length LSB |  0x08  | Frame ID | CMD MSB | CMD LSB | value (OPT) | CHKSUM |
 *      \-----------------------------------------------------------------------------------------------/
 *
 * accepts an AT command and an optional new value. If a new value is supplied, it replaces the corresponding
 * parameter on the xbee module. If no new value is supplied, the parameter is queried from the xbee module
 */
std::vector<uint8_t>* formATFrame(std::string ATCommand, std::string newvalue = "");

/* TX REQUEST
 * transmit data from one xbee module to xbee module whose 64bit and 16bit address is provided
 * by default, send message to coordinator
 *
 *      /------------------------------------------------------------/ /----------------------------------------------------------------/ /------------------\
 *      |   0  |     1      |      2     |    3   |     4    |   5   \ \    12    |     13      |      14     |  15   |  16   |    17   \ \    N    |  EOF   |
 *      |------|------------|------------|--------|----------|-------/ /----------|-------------|-------------|-------|-------|---------/ /---------|--------|
 *      | 0x7E | length MSB | length LSB |  0x10  | Frame ID | 8BYTE \ \ DST ADDR | 2B ADDR MSB | 2B ADDR LSB |  BCR  |  OPT  | RF DATA \ \ RF DATA | CHKSUM |
 *      \------------------------------------------------------------/ /----------------------------------------------------------------/ /------------------/
 *
 * constant addresses:
 * 64 bit coordinator address           = 0x0000000000000000
 * 64 bit broadcast address             = 0x000000000000FFFF
 * 16 bit broadcast/unknown address     = 0xFFFE
 * 16 bit coordinator address           = 0x0000
 * BCR = maximum # of transmission hops -> default = 0x00 = maximum
 * OPT = options -> Default = 0x00
 *      0x01 = disable retries + route repair
 *      0x02 = enable APS encryption (decreases RF payload by 4 bytes)
 */
std::vector<uint8_t>  formTXFrame(std::string RFData, uint64_t dst_64 = 0x0000000000000000, uint16_t dst_16 = 0x0000, uint8_t bcr = 0x00, uint8_t opt = 0x00);
std::vector<uint8_t>  formTXFrame(std::vector<uint8_t> *RFData, uint64_t dst_64, uint16_t dst_16, uint8_t bcr, uint8_t opt);

/* REMOTE AT COMMAND SET/REQUEST
 * query or set AT parameter on remote xbee module; default is to read the passed parameter
 * to read a parameter: supply the AT COMMAND, ADDRESS
 * to set a parameter:  supply the AT COMMAND, ADDRESS, VALUE, and set OPT 0x02
 *
 *      /------------------------------------------------------------/ /----------------------------------------------------------------------------/ /------------------\
 *      |   0  |     1      |      2     |    3   |     4    |   5   \ \    12    |     13      |      14     |  15   |    16   |    17   |    18   \ \    N    |  EOF   |
 *      |------|------------|------------|--------|----------|-------/ /----------|-------------|-------------|-------|---------|---------|---------/ /---------|--------|
 *      | 0x7E | length MSB | length LSB |  0x17  | Frame ID | 8BYTE \ \ DST ADDR | 2B ADDR MSB | 2B ADDR LSB |  OPT  | CMD MSB | CMD LSB | VAL MSB \ \ VAL LSB | CHKSUM |
 *      \------------------------------------------------------------/ /----------------------------------------------------------------------------/ /------------------/
 *
 * OPT = options -> Default = 0x00
 *      0x01 = disable ACK
 *      0x02 = apply change (0 = will not set)
 *      0x10 = send securely
 */
std::vector<uint8_t>*  formATFrame_Remote(std::string ATCommand, uint64_t dst_64, std::string newvalue = "", uint8_t opt = 0x00, uint16_t dst_16 = 0xFFFE);

////////////////////////////////////////////////////////
//                 RECEIVE DATA FRAMES                //
////////////////////////////////////////////////////////

// interpret received AT frame as json
json parseATCR(uint8_t* frame, uint16_t frameLength);

// interpret received MS frame as json
json parseMS(uint8_t* frame, uint16_t frameLength);

// interpret received TS frame as json
json parseTS(uint8_t* frame, uint16_t frameLength);

// interpret received packet as json
json receivePacket(uint8_t* frame, uint16_t frameLength);

// interpret EXPLICIT RX frame as json
json explicitRX(uint8_t* frame, uint16_t frameLength);

// interpret remote AT frame as json
json remoteAT(uint8_t* frame, uint16_t frameLength);

// interpret frame and return data in JSON form
json readFrame(uint8_t* frame);