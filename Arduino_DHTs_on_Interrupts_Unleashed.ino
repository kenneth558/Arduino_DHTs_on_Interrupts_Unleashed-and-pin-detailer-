/*                      So, what does this sketch do?  
 *  This sketch obtains accurate and thorough PCINT data by toggling every digital pin.  Circuit faults that prevent pins from toggling their 
 *  voltage levels will prevent accurate assessment by this sketch, if such pins are served by the PCINT infrastructure.  Original header files are 
 *  still referred to for port mask data, however. 
 * 
 * 
 * 
 * TODO: debug eligible mask the rest of the way
 * remove debugging code and comments
 * add functionalities:
 * make able to integrate into various environments including relay
 * ensure saving of enough info to operate DHT devices on pins that don't have ISR so as to maximize number of DHTs we can handle
 * post a warning about instances where two pins have the same port and mask.  suggest a first-time test of any board: rotate a device through all the pins and make sure you're satisfied with results shown
 * make 2 memory-fragmentation-model modes: default for ensuring this process can never cause memory fragmentation (high-rel applications)
 *      the other mode for maximizing free memory for other processe ( not for high-rel applications, but frees up the most memory it can afford for other processes.  
 *      The way that memory heap fragmentation could happen is if user then decides to revert back to the default memory-fragmentation mode without a reboot.  
 *      Heap fragmentation is detected by comparing the new location of the master heap array to its old location.  If the location has changed to a higher address,
 *      most likely the previous master array memory footprint is unsuitable in size for efficient use by any other process.  That risk is extrapolated 
 *      into a certainty for safety's sake, so a "memory fragmentation" notice will get serial printed to alert you of said memory waste condition.
 *      
 *
 */
#ifndef NOT_AN_INTERRUPT //This macro was introduced on Oct 1, 2013, IDE version 1.5.5-r2
Did you use your system package manager to install an obsolete Arduino IDE rather than downloading the current IDE directly from arduino.cc?
/*
This section is here for one purpose - to trigger a compile-time error if you are compiling with an obsolete Arduino IDE,
If the line above causes a compile-time error, there are two possible reasons listed below.  Reason #1 below is what we are trying to catch.

 1 )  You are using an obsolete IDE.  This will be the case, for example, if you installed your IDE via a Linux repository rather than from arduino.cc
     If above reason #1 is the case, do yourself a favor and install a newer Arduino IDE directly from arduino.cc instead of using your system's package manager.
     See https://github.com/arduino/Arduino/releases/latest
 
 2 )  Your board truly does not have features to necessitate this by design, or you are compiling in a mixed-technology environment.
     If above reason #2 is the case, simply edit this sketch ( comment out or remove the invalid instruction line/section ), then save and recompile it to get rid of the compile-time error.  With your board, you forfeit the ability to use resistors for mildly helpful boot-up options.  Not a big deal at all.
*/
#endif

// You as end-user can specify in next line the pins that you want protected from the signals placed on ISR( PCINT_vect )-capable pins during the DHT discovery process.  These pins will not be mucked with, but nor will they then support an ISR( PCINTn_vect )-serviced DHT device.
const u8 pins_NOT_safe_even_to_make_low_Z_during_testing[ ] = { };
#if defined (SERIAL_PORT_HARDWARE) && defined (LED_BUILTIN)
    const u8 pins_NOT_safe_to_toggle_during_testing[ ] = { SERIAL_PORT_HARDWARE, LED_BUILTIN };
#else
    #ifdef (LED_BUILTIN)
        const PROGMEM u8 pins_NOT_safe_to_toggle_during_testing[ ] = { LED_BUILTIN };
    #else
        const PROGMEM u8 pins_NOT_safe_to_toggle_during_testing[ ] = { };
    #endif
#endif
/*
 * If the line above causes a compile fail, you'll need to adjust it per your needs or upgrade your compiler/IDE
 */
#include "misc_maskportitems.h"
const u8 dht_max_transitions_for_valid_acquisition_stream = 42;
#include "structs.h"
ISRXREF* Isrxref;
//PINXREF* Pinxref;
PORTXREF* Portxref;
ISRSPEC* Isrspec;
PORTSPEC* Portspec;
DEVSPEC* Devspec;

bool mswindows = false;  //Used for line-end on serial outputs.  Will be determined true during run time if a 1 Megohm ( value not at all critical as long as it is large enough ohms to not affect operation otherwise ) resistor is connected from pin LED_BUILTIN to PIN_A0
u8 number_of_ports_found = 0; //Doesn't ever need to be calculated a second time, so make global and calculate in setup
u8 number_of_devices_found = 0;
u8 number_of_populated_isrs = 0;
const PROGMEM unsigned long halftime = ( ( unsigned long ) -1 )>>1;
const PROGMEM u8 allowed_number_consecutive_read_failures = 25;//JUST A GUESS, NOT EVEN EMPIRICS TO SUPPORT THIS
const PROGMEM u8 consecutive_reads_to_verify_device_type = 20;
const PROGMEM u8 best_uSec_time_translate = 100;
u8 number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR = 0; //Needs to be calculated every time discovery of DHT devices function is executed
static port_specific* ptr_to_portspecs_stack;
static port_specific* previous_ptr_to_portspecs_stack = NULL;
static u8 numOfPortsWithAnyDHTDevice;
char* ports_string_in_heap_array;
char* pre_array_boards_ports_string;
byte* ISR_WITH_DHT_port_pinmask_stack_array;
byte* PCMSK_indexwise_array;//One byte per ISR, values change each time PCMSK for that ISR's PCINT changes 
byte* DHT_without_ISR_port_pinmask_stack_array;
byte* previous_ISR_WITH_DHT_port_pinmask_stack_array = NULL;
u8 number_of_elements_in_ISR_part_of_port_pinmask_stack = 0;
#ifndef PCINT_B_vect
    #ifdef PCMSK
    u8 number_of_elements_in_PCMSK_port_pinmask_stack_array = 0;//0-8 This relates to PCMSK of ISR, one element per PCMSK bit serving a DHT
    byte* PCMSK_port_pinmask_stack_array;//Each bit in PCMSK3 starting with FIRST USED bit, ending with last USED bit.  This schema allows for memory space savings
    #endif
    #ifdef PCMSK0
    u8 number_of_elements_in_PCMSK_port_pinmask_stack_array0 = 0;//0-8 This relates to PCMSK0 ( ISR0 ), one element per PCMSK bit serving a DHT
    byte* PCMSK0_port_pinmask_stack_array;//Each bit in PCMSK0 starting with FIRST USED bit, ending with last USED bit.  This schema allows for memory space savings
    #endif
    #ifdef PCMSK1
    u8 number_of_elements_in_PCMSK_port_pinmask_stack_array1 = 0;//0-8 This relates to PCMSK1 ( ISR1 ), one element per PCMSK bit serving a DHT
    byte* PCMSK1_port_pinmask_stack_array;//Each bit in PCMSK1 starting with FIRST USED bit, ending with last USED bit.  This schema allows for memory space savings
    #endif
    #ifdef PCMSK2
    u8 number_of_elements_in_PCMSK_port_pinmask_stack_array2 = 0;//0-8 This relates to PCMSK2 ( ISR2 ), one element per PCMSK bit serving a DHT
    byte* PCMSK2_port_pinmask_stack_array;//Each bit in PCMSK2 starting with FIRST USED bit, ending with last USED bit.  This schema allows for memory space savings
    #endif
    #ifdef PCMSK3
    u8 number_of_elements_in_PCMSK_port_pinmask_stack_array3 = 0;//0-8 This relates to PCMSK3 ( ISR3 ), one element per PCMSK bit serving a DHT
    byte* PCMSK3_port_pinmask_stack_array;//Each bit in PCMSK3 starting with FIRST USED bit, ending with last USED bit.  This schema allows for memory space savings
    #endif
#else
    This sketch does not work on the ATtiny4313 right now
#endif
u8 number_of_elements_in_non_ISR_DHT_ports_stack = 0;

const u8 element_bytes_in_ISR_part_of_port_pinmask = 5;//
const u8 element_bytes_in_DHT_part_of_port_pinmask = 3;
const u8 element_bytes_in_PCMSK_part_of_port_pinmask = 4;//

const u8 millis_DHT_MCU_start_signal_bit[ ] = { 19, 2 };//consider the first one lost due to resolution err and System millis are a little short anyway, so adding one for margin. NOTE - the manufacturer's latest data sheets indicate a trend to reduce the MCU start bit lengths into the uSecs.
//To accommodate MCU start bit lengths less than 1 ms we may need to utilize the TIMER0_COMPB_vect and the associated OCR0B, TCCR0B...to be determined in a later revision of this sketch if wanted. Scheme would be to have OCR0B set to a value higher or lower than OCR0A and have the first ...vect flip the bit low and tell the other one to call the start the dht function forthwith
//FUTURE - uSec values are simply the ones great than the first value in this array, so when the first value is 19, any later value 20 or greater indicates a uSec time length - NOT YET USED
const struct one_line_device_protocols_supported DEV_PROT_DHT11 = {
     millis_DHT_MCU_start_signal_bit[ 0 ], 5000, 5000, dht_max_transitions_for_valid_acquisition_stream //Note that the manufacturer recommends 5 second wait intervals between reads in a continuous reading environment such as this
};

const struct one_line_device_protocols_supported DEV_PROT_DHT22 = {
    millis_DHT_MCU_start_signal_bit[ 0 ], 2000, 5000, dht_max_transitions_for_valid_acquisition_stream 
};

volatile DEVICE_PROTOCOL Devprot[ ] = { DEV_PROT_DHT11, DEV_PROT_DHT22 };// DHT11 first, DHT22 second.  Since it is by value, we can change values as more suitable params are determined via tests even though the original copies are consts

#include "ISRs.h"

int freeRam () 
{ 
  extern int __heap_start, *__brkval; 
  int v; 
  return ( int ) &v - ( __brkval == 0 ? ( int ) &__heap_start : ( int ) __brkval ); 
 }
    
bool compact_the_heap( bool somevar )
{ 
    return( true );
/*        previous_ = ( void* );//before a free instruction, this line will match the names of the pointers
    free ( previous_ISR_WITH_DHT_port_pinmask_stack_array ); //Just as a matter of good practice, doesn't really do anything here
    Elements_in = ( ELEMENTS_IN* )malloc( \
        sizeof( ELEMENTS_IN ) + \
        ( sizeof( ISRSPEC ) * number_of_ISRs ) + \
        ( sizeof( PORTSPEC ) * number_of_ports_found ) + \
        ( sizeof( PINSPEC ) * NUM_DIGITAL_PINS ) + \
         strlen( populated_ports_string ) + 1 \
        );//not using DEVSPEC at this time *//* not using DEVICE_PROTOCOL at this time */

//Prepare to sparse the array:  the following is obsolete
/* */
    u8 trimmed_size_with_ISR = 0;
    u8 trimmed_size_without_ISR = 0;
/*
    for ( u8 tmp_index = 0; tmp_index < number_of_ports_found ; tmp_index++ )
        if ( ( bool ) *&ISR_WITH_DHT_port_pinmask_stack_array[ tmp_index + ( 4 * number_of_ports_found ) ] )
            trimmed_size_with_ISR++;
        else if ( ( bool ) *&ISR_WITH_DHT_port_pinmask_stack_array[ tmp_index + ( 3 * number_of_ports_found ) ] )
            trimmed_size_without_ISR++;
    
    u8 ISR_WITH_DHT_port_pinmask_stack_array_tmp[ element_bytes_in_ISR_part_of_port_pinmask * trimmed_size_with_ISR ];
    u8 DHT_without_ISR_port_pinmask_stack_array_tmp[ element_bytes_in_ISR_part_of_port_pinmask * trimmed_size_without_ISR ];
*/
    free ( ISR_WITH_DHT_port_pinmask_stack_array );
/* */
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "Line 1001" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
/* */
    ISR_WITH_DHT_port_pinmask_stack_array = ( byte* )malloc( element_bytes_in_ISR_part_of_port_pinmask * ( trimmed_size_with_ISR + trimmed_size_without_ISR ) );
//    number_of_elements_in_ISR_part_of_port_pinmask_stack = trimmed_size_with_ISR;  These comments are here as a reminder of what they will become when finalized
//    number_of_elements_in_non_ISR_DHT_ports_stack = trimmed_size_without_ISR;  These comments are here as a reminder of what they will become when finalized
// total number of elements in array stack = ( number_of_elements_in_ISR_part_of_port_pinmask_stack + number_of_elements_in_non_ISR_DHT_ports_stack )
/* */
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "Line 1017" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
/* */
    if( ISR_WITH_DHT_port_pinmask_stack_array == NULL ) return ( false );
/* */
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "Line 2461" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
/* */

    if ( previous_ISR_WITH_DHT_port_pinmask_stack_array != NULL && previous_ISR_WITH_DHT_port_pinmask_stack_array != ISR_WITH_DHT_port_pinmask_stack_array )//  Entered state of possibility of memory fragmentation
    { 
        if ( previous_ISR_WITH_DHT_port_pinmask_stack_array < ISR_WITH_DHT_port_pinmask_stack_array ) mem_frag_alert();
        else mem_defrag_alert();
    }
    previous_ISR_WITH_DHT_port_pinmask_stack_array = ISR_WITH_DHT_port_pinmask_stack_array;

    for( u8 tmp_index = 0; tmp_index < number_of_elements_in_ISR_part_of_port_pinmask_stack; tmp_index++ )
    { 
//        for ( u8 tmp1_index = 0; tmp1_index < element_bytes_in_ISR_part_of_port_pinmask; tmp1_index++ )
//            *&ISR_WITH_DHT_port_pinmask_stack_array[ tmp_index + ( tmp1_index * ( trimmed_size_with_ISR + trimmed_size_without_ISR ) ) ] = ISR_WITH_DHT_port_pinmask_stack_array_tmp[ tmp_index + ( tmp1_index * ( trimmed_size_with_ISR + trimmed_size_without_ISR ) ) ];
    }
    
    //    ptr_ports_with_ISRs_string = &ports_with_ISRs_string[ 0 ];
    //    return ( &ports_with_ISRs_string[ 0 ] );
    //    return ( &ports_with_ISRs_string[ 0 ] );

}

void prep_ports_for_detection()
{
//We malloc here instead of using the stack so we can run strchr on the string
    pre_array_boards_ports_string = ( char* )malloc( 1 + ( as_many_ports_as_a_board_could_ever_have * 2 ) ); //Temporary array. This will be the absolute maximum number of ports that could ever need to be examined for DHT devices on them.
    if ( !pre_array_boards_ports_string )
    {
/* */
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
/* */
        Serial.print( F( "Not able to acquire enough heap memory to properly prepare any DHT device to respond in detection process." ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
/* */   delay( 10000 );
        return;//TODO: handle a return from here with some kind of propriety
    }
    pre_array_boards_ports_string[ 0 ] = 0;
    delay( 5 );//to allow devices to settle down before making their pins into outputs HIGH
    byte eligible_devices_this_port = 0;//If we did this global, it would take a byte
    number_of_ports_found = 0;
//For every pin, check the ports_found to see if the return value is already in there.  If not in the array, add it to the end of the array.  string is best type
    for ( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ ) //purpose for this is purely to determine number of devices connected to ports/pins to size the local-scope array
    {//problems: we need two different protections masks for each port, and #2 what does the next line really do?
        if( !pin_in_protected_arrays( pin ) )
        { 
            char portchar = ( char ) ( digitalPinToPort( pin ) + 64 );                                                      //Compute the alpha of the port
            if ( strchr( pre_array_boards_ports_string, portchar ) == NULL )                                                       //if it is not already found in the array of supporting ports found in pinset
            {
                eligible_devices_this_port = 0;
                pre_array_boards_ports_string[ number_of_ports_found ] = portchar;
                pre_array_boards_ports_string[ ++number_of_ports_found ] = 0;
                for( u8 tmp_pin = pin; tmp_pin < NUM_DIGITAL_PINS; tmp_pin++ )//for remainder of pins
                {
                    if( ( digitalPinToPort( pin ) == digitalPinToPort( tmp_pin ) ) && !pin_in_protected_arrays( tmp_pin ) && tmp_pin != LED_BUILTIN )
                    {
                        eligible_devices_this_port |= digitalPinToBitMask( tmp_pin );
                    }
                }
                *portModeRegister( digitalPinToPort( pin ) ) = eligible_devices_this_port;
                *portOutputRegister( digitalPinToPort( pin ) ) = eligible_devices_this_port;
                pre_array_boards_ports_string[ number_of_ports_found + as_many_ports_as_a_board_could_ever_have ] = eligible_devices_this_port;//by using number_of_ports_found after it got incremented, we continue to allow for zero termed string at beginning
            }
        }
    }
    delay( 2000 );//to allow devices to settle down before expecting DHTs to act right
}

bool build_from_nothing()
{
    u8 duplicate_pin_higher = 0;
    u8 duplicate_pin_lower = 0;
    if ( F_CPU != 16000000 )
    { //In here we change the two checktimes for other clock rates TODO
/* */
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
/* */
        Serial.print( F( "Not working with an expected clock rate, so DHT devices will not be detected." ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
/* */   delay( 10000 );
/* */
    }
    if( !( bool )number_of_ISRs )
    { //In here we change the two checktimes for other clock rates TODO
/* */
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
/* */
        Serial.print( F( "No ISR was found in this microcontroller board, so this product won't perform optimally.  Your very next step should be to check if any PCMSK variables are used with this board, or check connections and reboot if this message might reflect some fault condition..." ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
/* */   delay( 10000 );
/* */
    }
    prep_ports_for_detection();//This should return info like 
//populated ports string, 
//    pre_array_boards_ports_string[ 0 ]
//    byte eligible_devices_this_port
//    number_of_ports_found


//Make a way to keep the info in the array so to avoid having to obtain it all over again


    free ( pre_array_boards_ports_string ); //Just as a matter of good practice if we want to copy-paste this section, doesn't really do anything here
    pre_array_boards_ports_string = ( char* )malloc( as_many_ports_as_a_board_could_ever_have ); //Temporary array. This will be the absolute maximum number of ports that could ever need to be examined for DHT devices on them.
//    if( !pre_array_boards_ports_string ) return ( false ); //Just as a matter of good practice, shouldn't really do anything here
//    if( previous_ISR_WITH_DHT_port_pinmask_stack_array != NULL && previous_ISR_WITH_DHT_port_pinmask_stack_array != pre_array_boards_ports_string )//  Entered state of possibility of memory fragmentation
//        mem_frag_alert();//In this case this would mean the previous working array was quite small, less than 27 bytes
    previous_ISR_WITH_DHT_port_pinmask_stack_array = ( byte* )pre_array_boards_ports_string;//everywhere else, this line will go prior to the free instruction above, and it will better match the names of the pointers
    number_of_ports_found = 0;
    number_of_devices_found = 0;
    pre_array_boards_ports_string[ 0 ] = 0;
    byte eligible_devices_this_port = 0;

//    u8 devspec_index[ NUM_DIGITAL_PINS ];//TODO: take this out if not needed or make sure it is getting used properly

    u8 pre_array_devspec_index[ NUM_DIGITAL_PINS ];//
    u8 populated_port_count = 0;//
//    u8 pre_array_devspec_count = 0;//
    char string_of_all_ports_that_are_populated[ as_many_ports_as_a_board_could_ever_have ] = { 0 };//

//find a better place to do the next line, later on

//For every pin, check the ports_found to see if the return value is already in there.  If not in the array, add it to the end of the array.  string is best type
    for ( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ ) //purpose for this is purely to determine number of ports connected to pins to size the local scope array
    {
        pre_array_devspec_index[ pin ] = 0;
    }
    for ( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ ) //purpose for this is purely to determine number of ports connected to pins to size the local scope array
    {
        if( !pin_in_protected_arrays( pin ) )
        { 
            char portchar = ( char ) ( digitalPinToPort( pin ) + 64 );                                                      //Compute the alpha of the port
            if ( strchr( pre_array_boards_ports_string, portchar ) == NULL )                                                       //if it is not already found in the array of supporting ports found in pinset
            {
                eligible_devices_this_port = 0;
                pre_array_boards_ports_string[ number_of_ports_found ] = portchar;
                pre_array_boards_ports_string[ ++number_of_ports_found ] = 0;
                for( u8 tmp_pin = pin; tmp_pin < NUM_DIGITAL_PINS; tmp_pin++ )//for remainder of pins
                {
                    if( !pin_in_protected_arrays( tmp_pin ) && tmp_pin != LED_BUILTIN )
                        { 
                            eligible_devices_this_port |= digitalPinToBitMask( tmp_pin );
                        }
                }
                eligible_devices_this_port = find_all_dhts_this_port( digitalPinToPort( pin ), eligible_devices_this_port );
                if( eligible_devices_this_port )
                {
                    string_of_all_ports_that_are_populated[ populated_port_count ] = portchar;
                    string_of_all_ports_that_are_populated[ ++populated_port_count ] = 0;
/*  
                    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
                    Serial.setTimeout( 10 ); //
                    while ( !Serial ) { 
                      ; // wait for serial port to connect. Needed for Leonardo's native USB
                    }
                    Serial.print( F( "string_of_all_ports_that_are_populated = " ) );
                    Serial.print( string_of_all_ports_that_are_populated );
                    Serial.print( F( " as pin " ) );
                    Serial.print( pin );
                    Serial.print( F( " brings in the new port" ) );
                    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
                    Serial.flush();
                    Serial.end();
 */

                    for( u8 tmp_pin = pin; tmp_pin < NUM_DIGITAL_PINS; tmp_pin++ )//for remainder of pins again, see all on this port that need a devspec entry
                    {
                        if ( digitalPinToPort( pin ) == digitalPinToPort( tmp_pin ) )
                            if ( eligible_devices_this_port & digitalPinToBitMask( tmp_pin ) )
                            {
                                pre_array_devspec_index[ number_of_devices_found++ ] = tmp_pin;//TODO: see if we need to add one to pin number//building a sparsed array
                                eligible_devices_this_port &= ~digitalPinToBitMask( tmp_pin );//This will make sure that duplicate pins with higher digital pin numbers get ignored
//                                pre_array_devspec_count++;
 /*
                                Serial.begin( 57600 ); //This speed is very dependent on the host's ability
                                Serial.setTimeout( 10 ); //
                                while ( !Serial ) { 
                                  ; // wait for serial port to connect. Needed for Leonardo's native USB
                                }
                                Serial.print( F( "Line 1064, number_of_devices_found: " ) );
                                Serial.print( number_of_devices_found );
                                Serial.print( F( ", pre_array_devspec_count: " ) );
                                Serial.print( number_of_devices_found );
                                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
                                Serial.flush();
                                Serial.end();
 */
                            }
                    }
                }
            }
        }
    }
//At this point we have an array called pre_array_devspec_index holding sparsed pin numbers of devices
// we know the number of elements and stored that number in number_of_devices_found
// array called string_of_all_ports_that_are_populated holding sparsed port letters of device-populated ports
// so we know by that array size - 1 how many ports have devices on them
/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        for( u8 i = 0; i < NUM_DIGITAL_PINS; i++ )
        {
            Serial.print( F( "pre_array_devspec_index[ " ) );//pre_array_devspec_index
            Serial.print( i );
            Serial.print( F( " ] = " ) );
            Serial.print( pre_array_devspec_index[ i ] );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        }
        Serial.print( F( "string_of_all_ports_that_are_populated = " ) );
        Serial.print( string_of_all_ports_that_are_populated );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
 */

//    char populated_ports_string[ populated_port_count + 1 ];
//    strcpy( populated_ports_string, string_of_all_ports_that_are_populated ); //brings data out of heap into stack so that we can free heap thenb load the string into a new heap array
    free ( pre_array_boards_ports_string );//purpose for this heap-scope var was purely to determine number of ports connected to pins to size the setup()-scope array
/* 
        Serial.print( F( "populated_ports_string = " ) );
        Serial.print( populated_ports_string );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */
//    char populated_ports_string[ number_of_ports_found + 1 ] = { 0 };


//AVOID: Using a x-ref table separate from mask stack instead will use two extra bytes for EVERY possible port index between 1 and highest index ( having pins or not )
//Using only a port index embedded before each mask in ISR_WITH_DHT_port_pinmask_stack_array table leaves masks in pick-order, thus forcing a browse-through for every access, but uses the least memory
/*  May not really need:
    u8 number_of_elements_in_all_PCMSK_port_pinmask_stack_array = 0;
    #ifdef PCMSK
        number_of_elements_in_all_PCMSK_port_pinmask_stack_array += 1;
    #endif
    #ifdef PCMSK0
        number_of_elements_in_all_PCMSK_port_pinmask_stack_array += number_of_elements_in_PCMSK_port_pinmask_stack_array0;
    #endif
    #ifdef PCMSK1
        number_of_elements_in_all_PCMSK_port_pinmask_stack_array += number_of_elements_in_PCMSK_port_pinmask_stack_array1;
    #endif
    #ifdef PCMSK2
        number_of_elements_in_all_PCMSK_port_pinmask_stack_array += number_of_elements_in_PCMSK_port_pinmask_stack_array2;
    #endif
    #ifdef PCMSK3
        number_of_elements_in_all_PCMSK_port_pinmask_stack_array += number_of_elements_in_PCMSK_port_pinmask_stack_array3;
    #endif
*/

//instantiating a pro-array for, and similar to ISR_WITH_DHT_port_pinmask_stack_array, so we dual-purpose its pointer
//This uses the "spare while not needed now and similar" pointer so as not to waste the amount of memory space a dedicated pointer would take

//make a pick-ordered array of that size to hold masks and byte for the supplying port's index
/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "line 1179" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */
        /*
 */
    previous_ISR_WITH_DHT_port_pinmask_stack_array = ( byte* )pre_array_boards_ports_string;//Just as a matter of good practice, doesn't really do anything here. everywhere else before a free instruction, this line will better match the names of the pointers
/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "line 1191" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
*/
    free ( previous_ISR_WITH_DHT_port_pinmask_stack_array ); //Just as a matter of good practice, doesn't really do anything here
/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "line 1204: " ) );
        Serial.print( F( "number_of_ISRs = " ) );
        Serial.print( number_of_ISRs );
        Serial.print( F( ", populated_port_count = " ) );
        Serial.print( populated_port_count );
        Serial.print( F( ", number_of_devices_found = " ) );
        Serial.print( number_of_devices_found );
        Serial.print( F( ", populated_ports_string = " ) );
        Serial.print( populated_ports_string );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */

unsigned long int main_array_size_now = sizeof( ISRXREF ) + \
        sizeof( PORTXREF ) + \
        ( sizeof( ISRSPEC ) * number_of_ISRs ) + \
        ( sizeof( PORTSPEC ) * populated_port_count ) + \
        ( sizeof( DEVSPEC ) * number_of_devices_found ) + \
         strlen( string_of_all_ports_that_are_populated );//string_of_all_ports_that_are_populated

        free( ( void* )Isrxref );
    Isrxref = ( ISRXREF* )malloc( main_array_size_now );






/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "Line 1387" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();        Serial.print( F( " " ) );

 */
/*
        sizeof( DEVXREF ) + \
        ( sizeof( ISRSPEC ) * number_of_ISRs ) + \
        ( sizeof( PORTSPEC ) * populated_port_count ) + \
        ( sizeof( DEVSPEC ) * number_of_devices_found ) + \
         strlen( populated_ports_string ) \
        );
*/
/*
ISRXREF* Isrxref;
PINXREF* Pinxref;
PORTXREF* Portxref;
DEVXREF* Devxref;
ISRSPEC* Isrspec;
PORTSPEC* Portspec;
PINSPEC* Pinspec;
DEVSPEC* Devspec;
*/
//???not using DEVSPEC until the next version of this array when we know how many are served by ISRs /* not using DEVICE_PROTOCOL at this time *//* 1592 with*/ //Dev_Prot, 
/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "Memory space of " ) );
        Serial.print( \
        sizeof( ISRXREF ) + \
        sizeof( PORTXREF ) + \
        sizeof( PINXREF ) + \
        sizeof( DEVXREF ) + \
        ( sizeof( ISRSPEC ) * number_of_ISRs ) + \
        ( sizeof( PORTSPEC ) * number_of_ports_found ) + \
        ( sizeof( PINSPEC ) * NUM_DIGITAL_PINS ) + \
        ( sizeof( DEVSPEC ) * number_of_devices_found ) + \
         strlen( populated_ports_string ) \
        );
        Serial.print( F( " bytes was just allocated" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */
/*
 


        ( sizeof( DHT_NO_ISR ) * NUM_DIGITAL_PINS ) + \
 * Goal is to minimize memory useage by only storing ports, pins, and isrs that have DHT devices on them, let's call it stuffing or sparsing
 * That means the indexes will no longer conform to the original board-defined indexes but rather to found order ( pick order ) by pin number traverse order
 * The ISR list starts in ISR index order for the purpose of ISR discovery, but must end up different as well.  It won't be in pick order necessarily, 
 * but holes in the index continuum for ISRs must be removed and thereafter ISR indexes will be decreased
 * 
 */
 /* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "line 2302" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */
/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "line 1265" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */

    if( !Isrxref )
    { //Come up with a better way to handle this
 /* */ 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "Memory space of " ) );
        Serial.print( main_array_size_now );
/*
        sizeof( DEVXREF ) + \
        ( sizeof( ISRSPEC ) * number_of_ISRs ) + \
        ( sizeof( PORTSPEC ) * number_of_ports_found ) + \
        ( sizeof( PINSPEC ) * NUM_DIGITAL_PINS ) + \
        ( sizeof( DEVSPEC ) * number_of_devices_found ) + \
         strlen( populated_ports_string ) \
        );
*/
        Serial.print( F( " bytes was refused allocation.  Aborting..." ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
/* */
//        ( sizeof( DEVICE_PROTOCOL ) * protocols_supported ) + \  ISR_WITH_DHT_port_pinmask_stack_array

        return ( false );

    }
/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "Sparsed malloc just done" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
  */
/*  Order:
/*
ISRXREF* Isrxref;
PINXREF* Pinxref;
PORTXREF* Portxref;
DEVXREF* Devxref;//takes pin numbers as index in, should also take isr/PCIbitmask combo?
ISRSPEC* Isrspec;
PORTSPEC* Portspec;
PINSPEC* Pinspec;
DEVSPEC* Devspec;

Crossing:   To go from ISR/PCIbitmask to Devspec:  convert PCIbitmask to index for index to array_of_all_pinnums_plus_one_this_ISR,  Isrxref->ISR_xref[ ISR# ]->array_of_all_devspec_index_plus_1_this_ISR[ ]

            u8 _bit = PCIbitmask;
            u8 counter = 0;
            for( ; _bit >>= 1; counter++ );
            Isrspec[ Isrxref->ISR_xref[ ? ] ].array_of_all_devspec_index_plus_1_this_ISR[ counter ] = 

Crossing:   To go from / to Devspec:  convert PCIbitmask to index for index to array_of_all_pinnums_plus_one_this_ISR,  Isrxref->ISR_xref[ ISR# ]->array_of_all_devspec_index_plus_1_this_ISR[ ]

            u8 _bit = PCIbitmask;
            u8 counter = 0;
            for( ; _bit >>= 1; counter++ );
            Isrspec[ Isrxref->ISR_xref[ ? ] ].array_of_all_devspec_index_plus_1_this_ISR[ counter ]
*/
//number_of_ISRs is the number of ISRs for this board
//sizeof( ISRSPEC ) * number_of_ISRs memory space needed for one ISRSPEC per ISR
//sizeof( PORTSPEC ) * number_of_ports_found memory space needed for one PORTSPEC per port
    if( previous_ISR_WITH_DHT_port_pinmask_stack_array != NULL && previous_ISR_WITH_DHT_port_pinmask_stack_array != ( byte* )Isrxref )//  Entered state of possibility of memory fragmentation
        if ( previous_ISR_WITH_DHT_port_pinmask_stack_array < ( byte* )Isrxref ) mem_frag_alert();
        else mem_defrag_alert();
    number_of_elements_in_ISR_part_of_port_pinmask_stack = number_of_ports_found;//This is just a temporary number until detection and re-allocate the array
    previous_ISR_WITH_DHT_port_pinmask_stack_array = ( byte* )Isrxref;
/*  Order:
/*
ISRXREF* Isrxref;
PINXREF* Pinxref;
PORTXREF* Portxref;
DEVXREF* Devxref;
ISRSPEC* Isrspec;
PORTSPEC* Portspec;
PINSPEC* Pinspec; 
DEVSPEC* Devspec;
*/
//    Pinxref = ( PINXREF* )
    Portxref = ( PORTXREF* )( ( long unsigned int )Isrxref + sizeof( ISRXREF ) );
    Isrspec = ( ISRSPEC* )( ( long unsigned int )Portxref + sizeof( PORTXREF ) );
    Portspec = ( PORTSPEC* )( ( long unsigned int )Isrspec + ( sizeof( ISRSPEC) * number_of_ISRs ) );
    Devspec = ( DEVSPEC* )( ( long unsigned int )Portspec + ( sizeof( PORTSPEC ) * populated_port_count ) );
    ports_string_in_heap_array = ( char* )( ( long unsigned int )Devspec + ( sizeof( DEVSPEC ) * number_of_devices_found ) );
    strcpy( ports_string_in_heap_array, string_of_all_ports_that_are_populated );//This makes ports_string_in_heap_array not suitable for interrupt findings if any pins served by interrupts don't have devices on them!!!
/* 
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
*/ /*
    Serial.print( F( "Devxref = " ) );
    Serial.print( ( long unsigned int )Devxref );
    Serial.print( F( ", Devxref - Isrxref = " ) );
    Serial.print( ( long unsigned int )Devxref - ( long unsigned int )Isrxref );
    Serial.print( F( " and should be " ) );
    Serial.print( sizeof( ISRXREF ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Pinxref = " ) );
    Serial.print( ( long unsigned int )Pinxref );
    Serial.print( F( ", Pinxref - Devxref = " ) );
    Serial.print( ( long unsigned int )Pinxref - ( long unsigned int )Devxref );
    Serial.print( F( " and should be " ) );
    Serial.print( sizeof( DEVXREF ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Portxref = " ) );
    Serial.print( ( long unsigned int )Portxref );
    Serial.print( F( ", Portxref - Pinxref = " ) );
    Serial.print( ( long unsigned int )Portxref - ( long unsigned int )Pinxref );
    Serial.print( F( " and should be " ) );
    Serial.print( sizeof( PINXREF ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Isrspec = " ) );
    Serial.print( ( long unsigned int )Isrspec );
    Serial.print( F( ", Isrspec - Portxref = " ) );
    Serial.print( ( long unsigned int )Isrspec - ( long unsigned int )Portxref );
    Serial.print( F( " and should be " ) );
    Serial.print( sizeof( PORTXREF ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Portspec = " ) );
    Serial.print( ( long unsigned int )Portspec );
    Serial.print( F( ", Portspec - Isrspec = " ) );
    Serial.print( ( long unsigned int )Portspec - ( long unsigned int )Isrspec );
    Serial.print( F( " and should be " ) );
    Serial.print( sizeof( ISRSPEC) * number_of_ISRs );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Pinspec = " ) );
    Serial.print( ( long unsigned int )Pinspec );
    Serial.print( F( ", Pinspec - Portspec = " ) );
    Serial.print( ( long unsigned int )Pinspec - ( long unsigned int )Portspec );
    Serial.print( F( " and should be " ) );
    Serial.print( sizeof( PORTSPEC ) * number_of_ports_found );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Devspec = " ) );
    Serial.print( ( long unsigned int )Devspec );
    Serial.print( F( ", Devspec - Pinspec = " ) );
    Serial.print( ( long unsigned int )Devspec - ( long unsigned int )Pinspec );
    Serial.print( F( " and should be " ) );
    Serial.print( sizeof( PINSPEC ) * NUM_DIGITAL_PINS );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
*/
/*
    Serial.print( F( "ports_string_in_heap_array = " ) );
    Serial.print( ports_string_in_heap_array );
*/
/*
    Serial.print( F( ", ports_string_in_heap_array - Devspec = " ) );
    Serial.print( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec );
    Serial.print( F( " and should be " ) );
    Serial.print( sizeof( DEVSPEC ) * number_of_devices_found  );
*/
/*
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.flush();
    Serial.end();
*/
/*
 * 
 * Example for Isrxref:
 *     for ( u8 i = 0; i < ( ( long unsigned int )Pinxref - ( long unsigned int )Isrxref ) / sizeof( ISRXREF ); i++ )
 * Example for Pinxref:
 *     for ( u8 i = 0; i < ( ( long unsigned int )Portxref - ( long unsigned int )Pinxref ) / sizeof( PINXREF ); i++ )
 * Example for Portxref:
 *     for ( u8 i = 0; i < ( ( long unsigned int )Devxref - ( long unsigned int )Portxref ) / sizeof( PORTXREF ); i++ )
 * Example for Devxref:
 *     for ( u8 i = 0; i < ( ( long unsigned int )Isrspec - ( long unsigned int )Devxref ) / sizeof( DEVXREF ); i++ )
 * Example for Isrspec:
 *     for ( u8 i = 0; i < ( ( long unsigned int )Portspec - ( long unsigned int )Isrspec ) / sizeof( ISRSPEC ); i++ )
 * Example for Portspec:
 *     for ( u8 i = 0; i < ( ( long unsigned int )Pinspec - ( long unsigned int )Portspec ) / sizeof( PORTSPEC ); i++ )
 * Example for Pinspec:  OBSOLETE
 *     for ( u8 i = 0; i < ( ( long unsigned int )Devspec - ( long unsigned int )Pinspec ) / sizeof( PINSPEC ); i++ )
 * Example for Devspec:
 *     for ( u8 i = 0; i < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); i++ )
 * Example for ports_string_in_heap_array:
 *     for ( u8 i = 0; i < sizeof( ports_string_in_heap_array ) - 1; i++ )
 */
//Fill with neutral, non-sparsed values
    for ( u8 i = 0; i < number_of_ISRs; i++ )
    {
        Isrxref->ISR_xref[ i ] = i;
        Isrxref->my_isrspec_addr[ i ] = &Isrspec[ i ];//Will only work if and because Isrspec has not been sparsed, yet
    }
//    for( u8 i = 0; i < sizeof( Pinxref->PIN_xref_dev ); i++ )
//        Pinxref->PIN_xref_dev[ i ] = 255;//MOST OF THESE WILL GO TO WASTE
//    for( u8 i = 0; i < sizeof( Devxref->DEV_xref ); i++ )
//        Devxref->DEV_xref[ i ] = devspec_index[ i ] - 1; //pin number gets stored in xref
/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "Line 1876, device_busy_resting_this_more_millis getting assigned" ) );
//        Serial.print( number_of_devices_found );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );

        Serial.flush();
        Serial.end();
*/
/*
                        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
                        Serial.setTimeout( 10 ); //
                        while ( !Serial ) { 
                          ; // wait for serial port to connect. Needed for Leonardo's native USB
                        }
                        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
                        Serial.print( F( "Waiting for you to send a character..." ) );  
                        while (Serial.available() == 0);
                        char userresponse = Serial.read();
//                        Serial.print( this_Isrspec_address->next_bit_coming_from_dht );  
                        Serial.flush();
                        Serial.end();
 */
delay( 5 );//This is to let all dht devices that got triggered to end their data streams
    for( u8 i = 0; i < number_of_devices_found; i++ )//make sure we never make the number of elements in devspec_index a different amount than this line thinks
    {
        digitalWrite( pre_array_devspec_index[ i ], HIGH );
//        Pinxref[ i ].Dpin = pre_array_devspec_index[ i ];
        Devspec[ i ].Dpin = pre_array_devspec_index[ i ];
        for( u8 ij = 0; ij < sizeof( Devspec[ i ].last_valid_data_bytes_from_dht_device )/ sizeof( Devspec[ i ].last_valid_data_bytes_from_dht_device[ 0 ] ); ij++ )
            Devspec[ i ].last_valid_data_bytes_from_dht_device[ ij ] = 0;
        Devspec[ i ].timestamp_of_pin_valid_data_millis = 0;
        Devspec[ i ].devprot_index = 0;
        Devspec[ i ].consecutive_read_failures = 0;
        Devspec[ i ].consecutive_read_successes = 0;
        Devspec[ i ].start_time_plus_max_acq_time_in_uSecs = 0;
        long unsigned timenow = millis();//A single point of reference to prevent changing during the following
        Devspec[ i ].device_busy_resting_this_more_millis = Devprot[ Devspec[ i ].devprot_index ].millis_rest_length;
        //          Devspec[ i ].device_busy_resting_this_more_millis = timenow + Devprot[ Devspec[ i ].devprot_index ].millis_rest_length;
//        Devspec[ i ].millis_will_overflow = false;
//        Devspec[ i ].micros_will_overflow = false;
        if( !Devspec[ i ].device_busy_resting_this_more_millis )
        {
          Devspec[ i ].device_busy_resting_this_more_millis++; //zero is not a valid value unless device is rested
        }
//        if( Devspec[ i ].device_busy_resting_this_more_millis < timenow ) Devspec[ i ].millis_will_overflow = true;
        Devspec[ i ].mask_in_port = digitalPinToBitMask( pre_array_devspec_index[ i ] );
        Devspec[ i ].output_port_reg_addr = portOutputRegister( digitalPinToPort( Devspec[ i ].Dpin ) );
        Devspec[ i ].ddr_port_reg_addr = portModeRegister( digitalPinToPort( Devspec[ i ].Dpin ) );
        Devspec[ i ].pin_reg_addr = portInputRegister( digitalPinToPort( Devspec[ i ].Dpin ) );
        Devspec[ i ].index_of_next_valid_readings_sets = 0;
/*
Devspec[ i ].debug_PCICR = 0;
Devspec[ i ].debug_pcmsk = 0;
Devspec[ i ].debug_device_busy_resting_this_more_millis = 0;
Devspec[ i ].debug_timestamp_of_pin_last_attempted_device_read_millis = 0;
Devspec[ i ].debug_start_time_plus_max_acq_time_in_uSecs = 0;
Devspec[ i ].debug_start_time_plus_max_acq_time_in_uSecs = 0;
Devspec[ i ].debug_active_pin_ddr_port_reg_addr = 0;
Devspec[ i ].debug_active_pin_output_port_reg_addr = 0;
Devspec[ i ].debug_ddr_b4 = 0;
*/
//        Serial.print( F( "Devspec[ i ].Dpin = " ) );
//        Serial.print( Devspec[ i ].Dpin );
//        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
          

/*        if( devspec_index[ i ] )
//        {
//This loop does not seem to take
            Devspec[ Pinxref->PIN_xref[ i ] ].Dpin = devspec_index[ i ] - 1;
            //Isrxref.array_of_all_devspec_index_plus_1_this_ISR[ index_in_PCMSK_of_current_device_within_ISR ] = ;
            Devspec[ devspec_index[ i ] - 1 ].last_valid_data_bytes_from_dht_device[ 0 ] = 0;
            Devspec[ devspec_index[ i ] - 1 ].last_valid_data_bytes_from_dht_device[ 1 ] = 0;
            Devspec[ devspec_index[ i ] - 1 ].last_valid_data_bytes_from_dht_device[ 2 ] = 0;
            Devspec[ devspec_index[ i ] - 1 ].last_valid_data_bytes_from_dht_device[ 3 ] = 0;
            Devspec[ devspec_index[ i ] - 1 ].timestamp_of_pin_valid_data_millis = 0;
            Devspec[ devspec_index[ i ] - 1 ].timestamp_of_pin_last_attempted_device_read_millis = millis();
            Devspec[ devspec_index[ i ] - 1 ].next_bit_coming_from_dht = 0;
            Devspec[ devspec_index[ i ] - 1 ].device_busy_resting_this_more_millis = millis() + 2000;
            Devspec[ devspec_index[ i ] - 1 ].millis_will_overflow = false;
            Devspec[ devspec_index[ i ] - 1 ].devprot_index = 1;
//        }
*/
    }
delay( 2000 );//ensure all devices get a rest period right here
/*
        Serial.flush();
        Serial.end();
*/
    for( u8 fill_index_in_heap_proarray = 0; fill_index_in_heap_proarray < populated_port_count; fill_index_in_heap_proarray++ )
    {
        Portspec[ fill_index_in_heap_proarray ].this_port_index_in_order_of_discovery_traversing_pins = fill_index_in_heap_proarray;//not necessary?
        Portspec[ fill_index_in_heap_proarray ].this_port_index_from_core_lookup_alphabetic_order = string_of_all_ports_that_are_populated[ fill_index_in_heap_proarray ] - 64;
        Portspec[ fill_index_in_heap_proarray ].this_port_main_reg = ( u8* )portOutputRegister( string_of_all_ports_that_are_populated[ fill_index_in_heap_proarray ] - 64 );
        Portspec[ fill_index_in_heap_proarray ].this_port_pin_reg = ( u8* )portInputRegister( string_of_all_ports_that_are_populated[ fill_index_in_heap_proarray ] - 64 );
        Portspec[ fill_index_in_heap_proarray ].this_port_mode_reg = ( u8* )portModeRegister( string_of_all_ports_that_are_populated[ fill_index_in_heap_proarray ] - 64 );
        Portspec[ fill_index_in_heap_proarray ].mask_of_safe_pins_to_detect_on = 0;
        Portspec[ fill_index_in_heap_proarray ].mask_of_unsafe_pins_even_to_make_low_Z = 0;
        for( u8 prot_pin_index = 0; prot_pin_index < sizeof( pins_NOT_safe_even_to_make_low_Z_during_testing ); prot_pin_index++ )
            if( digitalPinToPort( pins_NOT_safe_even_to_make_low_Z_during_testing[ prot_pin_index ] ) == Portspec[ fill_index_in_heap_proarray ].this_port_index_from_core_lookup_alphabetic_order ) Portspec[ fill_index_in_heap_proarray ].mask_of_unsafe_pins_even_to_make_low_Z |= digitalPinToBitMask( pins_NOT_safe_even_to_make_low_Z_during_testing[ prot_pin_index ] );
        Portspec[ fill_index_in_heap_proarray ].mask_of_unsafe_pins_to_toggle_and_otherwise = 0;
        for( u8 prot_pin_index = 0; prot_pin_index < sizeof( pins_NOT_safe_to_toggle_during_testing ); prot_pin_index++ )
            if( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ prot_pin_index ] ) == Portspec[ fill_index_in_heap_proarray ].this_port_index_from_core_lookup_alphabetic_order ) Portspec[ fill_index_in_heap_proarray ].mask_of_unsafe_pins_to_toggle_and_otherwise |= digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ prot_pin_index ] );
        Portspec[ fill_index_in_heap_proarray ].mask_of_real_pins_this_port = 0;
        for( u8 _pin_ = 0; _pin_ < NUM_DIGITAL_PINS; _pin_++ )
            if( digitalPinToPort( _pin_ ) == Portspec[ fill_index_in_heap_proarray ].this_port_index_from_core_lookup_alphabetic_order ) Portspec[ fill_index_in_heap_proarray ].mask_of_real_pins_this_port |= digitalPinToBitMask( _pin_ );
        Portspec[ fill_index_in_heap_proarray ].mask_of_DHT_devices_this_port = 0;
        Portspec[ fill_index_in_heap_proarray ].PCINT_pins_mask = 0;
        Portspec[ fill_index_in_heap_proarray ].timestamp_of_last_portwide_device_detection_action_by_thisport_micros = micros();
        

        u8 l = 0;
//        Serial.print( F( ", searching through devspecs, " ) );
        for( ; l < number_of_devices_found; l++ )
        {
            if( Portspec[ fill_index_in_heap_proarray ].this_port_index_from_core_lookup_alphabetic_order == digitalPinToPort( Devspec[ l ].Dpin ) )
            {
//                Serial.print( F( " for pin " ) );
//                Serial.print( Devspec[ l ].Dpin );
//                Serial.print( F( " compared to pinxref_index = " ) );
//                Serial.print( pinxref_index );
//            if ( Devspec[ l ].Dpin == pinxref_index )
//                Serial.print( F( " found pin match in Devspec index " ) );
//                    Serial.print( l );
//                    Serial.print( F( ", making " ) );
//                Serial.print( Devspec[ l ].Dpin );
//                Serial.print( F( " = " ) );
//                Serial.print( Devspec[ l ].portspec_index_for_pin );
            }
        }
    }


 /* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( " " ) );
        Serial.print( );
        Serial.print( F( " " ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */

    
    
    for( u8 i = 0; i < sizeof( Portxref->PORT_xref ); i++ )
        Portxref->PORT_xref[ i ] = i;

//adjust so ptr is at first ISRSPEC element to eliminate later calculations to find it
//    ISR_WITH_DHT_port_pinmask_stack_array += sizeof( ELEMENTS_IN ) + ( sizeof( ISRSPEC ) * number_of_ISRs );

//Fill in the ELEMENTS_IN info

/* 
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Trying to print out Elements_in structure, located at : " ) );
    Serial.print( ( num_element_list_of_structs* )&Elements_in->ISRSPEC - Elements_in );
    Serial.print( F( " offset beyond Elements_in. Contents of Elements_in->ISRSPEC, etc. =" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "located at : " ) );
    Serial.print( ( num_element_list_of_structs* )&Elements_in->ISRSPEC - Elements_in );
    Serial.print( F( " is " ) );
    Serial.print( Elements_in->ISRSPEC );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "located at : " ) );
    Serial.print( ( num_element_list_of_structs* )&sizeof PORTSPEC / sizeof *PORTSPEC - Elements_in );
    Serial.print( F( " is " ) );
    Serial.print( sizeof PORTSPEC / sizeof *PORTSPEC );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "located at : " ) );
    Serial.print( ( num_element_list_of_structs* )&Elements_in->PINSPEC - Elements_in );
    Serial.print( F( " is " ) );
    Serial.print( Elements_in->PINSPEC );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "located at : " ) );
    Serial.print( ( num_element_list_of_structs* )&Elements_in->DEVSPEC - Elements_in );
    Serial.print( F( " is " ) );
    Serial.print( Elements_in->DEVSPEC );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "located at : " ) );
    Serial.print( ( num_element_list_of_structs* )&Elements_in->DHT_NO_ISR - Elements_in );
    Serial.print( F( " is " ) );
    Serial.print( Elements_in->DHT_NO_ISR );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "located at : " ) );
    Serial.print( ( num_element_list_of_structs* )&Elements_in->DEVICE_PROTOCOL - Elements_in );
    Serial.print( F( " is " ) );
    Serial.print( Elements_in->DEVICE_PROTOCOL );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "The dash before the PORT name ( -PORTn ) means that is the first occurrance of the port when examining the pins in pin number order" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
 */

/* */
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "LIST OF " ) );
    Serial.print( NUM_DIGITAL_PINS );
//    Serial.print( F( " ( " ) );
//    Serial.print( NUM_DIGITAL_PINS );
//    Serial.print( F( " )") );
    Serial.print( F( " DIGITAL PINS" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//    Serial.print( F( "sizeof( PINSPEC ) = " ) );
//    Serial.print( sizeof( PINSPEC ) );
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//    Serial.print( F( "Devspec - Pinspec = " ) );
//    Serial.print( ( long unsigned int )Devspec - ( long unsigned int )Pinspec );
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//    Serial.print( F( "( long unsigned int )Devspec = " ) );
//    Serial.print( ( long unsigned int )Devspec );
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//    Serial.print( F( "( long unsigned int )Pinspec = " ) );
//    Serial.print( ( long unsigned int )Pinspec );
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//    Serial.print( F( "The dash before the PORT name ( -PORTn ) means that is the first occurrance of the port when examining the pins in pin number order" ) );
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );

/* */
//    populated_ports_string[ 0 ] = 0; //local_scope_populated_ports_string
//    u8 fill_index_in_heap_proarray = 0;
    for ( u8 pinxref_index = 0; pinxref_index < NUM_DIGITAL_PINS; pinxref_index++ )
    {
//        u8 pin = Pinxref->PIN_xref_dev[ pinxref_index ];
//        Serial.print( F( "New pin: " ) );
        Serial.print( F( "Pin D" ) );
        Serial.print( pinxref_index );
        if ( pinxref_index < 10 ) Serial.print( F( "  " ) );
        else Serial.print( F( " " ) );
        print_analog_if_exists( pinxref_index );
//        Serial.print( F( ":" ) );
//        Serial.print( F( ", Pinxref->PIN_xref_dev[ pinxref_index ] = " ) );
//        Serial.print( pinxref_index );
//        Serial.print( F( ", fill_index_in_heap_proarray = " ) );
//        Serial.print( fill_index_in_heap_proarray );
        Serial.print( F( ":" ) );
/* */
        char portchar = ( char ) ( digitalPinToPort( pinxref_index ) + 64 );                                                      //Compute the alpha of the port
/*
//the following does not give us sparsed values.  Fix it  TODO
        if ( strchr( ports_string_in_heap_array, portchar ) == NULL )                                                      //if it is not already found in the array of supporting ports found in pinset
        {
            ports_string_in_heap_array[ fill_index_in_heap_proarray++ ] = portchar;//?????This is not getting done!!!!
            ports_string_in_heap_array[ fill_index_in_heap_proarray ] = 0;
*/
//        Serial.print( F( ", ports_string_in_heap_array = " ) );
//        Serial.print( ports_string_in_heap_array );
//        Serial.print( F( ", " ) );
//            Serial.print( F( "-" ) );

/*  Order:
/*
ISRXREF* Isrxref;
PINXREF* Pinxref;
PORTXREF* Portxref;
DEVXREF* Devxref;
ISRSPEC* Isrspec;
PORTSPEC* Portspec;
PINSPEC* Pinspec;
DEVSPEC* Devspec;
*/
/*                
                
                for( u8 ndx = 1; ndx < element_bytes_in_ISR_part_of_port_pinmask - 1; ndx++ )
                { 
                    Portspec[ fill_index_in_heap_proarray ].[ ndx ] = B0;  
                }
                //ndx = 0 That produced the port index array element 
                //ndx = 1 That produced the array element of physical pins mask found in port
                //ndx = 2 That produced the array element of testing-eligible pins mask this for port
                //ndx = 3 That produced the array element of DHT-found pins mask for this port
                //ndx = 4 That produced the array element of PCINT pins mask this for port

*/
//            *&Portspec[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * ( element_bytes_in_ISR_part_of_port_pinmask - 1 ) ) + fill_index_in_heap_proarray++ ] = B0;  

                
                
                //ndx = element_bytes_in_ISR_part_of_port_pinmask - 1 That produced the array element of PCINT pins connected to any DHT device mask this for port
/* 
                Serial.print( F( "-PORT" ) );
//given the second port in the array: 'B' at 
                Serial.print( ( char )( *&Portspec[ fill_index_in_heap_proarray - 1 ] + 64 ) );
                Serial.print( F( " " ) );
                Serial.print( digitalPinToBitMask( pin ), BIN );
                Serial.print( F( " " ) );
                Serial.print( ( char )( *&Portspec[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 0 ) + fill_index_in_heap_proarray - 1 ] + 64 ) );  
                Serial.print( F( " " ) );
                Serial.print( *&Portspec[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 1 ) + fill_index_in_heap_proarray - 1 ], BIN );  
                Serial.print( F( " " ) );
                Serial.print( *&Portspec[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 2 ) + fill_index_in_heap_proarray - 1 ], BIN );  
                Serial.print( F( " " ) );
                Serial.print( *&Portspec[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 3 ) + fill_index_in_heap_proarray - 1 ], BIN );  
                Serial.print( F( " " ) );
                Serial.print( *&Portspec[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 4 ) + fill_index_in_heap_proarray - 1 ], BIN );  
//                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//                Serial.print( F( " ( at index " ) );
//                Serial.print( fill_index_in_heap_proarray - 1 );
 */
 /*
        }
        else
        { 
            Serial.print( F( " " ) );
        }
*/
            Serial.print( F( "PORT" ) );
            Serial.print( portchar );
            Serial.print( F( " " ) );
//            Serial.print( digitalPinToBitMask( pin ), BIN );
            Serial.print( F( "bit " ) );
            u8 _bit = digitalPinToBitMask( pinxref_index );
            u8 counter = 0;
            for( ; _bit >>= 1; counter++ );
            Serial.print( counter );


//            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
/*
            Serial.print( F( ", digitalPinToPCICR " ) );
            Serial.print( ( unsigned long )digitalPinToPCICR( pin ) );
//            if ( ( u8 )( digitalPinToPCICRbit( pin ) + 1 ) )
//            { 
                Serial.print( F( ", digitalPinToPCICRbit " ) );
                Serial.print( bit( digitalPinToPCICRbit( pin ) ),BIN );
                Serial.print( F( ", digitalPinToPCMSK " ) );
                Serial.print( ( unsigned long )digitalPinToPCMSK( pin ),BIN );
                Serial.print( F( ", digitalPinToPCMSKbit " ) );
                Serial.print( bit( digitalPinToPCMSKbit( pin ) ),BIN );
//            }
*/
        for( u8 i = 0; i < number_of_devices_found; i++ )//make sure we never make the number of elements in devspec_index a different amount than this line thinks
        {
            if( Devspec[ i ].Dpin == pinxref_index ) Serial.print( F( " --DHT connected--" ) );
        }
        if( pin_in_protected_arrays( pinxref_index ) )
        { 
            Serial.print( F( " pin is protected by it being listed in a protected pins array" ) );
        }
        
        if ( pinxref_index == LED_BUILTIN )
        { 
            Serial.print( F( ", LED_BUILTIN " ) );//compiler ( one version in Linux Mint, at least ) is so problematic with printing the word "builtin" ( either case ) as the last thing on the line that we can't do it straightforwardly, so the space is added to end.  Simply amazing...
        }
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        if( !duplicate_pin_higher )
        {
            for( u8 dup_pin_index = pinxref_index + 1; dup_pin_index < NUM_DIGITAL_PINS; dup_pin_index++ )
            {
                if( digitalPinToPort( dup_pin_index ) == digitalPinToPort( pinxref_index ) && digitalPinToBitMask( dup_pin_index ) == digitalPinToBitMask( pinxref_index ) )
                {
                    duplicate_pin_higher = dup_pin_index;
                    duplicate_pin_lower = pinxref_index;
                    break;
                }
            }
        }
    }
    if( duplicate_pin_higher )
    {
        Serial.print( F( "Note: starting with digital pin number " ) );
        Serial.print( duplicate_pin_higher );
        Serial.print( F( " that is a duplicate of digital pin number " ) );
        Serial.print( duplicate_pin_lower );
        Serial.print( F( ", one" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "or more of these pins are virtual pins that are duplicates of real ones that have a lower" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "number, in which case a device connected to such will only show as being connected the" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "lower numbered pin.  Due to general Arduino memory space limitations, this sketch only lets" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "you know of the first one." ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    }
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//Now we know how many ports are serving pins
/*  
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */

/* 
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
*/
/*
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "line 1787 number_of_devices_found = " ) );
        Serial.print( number_of_devices_found );
*/
//REMOVING DEPENDENCE ON PINSPEC, but it means we lose track of duplicate pins: NO GOOD in case a pin with a duplicate is populated with a device!!!!
    for ( u8 devspec_index = 0; \
    devspec_index < number_of_devices_found; \
    devspec_index++ )
    //loop while within number of elements of pinspec AND the next element will not be null
    //purpose for this is to make the different pin masks for each array port element, and make the Pinspec, Devspec, Devprot array elements
    {
/*
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "New pin: " ) );
        Serial.print( Devspec[ devspec_index ].Dpin );
        if ( Devspec[ devspec_index ].Dpin < 10 ) Serial.print( F( " " ) );
*/
        char portchar = ( char ) ( digitalPinToPort( Devspec[ devspec_index ].Dpin ) + 64 );                                                      //Compute the alpha of the port
        if ( strchr( ports_string_in_heap_array, portchar ) != NULL )                                                      //if it is found in the array of supporting ports found in pinset
        {
            u8 str_index = ( u8 ) ( strchr( ports_string_in_heap_array, portchar ) - ports_string_in_heap_array );//
//Portspec[ strchr( ports_string_in_heap_array, portchar ) - ports_string_in_heap_array ].
            if( Portspec[ str_index ].this_port_index_from_core_lookup_alphabetic_order == digitalPinToPort( Devspec[ devspec_index ].Dpin ) )//does this port belong to this pin
            { //OF COURSE IT WILL MATCH.  TODO:  DELETE THIS CHECK or keep as only of these two checks
/*
                Serial.print( F( ", saving " ) );
                Serial.print( ( *&ISR_WITH_DHT_port_pinmask_stack_array[ number_of_elements_in_ISR_part_of_port_pinmask_stack  + str_index ] |= digitalPinToBitMask( pin ) ), BIN );
                Serial.print( F( " and " ) );
                Serial.print( ( *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 2 )  + str_index ] |= digitalPinToBitMask( pin ) ), BIN );
 */
                //physical pin:
/*                if ( ( bool ) ( Portspec[ str_index ].mask_of_real_pins_this_port & digitalPinToBitMask( pinxref_position ) ) )
                { 
                    for( u8 k = 0; k < pinxref_position; k++ )
                        if ( Portspec[ Devspec[ k ].portspec_index_for_pin ].this_port_main_reg != portOutputRegister( digitalPinToPort( pinxref_position ) ) || Devspec[ k ].mask_in_port != digitalPinToBitMask( pinxref_position ) )
                            continue;
                        else
                            Devspec[ pinxref_position ].duplicate_of_pin = Devspec[ k ].Dpin;
                }
                else 
                { 
                    Portspec[ str_index ].mask_of_real_pins_this_port |= digitalPinToBitMask( pinxref_position );
                    Devspec[ pinxref_position ].duplicate_of_pin = ( u8 ) -1;
                }
*/
/*  Order:
/*
ISRXREF* Isrxref;
PINXREF* Pinxref;
PORTXREF* Portxref;
DEVXREF* Devxref;
ISRSPEC* Isrspec;
PORTSPEC* Portspec;
PINSPEC* Pinspec;
DEVSPEC* Devspec;
*/
//The following won't work, they were from the unsparsed era
//                Devspec[ pin ].Dpin = pin;
//                Devspec[ pin ].portspec_index_for_pin =  str_index;
//                Devspec[ pin ].mask_in_port = digitalPinToBitMask( pin );
                if( !( pin_in_protected_arrays( Devspec[ devspec_index ].Dpin ) || Devspec[ devspec_index ].Dpin == LED_BUILTIN ) ) //testing-eligible:
                { 
                    Portspec[ str_index ].mask_of_safe_pins_to_detect_on |= digitalPinToBitMask( Devspec[ devspec_index ].Dpin );
                }
/*              mask_of_unsafe_pins_even_to_make_low_Z
                Serial.print( F( ", saved? " ) );
                Serial.print( *&ISR_WITH_DHT_port_pinmask_stack_array[ number_of_elements_in_ISR_part_of_port_pinmask_stack + str_index ], BIN );
                Serial.print( F( " and " ) );
                Serial.print( *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 2 ) + str_index ], BIN );
*/
            }
        }
    }
    delay( 5 );//to allow devices to settle down before making their pins into outputs HIGH
    
    for ( u8 port_placement = 0; port_placement < ( ( unsigned long )Devspec - ( unsigned long )Portspec )/ sizeof( PORTSPEC ); port_placement++ )
    {
        *( Portspec[ port_placement ].this_port_mode_reg ) = Portspec[ port_placement ].mask_of_safe_pins_to_detect_on;//Make safe and eligible pins into outputs
        *( Portspec[ port_placement ].this_port_main_reg ) = Portspec[ port_placement ].mask_of_safe_pins_to_detect_on; // set output pins HIGH for 2 seconds that are still eligible
    }
    delay( 2000 );//some types of device needed their line high like this for their wait period, so we do it for entire system
    unsigned long time_this_port_tested_millis[ ( ( unsigned long )Devspec - ( unsigned long )Portspec )/ sizeof( PORTSPEC ) ];
    for ( u8 port_placement = 0; port_placement < ( ( unsigned long )Devspec - ( unsigned long )Portspec )/ sizeof( PORTSPEC ); port_placement++ )
    {//find dht devices each port, number of ports = ( ( unsigned long )Pinspec - ( unsigned long )Portspec )/ sizeof( PORTSPEC )
           
    /* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
*/
    /* */
//        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        eligible_devices_this_port = Portspec[ port_placement ].mask_of_safe_pins_to_detect_on;
        eligible_devices_this_port = find_all_dhts_this_port( Portspec[ port_placement ].this_port_index_from_core_lookup_alphabetic_order, eligible_devices_this_port );//determikne if we really should send port_placement 

//        Serial.print( Portspec[ port_placement ].mask_of_safe_pins_to_detect_on, BIN ); 

        if ( ( bool ) ( eligible_devices_this_port ) )
        { //DO NOT SPARSE THE ARRAY YET BECAUSE THESE PORT ENTRIES NEED TO EXIST FOR THE ISR DISCOVERY PROCESS LATER.  IF THEY DON'T EXIST THEN, WE WOULD NEED MORE ELABORATE CODING THERE TO ACCOMODATE NON-EXISTENT PORT ENTRIES
            Portspec[ port_placement ].mask_of_DHT_devices_this_port = eligible_devices_this_port;//keep this out of the previous function called so that function can be used prior to this var existing
            time_this_port_tested_millis[ port_placement ] = millis();//don't yet know the particular rest period each device needs, but we are working towards guaranteeing them their rest after this exhausting acquisition
            //The first device this port will have Devspec index of....what?  Use port_placement and 
/*
//MUST ABSOLUTELY fill out all array alements referring to the Devprot array now: Dev_spec and Pin_spec
//First, assume enough Devspec space to handle was ensured before calling this function
//1.  find next empty element address in array by searching for 
//2.  fill in elements 
            Serial.print( F( "This port had DHT: PORT" ) );//port entry will only exist for ports having non-protected pin[ s ].  OK?
            Serial.print( ( char )( Portspec[ port_placement ].this_port_index_from_core_lookup_alphabetic_order + 64 ) );
            Serial.print( F( ", DHT-found: " ) );
            Serial.print( Portspec[ port_placement ].mask_of_DHT_devices_this_port, BIN ); 
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
 */
        }
/*
        Serial.flush();
        Serial.end();
    /*  */
    }
//                any_eligible_devices_here = B100101;  
//                any_eligible_devices_here = find_all_dhts_this_port( digitalPinToPort( pin ) );  
/* */
//                if ( ( bool ) any_eligible_devices_here ) 

/* */
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
/* */
/* 
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "Now reading the array at end of build_from_nothing()" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    for( u8 tmp_indx = 0; tmp_indx < sizeof PORTSPEC / sizeof *PORTSPEC; tmp_indx++ )
    { 
        Serial.print( F( "PORT" ) );
        Serial.print( ( char )( Portspec[ tmp_indx ].this_port_index_from_core_lookup_alphabetic_order + 64 ) ); 
        Serial.print( F( " ( at index " ) );
        Serial.print( tmp_indx );
        Serial.print( F( " ), physical pins: " ) );
        Serial.print( Portspec[ tmp_indx ].mask_of_real_pins_this_port, BIN ); 
        Serial.print( F( ", testing-eligible: " ) );
        Serial.print( Portspec[ tmp_indx ].mask_of_safe_pins_to_detect_on, BIN ); 
        Serial.print( F( ", DHT-found: " ) );
        Serial.print( Portspec[ tmp_indx ].mask_of_DHT_devices_this_port, BIN ); 
        Serial.print( F( ", PCINTs: " ) );
        Serial.print( Portspec[ tmp_indx ].PCINT_pins_mask, BIN ); 
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    }
*/
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "DHT devices found on pins:" ) );//
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    u8 DHT_count = 0;




//This loop stopped working
//    for ( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ )
//    {
/*        
        Serial.print( F( "string_of_all_ports_that_are_populated = " ) );
        Serial.print( string_of_all_ports_that_are_populated );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "Portspec[ ( u8 )( strchr( string_of_all_ports_that_are_populated, ( char )( digitalPinToPort( pin ) + 64 ) ) - ports_string_in_heap_array ) ].mask_of_DHT_devices_this_port = " ) );
        Serial.print( Portspec[ ( u8 )( strchr( string_of_all_ports_that_are_populated, ( char )( digitalPinToPort( pin ) + 64 ) ) - ports_string_in_heap_array ) ].mask_of_DHT_devices_this_port );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "digitalPinToBitMask( pin ) = " ) );
        Serial.print( digitalPinToBitMask( pin ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
*/
//        Serial.print( Portspec[ ( u8 )( strchr( string_of_all_ports_that_are_populated, ( char )( digitalPinToPort( pin ) + 64 ) ) - ports_string_in_heap_array ) ].mask_of_DHT_devices_this_port & digitalPinToBitMask( pin ) ) ); 
        for( u8 devspec_index = 0; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
        {
            ++DHT_count;
/*
            if ( Pinspec[ pin ].duplicate_of_pin == ( u8 ) -1 )
            { 
                if ( ++DHT_count < 10 ) Serial.print( F( " " ) );
                Serial.print( DHT_count );
                Serial.print( F( " - " ) );
            }
            else
            { 
                Serial.print( F( "     " ) );
            }
*/
            Serial.print( F( "    D" ) );
            Serial.print( Devspec[ devspec_index ].Dpin );
            if ( Devspec[ devspec_index ].Dpin < 10 ) Serial.print( F( "  " ) );
            else Serial.print( F( " " ) );
            print_analog_if_exists( Devspec[ devspec_index ].Dpin );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 

//the following won't work as is, from the unsparsed era
/*
            if ( Devspec[ pin ].duplicate_of_pin != ( u8 ) -1 )
            { 
                Serial.print( F( " duplicate of D" ) );
                Serial.print( Devspec[ pin ].duplicate_of_pin );
                if ( Devspec[ pin ].duplicate_of_pin < 10 ) Serial.print( F( "  " ) );
                else Serial.print( F( " " ) );
                print_analog_if_exists( Devspec[ pin ].duplicate_of_pin );
            }
*/
        }
//        if( ( pinxref_position == Elements_in->PINSPEC - 1 ) || ( !Pinxref->PIN_xref[ pinxref_position + 1 ] ) ) break;

//    }
//    for( u8 devspec_index = 0; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
//    {
//        Serial.print( F( "   D" ) );
//        Serial.print( Devspec[ devspec_index ].Dpin );
//        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
//    }
/*
    Serial.print( F( "The two counts " ) );
    if( pre_array_devspec_count == DHT_count )
        Serial.print( F( "are equal" ) );
    else
    {
        Serial.print( F( "differ: pre_array_devspec_count = " ) );
        Serial.print( pre_array_devspec_count );
        Serial.print( F( ", DHT_count = " ) );
        Serial.print( DHT_count );
    }
*/
    //Use DHT_count to malloc for Devspec
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    if( !DHT_count )
        Serial.print( F( "No " ) );
    else
    {
        Serial.print( F( "Total of " ) );
        Serial.print( DHT_count );
    }
    Serial.print( F( " DHT devices are connected" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );

    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.flush();
    Serial.end();
/*  */
//        while ( digitalPinToPortRegister( pinToOutputPort( pin ) ) != port_indexes_ddrmasks_and_pinlevels[ i ][ 0 ][ 0 ] && i < ( sizeof( port_indexes_ddrmasks_and_pinlevels ) )/3 ) i++;
//        ports_with_DHTs_indexes_ddrmasks_and_pinlevels[ from_pins_TOTAL_PORTS++ ][ 0 ][ 0 ];
}

void print_analog_if_exists( u8 pin )
{ 
    bool this_pin_is_also_analog = false; //will this line suffice?
    this_pin_is_also_analog = false;
#ifdef PIN_A0
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A0 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A0 ) ) //Reason we can't just compare pin to definition is because that will miss several real pins considering that analog pins map to duplicate virtual ones if available instead of real ones
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A0) " ) );
        }
#endif
#ifdef PIN_A1
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A1 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A1 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A1) " ) );
        }
#endif
#ifdef PIN_A2
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A2 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A2 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A2) " ) );
        }
#endif
#ifdef PIN_A3
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A3 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A3 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A3) " ) );
        }
#endif
#ifdef PIN_A4
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A4 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A4 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A4) " ) );
        }
#endif
#ifdef PIN_A5
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A5 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A5 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A5) " ) );
        }
#endif
#ifdef PIN_A6
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A6 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A6 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A6) " ) );
        }
#endif
#ifdef PIN_A7
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A7 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A7 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A7) " ) );
        }
#endif
#ifdef PIN_A8
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A8 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A8 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A8) " ) );
        }
#endif
#ifdef PIN_A9
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A9 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A9 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A9) " ) );
        }
#endif
#ifdef PIN_A10
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A10 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A10 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A10)" ) );
        }
#endif
#ifdef PIN_A11
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A11 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A11 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A11)" ) );
        }
#endif
#ifdef PIN_A12
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A12 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A12 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A12)" ) );
        }
#endif
#ifdef PIN_A13
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A13 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A13 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A13)" ) );
        }
#endif
#ifdef PIN_A14
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A14 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A14 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A14)" ) );
        }
#endif
#ifdef PIN_A15
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A15 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A15 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A15)" ) );
        }
#endif
//No boards known to have more than 16 analog pins, but that knowledge might be inaccurate or reality could change....
#ifdef PIN_A16
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A16 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A16 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A16)" ) );
        }
#endif
#ifdef PIN_A17
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A17 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A17 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A17)" ) );
        }
#endif
#ifdef PIN_A18
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A18 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A18 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A18)" ) );
        }
#endif
#ifdef PIN_A19
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A19 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A19 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A19)" ) );
        }
#endif
#ifdef PIN_A20
        if( digitalPinToPort( pin ) == digitalPinToPort( PIN_A20 ) && digitalPinToBitMask( pin ) == digitalPinToBitMask( PIN_A20 ) )
        { 
            this_pin_is_also_analog = true;
            Serial.print( F( "(A20)" ) );
        }
#endif
        if( !this_pin_is_also_analog ) Serial.print( F( "     " ) );
 }

void mem_frag_alert()
{ 
/* */
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( F( "Alert: Possible memory fragmentation as evidenced by internal acquisition of a memory block in a higher address of heap memory while changing the size of an internal array." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Memory fragmentation is NOT a fault condition - it is simply non-ideal due to its long term effects, and the developers of this product have gone to lengths to prevent it from happening on their account.  Accruing memory fragmentation long term usually eventually leads to unpredictable/degraded/unstable/locked operation.  Advisable action if this is a mission-critical application and this message appears periodically: reboot this device at your very next opportunity" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.flush();
    Serial.end();
/* */

 }

void mem_defrag_alert()
{ 
/* */
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( F( "Improved memory efficiency just occurred as evidenced by internal acquisition of a memory block in a lower address of heap memory while changing the size of an internal array.  Fragmentation has decreased." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.flush();
    Serial.end();
/* */

 }

/*
 * 
        char portchar = ( char ) ( digitalPinToPort( pin ) + 64 );                                                      //Compute the alpha of the port
        if ( strchr( ports_string_in_heap_array, portchar ) != NULL )                                                      //if it is found in the array of supporting ports found in pinset
        { //OF COURSE IT WILL BE FOUND.  TODO:  DELETE THIS CHECK


        
            u8 str_index = ( u8 ) ( strchr( ports_string_in_heap_array, portchar ) - ports_string_in_heap_array );//

            
            if( Portspec[ str_index ].this_port_index_from_core_lookup_alphabetic_order == digitalPinToPort( pin ) )


Portspec[ ( u8 ) ( strchr( ports_string_in_heap_array, ( char ) ( digitalPinToPort( pin ) + 64 ) ) - ports_string_in_heap_array ) ].

 * 
 * 
 */

byte mask_protected_pins_this_port( u8 _port )
{ 
//  mask_of_unsafe_pins_even_to_toggle
  return B0;
 }

bool pin_NOT_safe_even_to_make_low_Z_during_testing( u8 pin )//return the mask for entire port of devices in same situation?
{ 
    for ( u8 f = 0; f < sizeof( pins_NOT_safe_even_to_make_low_Z_during_testing ); f++ )
    { 
        if ( pin == pins_NOT_safe_even_to_make_low_Z_during_testing[ f ] )
        { 
            return ( true );//Portspecmask_of_safe_pins_to_detect_on
        }
    }
 }

bool pin_in_protected_arrays( u8 pin )
{ 
    for ( u8 f = 0; f < sizeof( pins_NOT_safe_even_to_make_low_Z_during_testing ); f++ )
    { 
        if ( pin == pins_NOT_safe_even_to_make_low_Z_during_testing[ f ] )
        { 
            return ( true );
        }
    }
    for ( u8 f = 0; f < sizeof( pins_NOT_safe_to_toggle_during_testing ); f++ )
    { 
        if ( pin == pins_NOT_safe_to_toggle_during_testing[ f ] )
        { 
            return ( true );
        }
    }
    return ( false );
 }

byte find_all_dhts_this_port( u8 port_index, u8 eligible_devices_this_port )                                           //Would be good also to discover each device's type but that would mean returning an array, an identifier byte for each port bit: 8 bytes return value?
{
// size of ISR_WITH_DHT_port_pinmask_stack_array at entry of this function: 
//number_of_ports_found
/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "PORT" ) );
        Serial.print( ( char )( port_index+64 ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */

    byte* port_ = ( byte* )portOutputRegister( port_index );
    byte* pinreg_ = ( byte* )portInputRegister( port_index );
    byte* portreg_ = ( byte* )portModeRegister( port_index );
//    byte valid_pins_this_port;
    u8 index = 0; //This here so later we can make it part of the array, if necessary
    unsigned long mid_bit_start_bit_low_checktime = 35;
    unsigned long mid_bit_start_bit_high_checktime = 120;
    unsigned long turnover_reference_time;
/*
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            Serial.print( F( "PORT" ) );
            Serial.print( ( char )( port_index+64 ) );
            Serial.print( F( " Starting to look through " ) );
            Serial.print( number_of_elements_in_ISR_part_of_port_pinmask_stack );
            Serial.print( F( " ports " ) );
            for ( u8 tmp_index = 0; tmp_index < number_of_elements_in_ISR_part_of_port_pinmask_stack; tmp_index++ )
                Serial.print( ( char )( ports_string_in_heap_array[ tmp_index ] ) );
//                Serial.print( ( char )( ports_string_in_heap_array[ tmp_index ] + 64 ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
*/
/* */ /* 
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            Serial.flush();
            Serial.end();
 */
/*
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "PORT" ) );
        Serial.print( ( char )( port_index+64 ) );
        Serial.print( F( " before *&ISR_WITH_DHT_port_pinmask_stack_array[ number_of_ports_found + index ] = " ) );
        Serial.print( *&ISR_WITH_DHT_port_pinmask_stack_array[ number_of_ports_found + index ],BIN );
        Serial.print( F( " and *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_ports_found * 2 ) + index ] = " ) );
        Serial.print( *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_ports_found * 2 ) + index ],BIN );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            Serial.flush();
            Serial.end();
*/
/* */
//        Serial.print( ( char )( *&ISR_WITH_DHT_port_pinmask_stack_array[ index ] ) );
//            valid_pins_this_port = Portspec[ index ].mask_of_real_pins_this_port;
//            if ( !( bool ) valid_pins_this_port ) return ( 0 );
//            eligible_devices_this_port =  Portspec[ port_placement ].mask_of_safe_pins_to_detect_on & Portspec[ port_placement ].mask_of_real_pins_this_port;
/* 
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
*/ /*
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            Serial.print( F( "PORT" ) );
            Serial.print( ( char )( port_index+64 ) );
            Serial.print( F( ", eligible_devices_this_port from array = " ) );
            Serial.print( eligible_devices_this_port,BIN );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            Serial.print( F( ", physical pins = " ) );
//            Serial.print( F( ", *&ISR_WITH_DHT_port_pinmask_stack_array[ number_of_elements_in_ISR_part_of_port_pinmask_stack + index ] = " ) );
            Serial.print( *&ISR_WITH_DHT_port_pinmask_stack_array[ number_of_elements_in_ISR_part_of_port_pinmask_stack + index ],BIN );
            Serial.print( F( ", testing-eligible pins = " ) );
//            Serial.print( F( ", *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 2 ) + index ] = " ) );
            Serial.print( *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 2 ) + index ],BIN );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            Serial.print( F( ", DHT-found pins = " ) );
//            Serial.print( F( ", *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 3 ) + index ] = " ) );
            Serial.print( *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 3 ) + index ],BIN );

//THIS DOES NOT START WITH A 0 BECAUSE THIS FUNCTION GETS CALLED AFTER THE FIRST INTERRUPT ON THIS PORT IS ALREADY CAUGHT
            Serial.print( F( ", PCINT pins = " ) );
//            Serial.print( F( ", *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 4 ) + index ] = " ) );
            Serial.print( *&ISR_WITH_DHT_port_pinmask_stack_array[ ( number_of_elements_in_ISR_part_of_port_pinmask_stack * 4 ) + index ],BIN );

*/
    
/* 
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            Serial.flush();
            Serial.end();
 */
//    if ( index == number_of_elements_in_ISR_part_of_port_pinmask_stack )// array got shrunk and trimmed out this port, so just make sure this gets put into DHT_without_ISR_port_pinmask_stack_array
//    previous_DHT_without_ISR_port_pinmask_stack_array
/* */
/* */





//For debugging only
//  eligible_devices_this_port = B10;  this bit on PORTD causes extraneous character to go out on serial line
//eligible_devices_this_port = B11111101;
//        if ( ( char )( port_index + 64 ) == 'D' ) eligible_devices_this_port = 0;
/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "PORT" ) );
        Serial.print( ( char )( port_index+64 ) );
        Serial.print( F( ", valid_pins_this_port = " ) );
        Serial.print( valid_pins_this_port, BIN );    //move these lines around in testing.  It does cause timing delays that will cause the remainder of timing-critical code to fail
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        delay( 100 );
        Serial.flush();
        Serial.end();
*/








/* 
//if ( ( char )( port_index + 64 ) == 'D' ) Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( ( char )( port_index + 64 ) );
//Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//Serial.print( F( ", where is found >" ) );
//Serial.print( ports_string_[ this_port_index_in_order_of_discovery_traversing_pins * 2 ] );
//Serial.print( F( " that translates to >" ) );
//Serial.println( ( u8 )( ports_string_[ this_port_index_in_order_of_discovery_traversing_pins * 2 ] - 64 ) );

*/
/*
//        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( ": pin presences: " ) );
        Serial.print( eligible_devices_this_port, BIN );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//          delay( 100 );
        Serial.flush();
        Serial.end();
*/
//    eligible_devices_this_port = B10101010;
/*
    for ( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ ) //See if pin is serviced by an ISR( PCINTn_vect ).  Build port masks and PCMSK, PCICR masks.  Skip unsafe pins
    { 
        for ( u8 f = 0; f < sizeof( pins_NOT_safe_even_to_make_low_Z_during_testing ); f++ )
        { 
            if ( pin == pins_NOT_safe_even_to_make_low_Z_during_testing[ f ] && digitalPinToPort( pin ) == port_index )
            { 
                eligible_devices_this_port &= ~digitalPinToBitMask( pin );
            }
        }
    
    
        for ( u8 f = 0; f < sizeof( pins_NOT_safe_to_toggle_during_testing ); f++ )
        { 
            if ( pin == pins_NOT_safe_to_toggle_during_testing[ f ] && digitalPinToPort( pin ) == port_index )
            { 
                eligible_devices_this_port &= ~digitalPinToBitMask( pin );
            }
        }
    }
    */
//    byte* safe_to_test_for_eligible_devices_pins_this_port = &safe_to_test_for_eligible_devices_pins_mask_this_port[ this_port_index_in_order_of_discovery_traversing_pins ];
//    byte* eligible_devices_this_port = &eligible_DHT_devices_mask_this_port[ this_port_index_in_order_of_discovery_traversing_pins ];
//    if ( eligible_devices_this_port != 0 ) numOfPortsWithAnyDHTDevice--;  
    /*
    digitalPinToPort( pin ) only gives us an index to the port in the port array
    
    THE PORT ADDRESSES ARE
    portOutputRegister( digitalPinToPort( pin ) ) gives address of PORTx port ( or primary register, if you'd like )
    
    THE DDR REGISTER ADDRESSES ARE
    portModeRegister( digitalPinToPort( pin ) ) gives a DDRx register address
    
    THE PINx REGISTER ADDRESSES ARE
    portInputRegister( digitalPinToPort( pin ) ) gives a PINx register address
    
    digitalPinToBitMask( pin ) gives index of the mask on the port for the pin.  Cast it as BIN for real mask
    
     */
/*
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
*/
/*
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "Entered findalldhts for PORT" ) );
Serial.print( ( char )( port_index + 64 ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( ", where is found >" ) );
Serial.print( ports_string_[ this_port_index_in_order_of_discovery_traversing_pins * 2 ] );
Serial.print( F( " that translates to >" ) );
Serial.println( ( u8 )( ports_string_[ this_port_index_in_order_of_discovery_traversing_pins * 2 ] - 64 ) );
 */
/*            delay( 100 );
            Serial.flush();
            Serial.end();
*/
//    delay( 500 );
/* 
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "PORT" ) );
    Serial.print( ( char )( port_index+64 ) );
    Serial.print( F( ", starting eligible_devices_this_port = " ) );
    Serial.print( eligible_devices_this_port,BIN );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.flush();
    Serial.end();
 */
    u8 i;
    //if any device on this port is in its resting period, we need to wait before continuing with this routine!!!
//    if ( port_resttime[ port_index ] != 0 ) //This wait time is only port-specific dealing with non-ISR code, not device-specific ISR-related.  The ISR code needs to check this variable before starting device communications on this port
//    { 
//        while ( ( millis() - port_resttime[ port_index ] < 2010 ) & ( millis() > port_resttime[ port_index ] ) ); //This time window needs to be adjustable upward depending on the device's needs
//    }
    //save the state of this port, pin directions and pullup or not and state ( if output ), restore when leaving
    byte startstate_port = *port_;// & valid_pins_this_port; //The AND boolean operation is done to mask out bits that don't have pins for them
    byte startstate_portreg = *portreg_;// & valid_pins_this_port;
    byte startstate_pinreg = *pinreg_;// & valid_pins_this_port;
//if ( ( char )( port_index + 64 ) == 'D' )  goto this_port_done_for_dht_detection;
    //Find the clock rate: 
/*
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
*/
/*    Serial.print( F( "F_CPU = " ) );
    Serial.print( F_CPU,DEC );
    Serial.print( F( " ( Hz ), " ) );
    Serial.print( F_CPU );
    Serial.println( F( " ( Hz ) in native" ) );
    Serial.print( F( "1,000,000/F_CPU = " ) );
    Serial.print( ( 1000000.0/( float )F_CPU ),4 ); //after 4, all zeros
    Serial.println( F( " ( usec )" ) );
*/
/*            delay( 100 );
            Serial.flush();
            Serial.end();
*/
//DO we need two different masks for protecting pins?
//    eligible_devices_this_port &= *safe_to_test_for_eligible_devices_pins_this_port;

/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( " eligible_devices_this_port = " ) );
        Serial.print( eligible_devices_this_port, BIN );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */
    if ( !( bool ) eligible_devices_this_port ) goto this_port_done_for_dht_detection;//This should never happen here in real life because such a port wouldn't even get in the ISR_WITH_DHT_port_pinmask_stack_array array
    
    *portreg_ |= eligible_devices_this_port; //Make safe and eligible pins into outputs
//    *port_ |= eligible_devices_this_port; // set output pins HIGH for 2 seconds that are still eligible
//    delay( 2000 ); //place these two lines in a loop prior to this function to catch all ports with a single 2 second delay
    *port_ &= ~( eligible_devices_this_port ); // set output pins LOW that are still eligible
/* The following Serial object is needed to stay open here like this in order to prevent extraneous characters from going out on the serial line */
/*        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
*/
        delay( 19 );

    // Here begins timing-critical code in this routine: From here to the end of timing-critical code, any changes you make to this established code result in untested timing

    *port_ = eligible_devices_this_port; // set eligible pins HIGH, still outputs
    eligible_devices_this_port &= *pinreg_;  //disqualify eligible lines that aren't high. 
    if ( !( bool ) eligible_devices_this_port ) goto this_port_done_for_dht_detection;
    *portreg_ &= ~( eligible_devices_this_port ); // change only the eligible pins to 0 ( inputs ), keep ineligible pins as they are//This line begins the problems
    *port_ |= eligible_devices_this_port; //making them have pullups so no resistors are needed to the DHT i/o pin
    turnover_reference_time = micros();
    eligible_devices_this_port &= *pinreg_;  //disqualify eligible lines that aren't high. All pins that stayed high with pullups are eligible candidates
    if ( !( bool ) eligible_devices_this_port ) goto this_port_done_for_dht_detection;
//    while ( micros() - turnover_reference_time < mid_bit_start_bit_low_checktime ); // device responds in reality by 8 u-sec going low ( data sheets say 20-40 uSec ).  This adds plenty of margin.
    while ( micros() - turnover_reference_time < 34 ); // device responds in reality by 8 u-sec going low ( data sheets say 20-40 uSec ).  This adds plenty of margin.
//    while ( micros() - turnover_reference_time < 40 ); // device responds in reality by 8 u-sec going low ( data sheets say 20-40 uSec ).  This adds plenty of margin.
    eligible_devices_this_port &= ~( *pinreg_ );  //disqualify eligible lines that aren't low
/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( " eligible_devices_this_port = " ) );
        Serial.print( eligible_devices_this_port, BIN );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */
    if ( !( bool ) eligible_devices_this_port ) goto this_port_done_for_dht_detection;
//    while ( micros() - turnover_reference_time < mid_bit_start_bit_high_checktime ); // device responds by 128 u-sec with good margin in time
    while ( micros() - turnover_reference_time < 118 ); // device responds by 128 u-sec with good margin in time
    eligible_devices_this_port &= *pinreg_; //HIGHs on eligible pins maintain eligibility


// Endpoint of timing-critical code
/* 
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "eligible_devices_this_port = " ) );
//Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( eligible_devices_this_port, BIN );Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//delay( 100 );*/
//    discover_port_order_and_populate_pin_port_mask();
this_port_done_for_dht_detection:
/* */
//    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
//    Serial.setTimeout( 10 ); //
//    while ( !Serial ) { 
//      ; // wait for serial port to connect. Needed for Leonardo's native USB
//    }
/* 
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "PORT" ) );
    Serial.print( ( char )( port_index+64 ) );
//    Serial.print( F( ": *portreg_ = " ) );
//    Serial.print( *portreg_,BIN );
*/
    *portreg_ = startstate_portreg | eligible_devices_this_port;  //Restore ports to inputs or outputs except that where devices were found become outputs regardless
/*
//    Serial.print( F( ", restored to " ) );
//    Serial.print( *portreg_,BIN );
//    Serial.print( F( ": *port_ = " ) );
//    Serial.print( *port_,BIN );
//        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
//        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( ", ending eligible_devices_this_port = " ) );
    Serial.print( eligible_devices_this_port,BIN );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
 */
//
// Is this really a good idea here instead of after finding ISR data?
    *port_ = startstate_port & ~( eligible_devices_this_port ); // set output pins LOW that are still eligible CAUSE EXTRANEOUS SYMBOLS TO APPEAR ON SERIAL LINE

/* 
//    Serial.print( F( ", restored to " ) );
//    Serial.print( *port_,BIN );
//    Serial.print( F( ", startstate_port = " ) );
//    Serial.print( startstate_port,BIN );
//    Serial.print( F( "...eligible_devices_this_port = " ) );
//    Serial.print( eligible_devices_this_port,BIN );
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.flush();
    Serial.end();
*/
//device_protocols; device_type_on_bit[ 
    if ( ( bool ) eligible_devices_this_port ) numOfPortsWithAnyDHTDevice++;
/*
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
        Serial.println( F( "Clock rate OK, so DHT devices have been detected." ) );
        Serial.flush();
        Serial.end();
*/


//TODO:  Fix the following
//    digitalWrite( LED_BUILTIN,0 );    //This line somehow does something not good as revealed by any serial print output that follows
//TODO:  fix the previous line                                    //This was high to disable relays and whatnots so they don't get driven during the device detection process.  The circuitry for that is the end-user's responsibility


//index needs to be changed to right one using a search
    return ( eligible_devices_this_port );
//    on return of something, start the wait timer for all devices this entire port, 1 second for DHT 11, 2 seconds for all others
}
/*
 * 
    free ( previous_ISR_WITH_DHT_port_pinmask_stack_array ); //Just as a matter of good practice, doesn't really do anything here
    Elements_in = ( ELEMENTS_IN* )malloc( \
        sizeof( ELEMENTS_IN ) + \
        ( sizeof( ISRSPEC ) * number_of_ISRs ) + \
        ( sizeof( PORTSPEC ) * number_of_ports_found ) + \
        ( sizeof( DHT_NO_ISR ) * NUM_DIGITAL_PINS ) + \
        ( sizeof( PINSPEC ) * NUM_DIGITAL_PINS ) + \
         0 + 0 \
        )
*/

u8 Portspec_ready_port_index_adjust ( u8 portindex )
{
    return( strchr( ports_string_in_heap_array, ( char )( portindex + 64 ) ) - ports_string_in_heap_array );
}

void delay_if_device_triggered( u8 pin )
{
/*
//test lines:
    pinMode( PIN_A0, OUTPUT );                                            
    digitalWrite( PIN_A0, LOW );
    delay( 26 );//on leonardo, test at 27 or more
    digitalWrite( PIN_A0, HIGH );
    pinMode( PIN_A0, INPUT );
//done with test lines
*/
    byte* port_ = ( byte* )portOutputRegister( digitalPinToPort( pin ) );
    byte* pinreg_ = ( byte* )portInputRegister( digitalPinToPort( pin ) );
    byte* portreg_ = ( byte* )portModeRegister( digitalPinToPort( pin ) );
    // timing-critical code: any changes you make to this established code result in untested timing
    *port_ |= digitalPinToBitMask( pin ); //making pin have pullup so no resistors are needed to the DHT i/o pin
    long turnover_reference_time = micros();
    if( !( bool )( *pinreg_ & digitalPinToBitMask( pin ) ) ) return;  //disqualify eligible lines that aren't high. All pins that stayed high with pullups are eligible candidates
    while ( micros() - turnover_reference_time < 190 )
    { 
        if( !( bool )( *pinreg_ & digitalPinToBitMask( pin ) ) ) 
        { 
/*
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
            Serial.print( F( "delay_if_device_triggered( () found a device on this pin.  Delaying 2 seconds" ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            Serial.flush();
            Serial.end();
*/
            delay( 2000 );//Allow time for any DHT device to settle down
        }
    }
}

unsigned short resistor_between_LED_BUILTIN_and_PIN_A0() //default purpose for this is to signify type of line-ends to send with serial communications: MS Windows-style ( resistor connects the pins ) will include a CR with the LF, while non-windows ( no resistor ) only has the LF
{ //Need to detect any device connected to this pin and add delay for it/them
#ifndef PIN_A0
        return ( ( unsigned short ) -1 );
#else
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, HIGH );
    delay( 100 );
    pinMode( PIN_A0, OUTPUT );
//                   BEWARE:
//The following line or lines cause[ s ] any DHT device on this pin to go out of detect-ability for a while 
    digitalWrite( PIN_A0, LOW );                                                //A test here for 1 MOhm resistor between pins A0 and LED_BUILTIN while led is high:  make A0 an output, take it low, make an analog input and read it for a high
    pinMode( PIN_A0, INPUT );
    delay( 1 );                                                               //allow settling time, but not long enough to trigger a DHT device, thankfully
    if ( analogRead( PIN_A0 ) > 100 )                                           // in breadboard testing, this level needed to be at least 879 for reliable detection of a high if no delay allowance for settling time
    { 
        digitalWrite( LED_BUILTIN, LOW );                                        //A high was used to disable relays and whatnots so they don't get driven during the dht device detection process.  The circuitry for that is the end-user's responsibility
        pinMode( PIN_A0, OUTPUT );                                            
        digitalWrite( PIN_A0, HIGH );                                             //A test here for 1 MOhm resistor between pins A0 and LED_BUILTIN while led is high:  make A0 an output, take it low, make an analog input and read it for a high
        pinMode( PIN_A0, INPUT );
        unsigned short returnvalue = analogRead( PIN_A0 ) + 1;
        delay_if_device_triggered( PIN_A0 );

        pinMode( PIN_A0, OUTPUT );                                           
        digitalWrite( PIN_A0, HIGH );                                             //A test here for 1 MOhm resistor between pins A0 and LED_BUILTIN while led is high:  make A0 an output, take it low, make an analog input and read it for a high
//
        return ( returnvalue );      //Will be 1 to 101 if connected to LED_BUILTIN through resistor
/*
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
            Serial.print( F( "A resistor might not be connected between pins LED_BUILTIN and PIN_A0" ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            delay( 100 );
            Serial.flush();
            Serial.end();
*/
    }
    else
    { 
        digitalWrite( LED_BUILTIN, LOW );                                        //A high was used to disable relays and whatnots so they don't get driven during the dht device detection process.  The circuitry for that is the end-user's responsibility
        pinMode( PIN_A0, OUTPUT );                                             
        digitalWrite( PIN_A0, HIGH );                                             //A test here for 1 MOhm resistor between pins A0 and LED_BUILTIN while led is high:  make A0 an output, take it low, make an analog input and read it for a high
        //delay( 2000 );//Allow time for any DHT device to settle down
        return ( 0 );
    }
/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "A resistor is definitely not connected between pins LED_BUILTIN and PIN_A0" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        delay( 1000 );
        Serial.flush();
        Serial.end();
*/
#endif
 }

unsigned short resistor_between_LED_BUILTIN_and_PIN_A1()//default purpose for this is to signify the verbosity level end-user wants, no resistor = max verbosity, user can utilize various resistor configurations to adjust verbosity
{ //OBSOLETE:The advantage of a sparsed memory model is that memory use is minimized by this process so that maximum memory remains for other, user-added functionality.  The disadvantage is that if the end-user needs to add DHT
//  devices without rebooting the uController, memory fragmentation of the heap is likely; thus, rebooting the uController might be advisable for high-reliable operation with that memory model when DHT devices are added, subtracted or changed around.
//  The default ( no resistor ) is for maximum capability of this software product ( no memory fragmentation when devices are changed without rebooting ) which leaves a little less memory for end-user-added functionality.
//Need to detect any device connected to this pin and add delay for it/them
#ifndef PIN_A1
        return ( ( unsigned short ) -1 );
#else
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, HIGH );
    delay( 100 );
    pinMode( PIN_A1, OUTPUT );                                              
    digitalWrite( PIN_A1, LOW );                                                //A test here for 1 MOhm resistor between pins A1 and LED_BUILTIN while led is high:  make A1 an output, take it low, make an analog input and read it for a high
    pinMode( PIN_A1, INPUT );
    delay( 1 );                                                               //allow settling time
    if ( analogRead( PIN_A1 ) > 100 )                                           // in breadboard testing, this level needed to be at least 879 for reliable detection of a high if no delay allowance for settling time
    { 
        digitalWrite( LED_BUILTIN, LOW );                                        //A high was used to disable relays and whatnots so they don't get driven during the dht device detection process.  The circuitry for that is the end-user's responsibility
        pinMode( PIN_A1, OUTPUT );                                         
        digitalWrite( PIN_A1, HIGH );                                             //A test here for 1 MOhm resistor between pins A1 and LED_BUILTIN while led is high:  make A1 an output, take it low, make an analog input and read it for a high
        pinMode( PIN_A1, INPUT );
        unsigned short returnvalue = analogRead( PIN_A1 ) + 1;
        delay_if_device_triggered( PIN_A1 );
        pinMode( PIN_A1, OUTPUT );                                          
        digitalWrite( PIN_A1, HIGH );                                             //A test here for 1 MOhm resistor between pins A1 and LED_BUILTIN while led is high:  make A1 an output, take it low, make an analog input and read it for a high
        return ( returnvalue );      //Will be 1 to 101 if connected to LED_BUILTIN through resistor
/*
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
            Serial.print( F( "A resistor might not be connected between pins LED_BUILTIN and PIN_A1" ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            delay( 100 );
            Serial.flush();
            Serial.end();
*/
    }
    else
    { 
        pinMode( PIN_A1, OUTPUT );                                             
        digitalWrite( PIN_A1, HIGH );                                             //A test here for 1 MOhm resistor between pins A1 and LED_BUILTIN while led is high:  make A1 an output, take it low, make an analog input and read it for a high
        //delay( 2000 );//Allow time for any DHT device to settle down
        digitalWrite( LED_BUILTIN, LOW );                                        //A high was used to disable relays and whatnots so they don't get driven during the dht device detection process.  The circuitry for that is the end-user's responsibility
        return ( 0 );
    }
/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "A resistor is definitely not connected between pins LED_BUILTIN and PIN_A1" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        delay( 1000 );
        Serial.flush();
        Serial.end();
*/
#endif
 }

unsigned short resistor_between_LED_BUILTIN_and_PIN_A2()//default purpose for this is for end-user to indicate the host system might provide bootup configuration information.  True means to ask host and wait a short while for response
{ //Need to detect any device connected to this pin and add delay for it/them
#ifndef PIN_A2
        return ( ( unsigned short ) -1 );
#else
    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, HIGH );
    delay( 100 );
    pinMode( PIN_A2, OUTPUT );                                               
    digitalWrite( PIN_A2, LOW );                                                //A test here for 1 MOhm resistor between pins A2 and LED_BUILTIN while led is high:  make A2 an output, take it low, make an analog input and read it for a high
    pinMode( PIN_A2, INPUT );
    delay( 1 );                                                               //allow settling time
    if ( analogRead( PIN_A2 ) > 100 )                                           // in breadboard testing, this level needed to be at least 879 for reliable detection of a high if no delay allowance for settling time
    { 
        digitalWrite( LED_BUILTIN, LOW );                                        //A high was used to disable relays and whatnots so they don't get driven during the dht device detection process.  The circuitry for that is the end-user's responsibility
        pinMode( PIN_A2, OUTPUT );                                             
        digitalWrite( PIN_A2, HIGH );                                             //A test here for 1 MOhm resistor between pins A2 and LED_BUILTIN while led is high:  make A2 an output, take it low, make an analog input and read it for a high
        pinMode( PIN_A2, INPUT );
        unsigned short returnvalue = analogRead( PIN_A2 ) + 1;
        delay_if_device_triggered( PIN_A2 );
        pinMode( PIN_A2, OUTPUT );                                             
        digitalWrite( PIN_A2, HIGH );                                             //A test here for 1 MOhm resistor between pins A2 and LED_BUILTIN while led is high:  make A2 an output, take it low, make an analog input and read it for a high
        return ( returnvalue );      //Will be 1 to 101 if connected to LED_BUILTIN through resistor
/*
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
            Serial.print( F( "A resistor might not be connected between pins LED_BUILTIN and PIN_A2" ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
            delay( 100 );
            Serial.flush();
            Serial.end();
*/
    }
    else
    { 
        pinMode( PIN_A2, OUTPUT );                                             
        digitalWrite( PIN_A2, HIGH );                                             //A test here for 1 MOhm resistor between pins A2 and LED_BUILTIN while led is high:  make A2 an output, take it low, make an analog input and read it for a high
        //delay( 2000 );//Allow time for any DHT device to settle down
        digitalWrite( LED_BUILTIN, LOW );                                        //A high was used to disable relays and whatnots so they don't get driven during the dht device detection process.  The circuitry for that is the end-user's responsibility
        return ( 0 );
    }
/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "A resistor is definitely not connected between pins LED_BUILTIN and PIN_A2" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        delay( 1000 );
        Serial.flush();
        Serial.end();
*/
#endif
 }

bool reset_ISR_findings_and_reprobe ( bool protect_protected_pins )
{ 
  volatile u8 PCINT_pins_by_PCMSK_and_ISR[ 2 ][ 8 ][ number_of_ISRs ];

// size of ISR_WITH_DHT_port_pinmask_stack_array at entry of this function the first time: 
//number_of_ports_found
//after that, this function can be called with a different value; namely, number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR

/*
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.print( F( "Entering reset_ISR_findings_and_reprobe" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.flush();
    Serial.end();
*/
    digitalWrite( LED_BUILTIN,1 ); //This is to be used by the end user to disable relays and whatnots so they don't get driven during the device detection process.  The circuitry for that is the end-user's responsibility/discretion
/* */
    u8 number_of_ports_that_responded_to_ISR_probing = 0;  //Needs to be calculated every time the ISR probing function is executed because pins forced by momentary low Z circuit faults could be overlooked as being serviced by ISRs
// and because sparsing will require it

//UNTESTED
//ensure free on entering this function for number_of_ports_that_responded_to_ISR_probing
/*
    bool just_getting_number_of_ports_with_devices;
    if ( number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR == 0 )
    { 
        just_getting_number_of_ports_with_devices = true;
    }
    else
    { 
        just_getting_number_of_ports_with_devices = false;
        free( ptr_to_portspecs_stack );                                                                        //unmalloc the memory from the previous probing
        ptr_to_portspecs_stack = ( port_specific* )malloc( sizeof( ptr_to_portspecs_stack ) * number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR );
        if ( ptr_to_portspecs_stack == NULL ) return ( false );
        if ( previous_ptr_to_portspecs_stack != NULL && previous_ptr_to_portspecs_stack != ptr_to_portspecs_stack )//  Entered state of possibility of memory fragmentation
        { 
            if ( previous_ptr_to_portspecs_stack < ptr_to_portspecs_stack ) mem_frag_alert();
            else mem_defrag_alert();
        }
        number_of_elements_in_ISR_part_of_port_pinmask_stack = number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR;
        previous_ptr_to_portspecs_stack = ptr_to_portspecs_stack;
        number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR = 0;
    }
*/
/* */
    
/*
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.print( F( "Entering reset_ISR_findings_and_reprobe" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.flush();
    Serial.end();
*/
    
    for ( u8 i = 0; i < 2; i++ )
    { 
        for ( u8 j = 0; j < 8; j++ )
        { 
            for ( u8 k = 0; k < number_of_ISRs; k++ )
            { 
                PCINT_pins_by_PCMSK_and_ISR[ i ][ j ][ k ] = ( u8 ) 0;
            }
        }
    }
    
    byte pmask;
    u8 port_indexes_ddrmasks_and_pinlevels[ number_of_ports_found ][ 1 ][ 1 ][ 1 ]; //Not yet ready, go through all pins first and get number of ports that way
//Get ready to trip interrupts and determine which ports ( and how many for array creation ) have any PCINT lines whatsoever
    byte* portaddr;                              //byte This just gets us an index value
    byte* ddraddr;            //u8 *
    byte* pinaddr;
    number_of_ports_that_responded_to_ISR_probing = 0;
//    u8 pin; //Left in, this allowed false 0 value in var later
    u8 i;
    u8 ports_string_size = number_of_ports_found + 1; //ARD_TOTAL_PORTS + 1;
//    if ( ARD_TOTAL_PORTS > 8 ) ports_string_size += ARD_TOTAL_PORTS - 8;
    char ports_with_ISRs_string[ ports_string_size ] { 0 }; //ARD_TOTAL_PORTS=11 make it 25, if=3 make it 6 ( ( ARD_TOTAL_PORTS * 2 )+( ARD_TOTAL_PORTS-8 if it is greater than 8 ) )
    char ISR_ports_with_DHTs_string[ ports_string_size ];
    char tmp_ports_under_test_string[ ports_string_size ] { 0 }; //ARD_TOTAL_PORTS=11 make it 25, if=3 make it 6 ( ( ARD_TOTAL_PORTS * 2 )+( ARD_TOTAL_PORTS-8 if it is greater than 8 ) )
//    ports_string_ = ports_string;
    char portchar = { ' ' };
    char ISRindexchar = { ' ' };
    char ISRs_used_[ 4 ] = { "   " };
//    char* ISRs_used_ = &ISRs_used_[ 0 ];

    PCICR = 0;

#ifdef PCMSK
    PCMSK = 0;
    isrspec_addr0 = &srspec[ 0 ];
    pin_change_reported_by_ISR = 0;
    Isrspec[ 0 ].mask_by_PCMSK_of_real_pins = 0;
    Isrspec[ 0 ].mask_by_PCMSK_of_valid_devices = 0;
    Isrspec[ 0 ].pcmsk = &PCMSK0;
    Isrspec[ 0 ].mask_by_port_of_current_device_being_actively_communicated_with_thisISR = 0;
    Isrspec[ 0 ].active_pin_ddr_port_reg_addr = 0;
    Isrspec[ 0 ].active_pin_output_port_reg_addr = 0;
    Isrspec[ 0 ].active_pin_pin_reg_addr = 0;
    Isrspec[ 0 ].mask_by_PCMSK_of_current_device_within_ISR = 0;
    Isrspec[ 0 ].index_in_PCMSK_of_current_device_within_ISR = 0;
    Isrspec[ 0 ].start_time_plus_max_acq_time_in_uSecs = 0;
    Isrspec[ 0 ].next_bit_coming_from_dht = 255;
    Isrspec[ 0 ].timestamps[ 0 ] = 0;
    Isrspec[ 0 ].interval = 255;
    Isrspec[ 0 ].offset = 0;
    Isrspec[ 0 ].val_tmp1 = ( unsigned short* )&Isrspec[ 0 ].sandbox_bytes[ 1 ];
    Isrspec[ 0 ].val_tmp2 = ( unsigned short* )&Isrspec[ 0 ].sandbox_bytes[ 3 ];
    Isrspec[ 0 ].millis_rest_length = Devprot[ 0 ].millis_rest_length;//make obsolete?
    for( u8 m = 0;m < sizeof( Isrspec[ 0 ].array_of_all_devspec_index_plus_1_this_ISR ) ;m++ )
        Isrspec[ 0 ].array_of_all_devspec_index_plus_1_this_ISR[ m ] = 0;
    for( u8 m = 0;m < sizeof( Isrspec[ 0 ].array_of_all_devprot_index_this_ISR ) ;m++ )
        Isrspec[ 0 ].array_of_all_devprot_index_this_ISR[ m ] = 0;
    #ifdef PCMSK0 //The purpose of this entry is for rationale only, never expected to materialize
        PCMSK0 = 0;
        isrspec_addr1 = &Isrspec[ 1 ];
        pin_change_reported_by_ISR0 = 0;
        Isrspec[ 1 ].mask_by_PCMSK_of_real_pins = 0;
        Isrspec[ 1 ].mask_by_PCMSK_of_valid_devices = 0;
        Isrspec[ 1 ].pcmsk = &PCMSK1;
        Isrspec[ 1 ].mask_by_port_of_current_device_being_actively_communicated_with_thisISR = 0;
        Isrspec[ 1 ].active_pin_ddr_port_reg_addr = 0;
        Isrspec[ 1 ].active_pin_output_port_reg_addr = 0;
        Isrspec[ 1 ].active_pin_pin_reg_addr = 0;
        Isrspec[ 1 ].mask_by_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 1 ].index_in_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 1 ].start_time_plus_max_acq_time_in_uSecs = 0;
        Isrspec[ 1 ].next_bit_coming_from_dht = 255;
        Isrspec[ 1 ].timestamps[ 0 ] = 0;
        Isrspec[ 1 ].interval = 255;
        Isrspec[ 1 ].offset = 0;
        Isrspec[ 1 ].val_tmp1 = ( unsigned short* )&Isrspec[ 1 ].sandbox_bytes[ 1 ];
        Isrspec[ 1 ].val_tmp2 = ( unsigned short* )&Isrspec[ 1 ].sandbox_bytes[ 3 ];
        Isrspec[ 1 ].millis_rest_length = Devprot[ 0 ].millis_rest_length;
        for( u8 m = 0;m < sizeof( Isrspec[ 1 ].array_of_all_devspec_index_plus_1_this_ISR ) ;m++ )
            Isrspec[ 1 ].array_of_all_devspec_index_plus_1_this_ISR[ m ] = 0;
        for( u8 m = 0;m < sizeof( Isrspec[ 1 ].array_of_all_devprot_index_this_ISR ) ;m++ )
            Isrspec[ 1 ].array_of_all_devprot_index_this_ISR[ m ] = 0;
    #endif
#else
    #ifdef PCMSK0
        PCMSK0 = 0;
        isrspec_addr0 = &Isrspec[ 0 ];
        Isrspec[ 0 ].mask_by_PCMSK_of_real_pins = 0;
        Isrspec[ 0 ].mask_by_PCMSK_of_valid_devices = 0;
        Isrspec[ 0 ].pcmsk = &PCMSK0;
        Isrspec[ 0 ].mask_by_port_of_current_device_being_actively_communicated_with_thisISR = 0;
        Isrspec[ 0 ].active_pin_ddr_port_reg_addr = 0;
        Isrspec[ 0 ].active_pin_output_port_reg_addr = 0;
        Isrspec[ 0 ].active_pin_pin_reg_addr = 0;
        Isrspec[ 0 ].mask_by_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 0 ].index_in_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 0 ].start_time_plus_max_acq_time_in_uSecs = 0;
        Isrspec[ 0 ].next_bit_coming_from_dht = 255;
        Isrspec[ 0 ].timestamps[ 0 ] = 0;
        Isrspec[ 0 ].interval = 255;
        Isrspec[ 0 ].offset = 0;
        Isrspec[ 0 ].val_tmp1 = ( unsigned short* )&Isrspec[ 0 ].sandbox_bytes[ 1 ];
        Isrspec[ 0 ].val_tmp2 = ( unsigned short* )&Isrspec[ 0 ].sandbox_bytes[ 3 ];
        Isrspec[ 0 ].millis_rest_length = Devprot[ 0 ].millis_rest_length;
        for( u8 m = 0;m < sizeof( Isrspec[ 0 ].array_of_all_devspec_index_plus_1_this_ISR ) ;m++ )
            Isrspec[ 0 ].array_of_all_devspec_index_plus_1_this_ISR[ m ] = 0;
        for( u8 m = 0;m < sizeof( Isrspec[ 0 ].array_of_all_devprot_index_this_ISR ) ;m++ )
            Isrspec[ 0 ].array_of_all_devprot_index_this_ISR[ m ] = 0;
    #endif
/* 
Isrspec[ 0 ].active_pin_ddr_port_reg_addr = 0;
Isrspec[ 0 ].active_pin_output_port_reg_addr = 0;
Isrspec[ 0 ].active_pin_pin_reg_addr = 0;
Isrspec[ 0 ].index_in_PCMSK_of_current_device_within_ISR = 0;
Isrspec[ 0 ].start_time_plus_max_acq_time_in_uSecs = 0;
Isrspec[ 0 ].next_bit_coming_from_dht = 0;
Isrspec[ 0 ].timestamps[ 0 ] = 0;
Isrspec[ 0 ].millis_rest_length = 0;
Isrspec[ 0 ].array_of_all_devspec_index_plus_1_this_ISR[ 0 ] = 0;
Isrspec[ 0 ].array_of_all_devprot_index_this_ISR[ 0 ] = 0;
 */
    #ifdef PCMSK1
        PCMSK1 = 0;
        isrspec_addr1 = &Isrspec[ 1 ];
        Isrspec[ 1 ].mask_by_PCMSK_of_real_pins = 0;
        Isrspec[ 1 ].mask_by_PCMSK_of_valid_devices = 0;
        Isrspec[ 1 ].pcmsk = &PCMSK1;
        Isrspec[ 1 ].mask_by_port_of_current_device_being_actively_communicated_with_thisISR = 0;
        Isrspec[ 1 ].active_pin_ddr_port_reg_addr = 0;
        Isrspec[ 1 ].active_pin_output_port_reg_addr = 0;
        Isrspec[ 1 ].active_pin_pin_reg_addr = 0;
        Isrspec[ 1 ].mask_by_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 1 ].index_in_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 1 ].start_time_plus_max_acq_time_in_uSecs = 0;
        Isrspec[ 1 ].next_bit_coming_from_dht = 255;
        Isrspec[ 1 ].timestamps[ 0 ] = 0;
        Isrspec[ 1 ].interval = 255;
        Isrspec[ 1 ].offset = 0;
        Isrspec[ 1 ].val_tmp1 = ( unsigned short* )&Isrspec[ 1 ].sandbox_bytes[ 1 ];
        Isrspec[ 1 ].val_tmp2 = ( unsigned short* )&Isrspec[ 1 ].sandbox_bytes[ 3 ];
        Isrspec[ 1 ].millis_rest_length = Devprot[ 0 ].millis_rest_length;
        for( u8 m = 0;m < sizeof( Isrspec[ 1 ].array_of_all_devspec_index_plus_1_this_ISR ) ;m++ )
            Isrspec[ 1 ].array_of_all_devspec_index_plus_1_this_ISR[ m ] = 0;
        for( u8 m = 0;m < sizeof( Isrspec[ 1 ].array_of_all_devprot_index_this_ISR ) ;m++ )
            Isrspec[ 1 ].array_of_all_devprot_index_this_ISR[ m ] = 0;
    #endif
    #ifdef PCMSK2
        PCMSK2 = 0;
        isrspec_addr2 = &Isrspec[ 2 ];
        Isrspec[ 2 ].mask_by_PCMSK_of_real_pins = 0;
        Isrspec[ 2 ].mask_by_PCMSK_of_valid_devices = 0;
        Isrspec[ 2 ].pcmsk = &PCMSK2;
        Isrspec[ 2 ].mask_by_port_of_current_device_being_actively_communicated_with_thisISR = 0;
        Isrspec[ 2 ].active_pin_ddr_port_reg_addr = 0;
        Isrspec[ 2 ].active_pin_output_port_reg_addr = 0;
        Isrspec[ 2 ].active_pin_pin_reg_addr = 0;
        Isrspec[ 2 ].mask_by_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 2 ].index_in_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 2 ].start_time_plus_max_acq_time_in_uSecs = 0;
        Isrspec[ 2 ].next_bit_coming_from_dht = 255;
        Isrspec[ 2 ].timestamps[ 0 ] = 0;
        Isrspec[ 2 ].interval = 255;
        Isrspec[ 2 ].offset = 0;
        Isrspec[ 2 ].val_tmp1 = ( unsigned short* )&Isrspec[ 2 ].sandbox_bytes[ 1 ];
        Isrspec[ 2 ].val_tmp2 = ( unsigned short* )&Isrspec[ 2 ].sandbox_bytes[ 3 ];
        Isrspec[ 2 ].millis_rest_length = Devprot[ 0 ].millis_rest_length;
        for( u8 m = 0;m < sizeof( Isrspec[ 2 ].array_of_all_devspec_index_plus_1_this_ISR ) ;m++ )
            Isrspec[ 2 ].array_of_all_devspec_index_plus_1_this_ISR[ m ] = 0;
        for( u8 m = 0;m < sizeof( Isrspec[ 2 ].array_of_all_devprot_index_this_ISR ) ;m++ )
            Isrspec[ 2 ].array_of_all_devprot_index_this_ISR[ m ] = 0;
    #endif
    #ifdef PCMSK3
        PCMSK3 = 0;
        isrspec_addr3 = &Isrspec[ 3 ];
        Isrspec[ 3 ].mask_by_PCMSK_of_real_pins = 0;
        Isrspec[ 3 ].mask_by_PCMSK_of_valid_devices = 0;
        Isrspec[ 3 ].pcmsk = &PCMSK3;
        Isrspec[ 3 ].mask_by_port_of_current_device_being_actively_communicated_with_thisISR = 0;
        Isrspec[ 3 ].active_pin_ddr_port_reg_addr = 0;
        Isrspec[ 3 ].active_pin_output_port_reg_addr = 0;
        Isrspec[ 3 ].active_pin_pin_reg_addr = 0;
        Isrspec[ 3 ].mask_by_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 3 ].index_in_PCMSK_of_current_device_within_ISR = 0;
        Isrspec[ 3 ].start_time_plus_max_acq_time_in_uSecs = 0;
        Isrspec[ 3 ].next_bit_coming_from_dht = 255;
        Isrspec[ 3 ].timestamps[ 0 ] = 0;
        Isrspec[ 3 ].interval = 255;
        Isrspec[ 3 ].offset = 0;
        Isrspec[ 3 ].val_tmp1 = ( unsigned short* )&Isrspec[ 3 ].sandbox_bytes[ 1 ];
        Isrspec[ 3 ].val_tmp2 = ( unsigned short* )&Isrspec[ 3 ].sandbox_bytes[ 3 ];
        Isrspec[ 3 ].millis_rest_length = Devprot[ 0 ].millis_rest_length;
        for( u8 m = 0;m < sizeof( Isrspec[ 3 ].array_of_all_devspec_index_plus_1_this_ISR ) ;m++ )
            Isrspec[ 3 ].array_of_all_devspec_index_plus_1_this_ISR[ m ] = 0;
        for( u8 m = 0;m < sizeof( Isrspec[ 3 ].array_of_all_devprot_index_this_ISR ) ;m++ )
            Isrspec[ 3 ].array_of_all_devprot_index_this_ISR[ m ] = 0;
    #endif
#endif
//Isrspec[ ].mask_by_PCMSK_of_valid_devices needs to get set here as well
/*
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.print( F( "Entering reset_ISR_findings_and_reprobe" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.flush();
    Serial.end();
*/
    digitalWrite( LED_BUILTIN, HIGH );                                           //A high is used to disable relays and whatnots so they don't get driven during the dht device detection process.  The circuitry for that is the end-user's responsibility
    delay( 100 ); //Allow time for even a mechanical relay to operate

    bool pinmode[ sizeof( pins_NOT_safe_to_toggle_during_testing ) ];
    bool pinstate[ sizeof( pins_NOT_safe_to_toggle_during_testing ) ];

    for ( u8 f = 0; f < sizeof( pins_NOT_safe_to_toggle_during_testing ); f++ ) //This loop appears to set protected pins only to outputs
    { //what we need is to set all real pins, skipping not_low_Z pins, to outputs while making the protected ones keep their level, safe ones be high.
//Then end this for loop and start another to cycle through all pins, skipping the "unsafe for any reason" ones, making each safe pin low, then high and check for interrupt
//After entire interrupt pins loop completed, pins with DHT devices on them need to be set to output, low.
/*
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.print( F( "Pin " ) );
    Serial.print( pins_NOT_safe_to_toggle_during_testing[ f ] );
    Serial.print( F( ",1=Out, 0=In:" ) );
//    Serial.print( F( " is starting as " ) );
    Serial.print( ( bool )( ( byte )*portModeRegister( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) \ 
            & ( byte )digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) );
    Serial.print( F( ", level: " ) );
    Serial.print( ( bool )( digitalRead( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) );
*/
//Store initial mode and state of semi-protected pins only
    pinmode[ f ] = ( bool )( ( byte )*portModeRegister( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) \
    & ( byte )digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ f ] ) );
    pinstate[ f ] = ( bool )( digitalRead( pins_NOT_safe_to_toggle_during_testing[ f ] ) );
//    if( pinmode[ f ] == OUTPUT ) continue;
/*
    if( ( bool )( ( byte )*portModeRegister( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) \ 
            & ( byte )digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) )
            { 
                Serial.print( F( " Out" ) );
                pinMode( pins_NOT_safe_to_toggle_during_testing[ f ], OUTPUT ); //reinforce
                pinMode( pins_NOT_safe_to_toggle_during_testing[ f ], INPUT );  //Switch
            }
    else 
    { 
        Serial.print( F( " In" ) );
        pinMode( pins_NOT_safe_to_toggle_during_testing[ f ], INPUT ); //reinforce
        pinMode( pins_NOT_safe_to_toggle_during_testing[ f ], OUTPUT );  //Switch
    }

//Verifying:
    Serial.print( F( ":" ) );
    Serial.print( F( " Switched: " ) );
    Serial.print( ( bool )( ( byte )*portModeRegister( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) \ 
            & ( byte )digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) );
//    Serial.print( F( ", which is" ) );
    if( ( bool )( ( byte )*portModeRegister( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) \ 
            & ( byte )digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) )
            { 
                Serial.print( F( " Out" ) );
            }
    else 
    { 
        Serial.print( F( " In" ) );
    }


//    Serial.print( F( "put that reads" ) );
    if( digitalRead( pins_NOT_safe_to_toggle_during_testing[ f ] ) == HIGH )Serial.print( F( " HIGH ( " ) );
    else Serial.print( F( " LOW ( " ) );
    Serial.print( digitalRead( pins_NOT_safe_to_toggle_during_testing[ f ] ) );

//Verified by pinMode commands
            
    
    Serial.print( F( " )-->" ) );
*/
//Change mode to OUTPUT only if pin is an INPUT, and keep the state
//    digitalWrite( pins_NOT_safe_to_toggle_during_testing[ f ], pinstate[ f ] );
//    pinMode( pins_NOT_safe_to_toggle_during_testing[ f ], OUTPUT );
//    digitalWrite( pins_NOT_safe_to_toggle_during_testing[ f ], pinstate[ f ] );
/*
    Serial.print( ( bool )( ( byte )*portModeRegister( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) \ 
            & ( byte )digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) );
//    Serial.print( F( ", which is" ) );
    if( ( bool )( ( byte )*portModeRegister( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) \ 
            & ( byte )digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) ) Serial.print( F( " Out" ) );
    else Serial.print( F( " In" ) );
//    Serial.print( F( "put that reads" ) );
    if( digitalRead( pins_NOT_safe_to_toggle_during_testing[ f ] ) == HIGH )Serial.print( F( " HIGH ( " ) );
    else Serial.print( F( " LOW ( " ) );
    Serial.print( digitalRead( pins_NOT_safe_to_toggle_during_testing[ f ] ) );

    Serial.print( F( " )-->" ) );

    
//    Serial.print( F( " ), and restored to " ) );
//Change back to initial mode and state
    if ( pinmode[ f ] == OUTPUT ) digitalWrite( pins_NOT_safe_to_toggle_during_testing[ f ], pinstate[ f ] );
    Serial.print( pinmode[ f ] );
    pinMode( pins_NOT_safe_to_toggle_during_testing[ f ], pinmode[ f ] ); //Is this the right level?
    if( ( bool )( ( byte )*portModeRegister( digitalPinToPort( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) \ 
            & ( byte )digitalPinToBitMask( pins_NOT_safe_to_toggle_during_testing[ f ] ) ) ) Serial.print( F( " Out" ) );
    else Serial.print( F( " In" ) );
//    Serial.print( F( "put that reads " ) );
    if( digitalRead( pins_NOT_safe_to_toggle_during_testing[ f ] ) == HIGH )Serial.print( F( " HIGH ( " ) );
    else Serial.print( F( " LOW ( " ) );
    Serial.print( digitalRead( pins_NOT_safe_to_toggle_during_testing[ f ] ) );
    Serial.print( F( " )." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.flush();
    Serial.end();
    */
    }


/* 
Serial.begin( 57600 ); //This speed is very dependent on the host's ability
Serial.setTimeout( 10 ); //
while ( !Serial ) { 
; // wait for serial port to connect. Needed for Leonardo's native USB
 } 
 */

    for ( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ )
    {
        if ( !pin_NOT_safe_even_to_make_low_Z_during_testing( pin ) ) //should only check for membership in pins_NOT_safe_even_to_make_low_Z_during_testing
        { 
            volatile u8* _portddr = portModeRegister( digitalPinToPort( pin ) );
            volatile u8* _port = portOutputRegister( digitalPinToPort( pin ) );
            bool pinlevel = ( bool ) ( *portInputRegister( digitalPinToPort( pin ) ) & digitalPinToBitMask( pin ) ); // store the level of the pin
/* 
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
Serial.print( F( "For pin D" ) );
Serial.print( pin );
Serial.print( F( " PORT" ) );
Serial.print( ( char ) ( digitalPinToPort( pin ) + 64 ) );
Serial.print( F( " " ) );
//Serial.print( F( " mask " ) );
Serial.print( digitalPinToBitMask( pin ), BIN );
Serial.print( F( ", IN levels in " ) );
Serial.print( *portInputRegister( digitalPinToPort( pin ) ), BIN );
//Serial.print( F( ", modes in " ) );
//Serial.print( *_portddr, BIN );
Serial.print( F( ", OUT levels in " ) );
Serial.print( *_port, BIN );
*/
            if ( pinlevel )
            { 
//              if ( pin == 56 ) Serial.print( *_port, BIN );  //set the pin to the level it reads
//Serial.print( F( " HIGH " ) );
              *_port = *portInputRegister( digitalPinToPort( pin ) ) | digitalPinToBitMask( pin ); 
//              if ( pin == 56 ) Serial.print( *_port, BIN );  //set the pin to the level it reads
            }
            else
            { 
//Serial.print( F( " LOW " ) );
              *_port = *portInputRegister( digitalPinToPort( pin ) ) & ~digitalPinToBitMask( pin ); //set the pin to the level read before
            }
            *_portddr |= digitalPinToBitMask( pin ); //Makes the pin an output
            if ( pinlevel )
            { 
//              if ( pin == 56 ) Serial.print( *_port, BIN );  //set the pin to the level it reads
//Serial.print( F( " HIGH " ) );
              *_port = *portInputRegister( digitalPinToPort( pin ) ) | digitalPinToBitMask( pin ); 
//              if ( pin == 56 ) Serial.print( *_port, BIN );  //set the pin to the level it reads
            }
            else
            { 
//Serial.print( F( " LOW " ) );
              *_port = *portInputRegister( digitalPinToPort( pin ) ) & ~digitalPinToBitMask( pin ); //set the pin to the level read before
            }
//Serial.print( F( ", IN levels out " ) );
//Serial.print( *portInputRegister( digitalPinToPort( pin ) ), BIN );
//Serial.print( F( ", modes out " ) );
//Serial.print( *_portddr, BIN );
//Serial.print( F( ", OUT levels out " ) );
//Serial.print( *_port, BIN );
        }
    }
/* 
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.flush();
    Serial.end();
 */
    for ( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ ) //See if pin is serviced by an ISR( PCINTn_vect ).  Build port masks and PCMSK, PCICR masks.  Skip unsafe pins
    {
//delay( 2000 );//allow time for devices to settle down
/* */
        byte original_port_levels;
            for ( u8 f = 0; f < sizeof( pins_NOT_safe_even_to_make_low_Z_during_testing ); f++ )
            { 
                if ( pin == pins_NOT_safe_even_to_make_low_Z_during_testing[ f ] )
                { 
                    goto EndOfThisPin;
                }
            }
        
        
            for ( u8 f = 0; f < sizeof( pins_NOT_safe_to_toggle_during_testing ); f++ )
            { 
                if ( pin == pins_NOT_safe_to_toggle_during_testing[ f ] )
                { 
                    goto EndOfThisPin;
                }
            }
//No variable instantiations allows from here until EndOfThisPin:
/* */
//        if ( ( digitalPinToPort( pin ) + 64 ) != 'B' ) goto EndOfThisPin;
//        byte safemaskforportofthispinonly = 255;
//        port_indexes_ddrmasks_and_pinlevels
//        tmp_ports_under_test_string
        portaddr = ( byte* )portOutputRegister( digitalPinToPort( pin ) );
        ddraddr = ( byte* )portModeRegister( digitalPinToPort( pin ) );
        pinaddr = ( byte* )portInputRegister( digitalPinToPort( pin ) );
        portchar = ( char ) ( digitalPinToPort( pin ) + 64 );                        //Compute the alpha of the port
        original_port_levels = *pinaddr;

//TESTING THE FOLLOWING LINE: ANDed with protected pins this port mask inverted
//        *portaddr = mask_protected_pins_this_port( digitalPinToPort( pin ) );  //Needs to be 255 to prevent extraneous signals on adjacent lines from tripping interrupts.  This is empirical and admittedly does not address pins on different ports.
//        *portaddr = 255;  //Needs to be 255 to prevent extraneous signals on adjacent lines from tripping interrupts.  This is empirical and admittedly does not address pins on different ports.



        /*
Serial.begin( 57600 ); //This speed is very dependent on the host's ability
Serial.setTimeout( 10 ); //
while ( !Serial ) { 
; // wait for serial port to connect. Needed for Leonardo's native USB
 } 
Serial.print( F( "For pin " ) );
Serial.println( pinStr );
//( char ) ( digitalPinToPort( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ]-1 ) + 64 )
Serial.print( F( " - on PORT" ) );
Serial.print( portchar );
Serial.print( F( " mask " ) );
Serial.println( digitalPinToBitMask( pin ), BIN );
delay( 50 );
Serial.flush();
Serial.end();
*/
        for ( byte mask = 1; mask != 0; mask <<= 1 ) //PCICR != 4 || mask != 0;mask <<= 1 )
        {
            pmask = digitalPinToBitMask( pin );
#ifdef PCMSK
            PCMSK = mask;
    #ifdef PCMSK0 //The purpose of this entry is for rationale only, never expected to materialize
                PCMSK0 = mask;
    #endif
#else
    #ifdef PCMSK0
                PCMSK0 = mask;
    #endif
    #ifdef PCMSK1
                PCMSK1 = mask;
    #endif
    #ifdef PCMSK2
                PCMSK2 = mask;
    #endif
    #ifdef PCMSK3
                PCMSK3 = mask;
    #endif
#endif
//                    *ddraddr |= bit( digitalPinToBitMask( pin ) ); // PRBLM MIGHT BE...WHAT?
//            pinMode( pin, OUTPUT );

                    *portaddr &= ~bit( digitalPinToBitMask( pin ) ); //makes pin low, but requires too much delay?  We're trying it again
//            digitalWrite( pin, LOW ); //Make pin low plus evidently add some delay
            delayMicroseconds( 2 ); //Empirically determined.  Circuit capacitance could make this value insufficient.  Not needed for Leonardo but needed for Uno
            
//            if( ( bool ) *pinaddr & bit( digitalPinToBitMask( pin ) ) )
//            { u8 bogus = 0;bogus++;
/* 
                Serial.begin( 57600 ); //This speed is very dependent on the host's ability
                Serial.setTimeout( 10 ); //
                while ( !Serial ) { 
                  ; // wait for serial port to connect. Needed for Leonardo's native USB
                }
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
                Serial.print( F( "Pin D" ) );
                Serial.print( pin );
                Serial.print( F( "'s PINx reads " ) );
                Serial.print( *pinaddr, BIN );
                Serial.print( F( ", " ) );
                Serial.print( pmask,BIN );
                Serial.print( F( " is the pin mask on the port" ) );
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
//                Serial.print( F( "Pin D" ) );
//                Serial.print( pin );
//                Serial.print( F( " is not showing a LOW level when being so instructed.  A circuit short ( unintended physical contact between conductors ) is likely" ) );
//                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
                Serial.flush();
                Serial.end();
 */
//            }
#ifdef PCMSK
            PCIFR |= B1;
    #ifdef PCMSK0 //The purpose of this entry is for rationale only, never expected to materialize
            PCIFR |= B11;
    #endif
#else
    #ifdef PCMSK0
            PCIFR |= B1;
    #endif
    #ifdef PCMSK1
            PCIFR |= B11;
    #endif
    #ifdef PCMSK2
            PCIFR |= B111;
    #endif
    #ifdef PCMSK3
            PCIFR |= B1111;
    #endif
#endif
//            PCIFR |= B111; //Doesn't help anything in bench environment whenever we've needed help here and there. It clears the PC-Interrupts flags for noisy environments in the field.
            cli();
#ifdef PCMSK
            PCICR = B1;
    #ifdef PCMSK0 //The purpose of this entry is for rationale only, never expected to materialize
            PCICR = B11;
    #endif
#else
    #ifdef PCMSK0
            PCICR = B1;
    #endif
    #ifdef PCMSK1
            PCICR = B11;
    #endif
    #ifdef PCMSK2
            PCICR = B111;
    #endif
    #ifdef PCMSK3
            PCICR = B1111;
    #endif
#endif
//            PCICR = 7;
            *pinaddr = pmask; //toggle pin state
            delayMicroseconds( 20 ); //Empirically determined, value is dependent on circuit capacitance and required signal characteristics.
//            if( digitalRead( pin ) != HIGH )
//            { 
/* 
                Serial.begin( 57600 ); //This speed is very dependent on the host's ability
                Serial.setTimeout( 10 ); //
                while ( !Serial ) { 
                  ; // wait for serial port to connect. Needed for Leonardo's native USB
                }
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
                Serial.print( F( "Pin D" ) );
                Serial.print( pin );
                Serial.print( F( " " ) );
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
                Serial.flush();
                Serial.end();
 */
//
//            }

            PCICR = 0;
            byte catchPCIs = PCIFR & B1111;
            PCIFR |= B1111;
            sei();
/* 
                Serial.begin( 57600 ); //This speed is very dependent on the host's ability
                Serial.setTimeout( 10 ); //
                while ( !Serial ) { 
                  ; // wait for serial port to connect. Needed for Leonardo's native USB
                }
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
                Serial.print( F( "PCMSK0 = " ) );
                Serial.print( PCMSK0, BIN );
                Serial.print( F( ", PCMSK1 = " ) );
                Serial.print( PCMSK1, BIN );
                Serial.print( F( ", PCMSK2 = " ) );
                Serial.print( PCMSK2, BIN );
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
                Serial.flush();
                Serial.end();
 */
//Put the pin levels back the way they were
               if ( ( bool ) ( original_port_levels | digitalPinToBitMask( pin ) ) )
            { 
                *portaddr = *portInputRegister( digitalPinToPort( pin ) ) | digitalPinToBitMask( pin ); 
            }
            else
            { 
                *portaddr = *portInputRegister( digitalPinToPort( pin ) ) & ~digitalPinToBitMask( pin ); //set the pin to the level read before
            }
            u8 position_of_port_index = 0;
            for( u8 devspec_index = 0; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
            {
                if( Devspec[ devspec_index ].Dpin == pin )
                {
                    unsigned long timenow = millis();//A single point of reference to prevent changing during the following
                    Devspec[ devspec_index ].timestamp_of_pin_last_attempted_device_read_millis = 0;
                    Devspec[ devspec_index ].device_busy_resting_this_more_millis = Devprot[ Devspec[ devspec_index ].devprot_index ].millis_rest_length;
                    break;
                }
            }
            if ( catchPCIs == 0 ) goto intcatchdone; //Detour if no INTs occurred
            mask = 0;
/*
//Now could make our own defines so they are correct: not tested

#define digitalPinToPCICR( p )    ( ( ( p ) >= 0 && ( p ) <= 21 ) ? ( &PCICR ) : ( ( u8 * )0 ) )
#define digitalPinToPCICRbit( p ) ( ( ( p ) <= 7 ) ? 2 : ( ( ( p ) <= 13 ) ? 0 : 1 ) )
#define digitalPinToPCMSK( p )    ( ( ( p ) <= 7 ) ? ( &PCMSK2 ) : ( ( ( p ) <= 13 ) ? ( &PCMSK0 ) : ( ( ( p ) <= 21 ) ? ( &PCMSK1 ) : ( ( u8 * )0 ) ) ) )
#define digitalPinToPCMSKbit( p ) ( ( ( p ) <= 7 ) ? ( p ) : ( ( ( p ) <= 13 ) ? ( ( p ) - 8 ) : ( ( p ) - 14 ) ) )

#define digitalPinToInterrupt( p )  ( ( p ) == 2 ? 0 : ( ( p ) == 3 ? 1 : NOT_AN_INTERRUPT ) )
*/
            //These lines now are executed if any interrupt occurred on this pin
// **********************************An interrupt occurred on this pin**********************************************
//  find the matching port looking up the index by search for it through
/*
 * Goal is to enable later minimizing memory useage by only storing ports, pins, and isrs that have DHT devices on them, let's call it sparsing the array
 * That means the indexes will no longer conform to the original board-defined indexes but rather to found order ( pick order ) by pin number traverse order
 * The ISR list starts in ISR index order for the purpose of ISR discovery, but must end up different as well.  It won't be in pick order necessarily, 
 * but holes in the index continuum for ISRs must be removed and thereafter ISR array indexes will be decreased
 * PORT INDEXES OF THE ARRAY ARE ARRANGED IN PICK-ORDER, NOT BOARD PORT INDEX ORDER
 * 
 * to get to proper port entry in array only having the original board-defined index, we get that index by
 */
//JUST DON'T FORGET THAT THE PORT MAY NOT HAVE AN ARRAY ENTRY IF NO DHT DEVICES WERE DISCOVERED ON IT?
//FIGURE OUT IF WE NEED TO KEEP THE ISR_WITH_DHT_port_pinmask_stack_array ARRAY NEXT... NOT NEEDED HERE!
//following lines: Save the port-pin mask of the pin in its Portspec/Isrspec/Pinspec to indicate this bitmask of this pin's port is served by an ISR
// position_of_port_index = index of port in Portspec  
//Save the port-pin mask of the pin to indicate this bitmask of this pin's port is served by an ISR
//error with pin 66 struct is committed next line
// ( u8 )( strchr( ports_string_in_heap_array, portchar ) - ports_string_in_heap_array ) evaluates to 55 on pin 0.  TODO FIX
/*
Serial.begin( 57600 ); //This speed is very dependent on the host's ability
Serial.setTimeout( 10 ); //
while ( !Serial );
    for( u8 i = 0; i < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); i++ )
        if( Devspec[ i ].Dpin == 66 ) 
        {
            for( u8 ij = 0; ij < sizeof( Devspec[ i ].last_valid_data_bytes_from_dht_device )/ sizeof( Devspec[ i ].last_valid_data_bytes_from_dht_device[ 0 ] ); ij++ )
            {
                Serial.print( pin );
                Serial.print( F( ":" ) );
                Serial.print( strchr( ports_string_in_heap_array, portchar ) - ports_string_in_heap_array );
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
                Serial.print( strchr( ports_string_in_heap_array, portchar ) );
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
                Serial.print( portchar );
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
                Serial.print( ports_string_in_heap_array );
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
                Serial.print( F( "," ) );
            }
        }
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.flush();
Serial.end();
*/
//            Portspec[ ( u8 )( strchr( ports_string_in_heap_array, portchar ) - ports_string_in_heap_array ) ].PCINT_pins_mask |= digitalPinToBitMask( pin ); 
            cli(); //Atomic read: Because variables checked below could become multi-byte in an interruptable code environment in a later version.  Irrelevant in this specific version, but giving recognition to the fact anyway
//Make the following into a function:
    #ifdef PCMSK
            if ( catchPCIs & B1 )
            { 
                Isrspec[ 0 ].mask_by_PCMSK_of_real_pins |= PCMSK;
                u8 devspec_index = 0;
                for( ; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
                {
                    if( Devspec[ devspec_index ].Dpin == pin )
                    { 
                        Isrspec[ 0 ].mask_by_PCMSK_of_valid_devices |= PCMSK;    //record a device found at this PCMSK bit
                        Devspec[ devspec_index ].my_isrspec_addr = &Isrspec[ 0 ];
                        break;
                    }
                }
                u8 indexwisePCMSK = 0;//make sure the scope extends outside the next for loop
                for ( ;PCMSK >>= 1; indexwisePCMSK++ );// converted to an index 0-7;
                if( Devspec[ devspec_index ].Dpin == pin ) 
                {
                    Isrspec[ 0 ].array_of_all_devspec_index_plus_1_this_ISR[ indexwisePCMSK ] = devspec_index + 1;//index_in_PCMSK_of_current_device_within_ISR;
                    if( !Isrspec[ 0 ].index_in_PCMSK_of_current_device_within_ISR )
                    {
                        Isrspec[ 0 ].index_in_PCMSK_of_current_device_within_ISR = indexwisePCMSK;
                        Isrspec[ 0 ].mask_by_PCMSK_of_current_device_within_ISR = PCMSK;
                    }
                }
                // Need to store this in an array for this ISR, ordered by PCMSK bit position, will store pin number from which port and bitmask will be obtained as needed
                if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK ][ 0 ] == NULL ) PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK ][ 0 ] = pin + 1;
                else PCINT_pins_by_PCMSK_and_ISR[ 1 ][ indexwisePCMSK ][ 0 ] = pin + 1;
            }
    #endif
    #ifdef PCMSK0
        #ifdef PCMSK
            if ( catchPCIs & B10 )
            { 
                Isrspec[ 1 ].mask_by_PCMSK_of_real_pins |= PCMSK0;
                u8 devspec_index = 0;
                for( ; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
                {
                    if( Devspec[ devspec_index ].Dpin == pin )
                    {
                        Isrspec[ 1 ].mask_by_PCMSK_of_valid_devices |= PCMSK0;    //record a device found at this PCMSK bit
                        Devspec[ devspec_index ].my_isrspec_addr = &Isrspec[ 1 ];
                        break;
                    }
                }
        #else
            if ( catchPCIs & B1 ) //This B1 limits the use of PCMSK0 to systems where PCMSK does not exist
            { //Isrspec[ ].mask_by_PCMSK_of_valid_devices needs to get set here as well
                Isrspec[ 0 ].mask_by_PCMSK_of_real_pins |= PCMSK0;
                u8 devspec_index = 0;
                for( ; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
                {
                    if( Devspec[ devspec_index ].Dpin == pin )
                    {
                        Isrspec[ 0 ].mask_by_PCMSK_of_valid_devices |= PCMSK0;    //record a device found at this PCMSK bit
                        Devspec[ devspec_index ].my_isrspec_addr = &Isrspec[ 0 ];
                        break;
                    }
                }
        #endif
                // Need to store this in an array for this ISR, ordered by PCMSK bit position, will store pin number from which port and bitmask will be obtained as needed
                u8 indexwisePCMSK0 = 0;//make sure the scope extends outside the next for loop
                for ( ;PCMSK0 >>= 1; indexwisePCMSK0++ );// converted to an index 0-7;
        #ifdef PCMSK
                if( Devspec[ devspec_index ].Dpin == pin )
                {
                    Isrspec[ 1 ].array_of_all_devspec_index_plus_1_this_ISR[ indexwisePCMSK0 ] = devspec_index + 1;
                    if( !Isrspec[ 1 ].index_in_PCMSK_of_current_device_within_ISR )
                    {
                        Isrspec[ 1 ].index_in_PCMSK_of_current_device_within_ISR = indexwisePCMSK0;
                        Isrspec[ 1 ].mask_by_PCMSK_of_current_device_within_ISR = PCMSK0;
                    }
                }
                if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK0 ][ 1 ] == NULL ) PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK0 ][ 1 ] = pin + 1;
                else PCINT_pins_by_PCMSK_and_ISR[ 1 ][ indexwisePCMSK0 ][ 1 ] = pin + 1;
        #else
                if( Devspec[ devspec_index ].Dpin == pin )
                {
                    Isrspec[ 0 ].array_of_all_devspec_index_plus_1_this_ISR[ indexwisePCMSK0 ] = devspec_index + 1;
                    if( !Isrspec[ 0 ].index_in_PCMSK_of_current_device_within_ISR )
                    {
                        Isrspec[ 0 ].index_in_PCMSK_of_current_device_within_ISR = indexwisePCMSK0;
                        Isrspec[ 0 ].mask_by_PCMSK_of_current_device_within_ISR = PCMSK0;
                    }
                }
                if ( !PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK0 ][ 0 ] ) PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK0 ][ 0 ] = pin + 1;
                else PCINT_pins_by_PCMSK_and_ISR[ 1 ][ indexwisePCMSK0 ][ 0 ] = pin + 1;
        #endif
            }
    #endif
    #ifdef PCMSK1
            if ( catchPCIs & B10 )
            { 
                Isrspec[ 1 ].mask_by_PCMSK_of_real_pins |= PCMSK1;
                u8 devspec_index = 0;
                for( ; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
                {
                    if( Devspec[ devspec_index ].Dpin == pin )
                    {
                        Isrspec[ 1 ].mask_by_PCMSK_of_valid_devices |= PCMSK1;    //record a device found at this PCMSK bit
                        Devspec[ devspec_index ].my_isrspec_addr = &Isrspec[ 1 ];
                        break;
                    }
                }
                u8 indexwisePCMSK1 = 0;//make sure the scope extends outside the next for loop
                for ( ;PCMSK1 >>= 1; indexwisePCMSK1++ );// converted to an index 0-7;
                if( Devspec[ devspec_index ].Dpin == pin )
                {
                    Isrspec[ 1 ].array_of_all_devspec_index_plus_1_this_ISR[ indexwisePCMSK1 ] = devspec_index + 1;
                    if( !Isrspec[ 1 ].index_in_PCMSK_of_current_device_within_ISR )
                    {
                        Isrspec[ 1 ].index_in_PCMSK_of_current_device_within_ISR = indexwisePCMSK1;
                        Isrspec[ 1 ].mask_by_PCMSK_of_current_device_within_ISR = PCMSK1;
                    }
                }
                if ( !PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK1 ][ 1 ] )  PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK1 ][ 1 ] = pin + 1;
                else PCINT_pins_by_PCMSK_and_ISR[ 1 ][ indexwisePCMSK1 ][ 1 ] = pin + 1;
            }
    #endif
    #ifdef PCMSK2
            if ( catchPCIs & B100 )
            {
                Isrspec[ 2 ].mask_by_PCMSK_of_real_pins |= PCMSK2;
                u8 devspec_index = 0;
                for( ; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
                {
                    if( Devspec[ devspec_index ].Dpin == pin )
                    {
                        Isrspec[ 2 ].mask_by_PCMSK_of_valid_devices |= PCMSK2;    //record a device found at this PCMSK bit
                        Devspec[ devspec_index ].my_isrspec_addr = &Isrspec[ 2 ];
                        break;
                    }
                }

                u8 indexwisePCMSK2 = 0;//make sure the scope extends outside the next for loop
                for ( ;PCMSK2 >>= 1; indexwisePCMSK2++ );// converted to an index 0-7;
                if( Devspec[ devspec_index ].Dpin == pin )
                {
                    Isrspec[ 2 ].array_of_all_devspec_index_plus_1_this_ISR[ indexwisePCMSK2 ] = devspec_index + 1;
                    if( !Isrspec[ 2 ].index_in_PCMSK_of_current_device_within_ISR )
                    {
                        Isrspec[ 2 ].index_in_PCMSK_of_current_device_within_ISR = indexwisePCMSK2;
                        Isrspec[ 2 ].mask_by_PCMSK_of_current_device_within_ISR = PCMSK2;
                    }
                }
                if ( !PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK2 ][ 2 ] )  PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK2 ][ 2 ] = pin + 1;
                else PCINT_pins_by_PCMSK_and_ISR[ 1 ][ indexwisePCMSK2 ][ 2 ] = pin + 1;
            }
    #endif
    #ifdef PCMSK3
            if ( catchPCIs & B1000 )
            {
                Isrspec[ 3 ].mask_by_PCMSK_of_real_pins |= PCMSK3;
                u8 devspec_index = 0;
                for( ; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
                {
                    if( Devspec[ devspec_index ].Dpin == pin )
                    {
                        Isrspec[ 3 ].mask_by_PCMSK_of_valid_devices |= PCMSK3;    //record a device found at this PCMSK bit
                        Devspec[ devspec_index ].my_isrspec_addr = &Isrspec[ 3 ];
                        break;
                    }
                }
                u8 indexwisePCMSK3 = 0;//make sure the scope extends outside the next for loop
                for ( ;PCMSK3 >>= 1; indexwisePCMSK3++ );// converted to an index 0-7;
                if( Devspec[ devspec_index ].Dpin == pin )
                {
                    Isrspec[ 3 ].array_of_all_devspec_index_plus_1_this_ISR[ indexwisePCMSK3 ] = devspec_index + 1;
                    if( !Isrspec[ 3 ].index_in_PCMSK_of_current_device_within_ISR )
                    {
                        Isrspec[ 3 ].index_in_PCMSK_of_current_device_within_ISR = indexwisePCMSK3;
                        Isrspec[ 3 ].mask_by_PCMSK_of_current_device_within_ISR = PCMSK3;
                    }
                }
                Isrspec[ 3 ].pin_numbers_in_PCMSK[ indexwisePCMSK3 ] = pin + 1;
                if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK3 ][ 3 ] == NULL )  PCINT_pins_by_PCMSK_and_ISR[ 0 ][ indexwisePCMSK3 ][ 3 ] = pin + 1;
                else PCINT_pins_by_PCMSK_and_ISR[ 1 ][ indexwisePCMSK3 ][ 3 ] = pin + 1;
            }
    #endif
            sei();
//Checks the DHT-found pins mask on this new-found ( previously unknown to be serviced by an ISR ) ISR-serviced port
//                if ( ( bool ) ( Portspec[ ]. ) ] ) )
//                if ( true )//temp line to allow compiling
//                { //Here we either are getting the size for the array or already had the array started from knowing the size after the first run
//                    ISR_ports_with_DHTs_string[ number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR++ ] = portchar;                       //Only fill this if dht devices are found on it?
//                    ISR_ports_with_DHTs_string[ number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR ] = 0;
intcatchdone:;
            cli();                                                  // If stopping all interrupts like this becomes problematic, we'll instead monitor what the ISRs do to the portspec array
//                    Serial.flush();
//                    Serial.end();
            //All Interrupt "ears" open for any activated DHT devices to be noticed
#ifdef PCMSK
            PCICR = B1;
            PCMSK = 255;
    #ifdef PCMSK0 //The purpose of this entry is for rationale only, never expected to materialize
            PCICR = B11;
            PCMSK0 = 255;
    #endif
#else
    #ifdef PCMSK0
            PCICR = B1;
            PCMSK0 = 255;
    #endif
    #ifdef PCMSK1
            PCICR = B11;
            PCMSK1 = 255;
    #endif
    #ifdef PCMSK2
            PCICR = B111;
            PCMSK2 = 255;
    #endif
    #ifdef PCMSK3
            PCICR = B1111;
            PCMSK3 = 255;
    #endif
#endif
            delayMicroseconds( 270 ); //allow enough time for any activated DHT devices to send their next bit transition
//            catchPCIs = PCIFR & 15;
            while( PCIFR & 15 != 0 )
            { //If any DHT devices were hit, wait here until they stop transmitting.  Each bit will be less than 190 uS
                PCIFR |= B1111;
                delayMicroseconds( 270 );
            }
            PCICR = 0;
#ifdef PCMSK
            PCMSK = 0;
#endif
#ifdef PCMSK0
            PCMSK0 = 0;
#endif
#ifdef PCMSK1
            PCMSK1 = 0;
#endif
#ifdef PCMSK2
            PCMSK2 = 0;
#endif
#ifdef PCMSK3
            PCMSK3 = 0;
#endif
            sei();//Allow interrupts again

        } //End of mask bit step for-loop  EDITOR IS WRONG ABOUT SHOWING THIS B/C IT DOESN'T KNOW OUTCOME OF IFDEFs
        


EndOfThisPin:;
    } //Here is end of do-every-pin loop.  EDITOR IS WRONG ABOUT SHOWING THIS B/C IT DOESN'T KNOW OUTCOME OF IFDEFs    ONLY HERE CAN WE KNOW HOW MANY PORTS SUPPLYING ISRs THERE ARE - number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR

//Here we must restore previous pin settings, skipping the protected pins altogether
    for ( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ )
    { 
        bool this_pin_protected = false;
        for ( u8 f = 0; f < sizeof( pins_NOT_safe_to_toggle_during_testing ); f++ )
        { 
            if ( pin == pins_NOT_safe_to_toggle_during_testing[ f ] )
            { 
                this_pin_protected = true;
                if ( pinmode[ f ] == OUTPUT ) digitalWrite( pin, pinstate[ f ] );
                pinMode( pin, pinmode[ f ] );
                if ( pinmode[ f ] == OUTPUT ) digitalWrite( pin, pinstate[ f ] );
            }
        }
        //Now we set all PCINT pins that have devices to output, low
        if ( !this_pin_protected )
        { 
            u8 a = 0;
            u8 b = 0;
            u8 c = 0;
            while( PCINT_pins_by_PCMSK_and_ISR[ a++ ][ b ][ c ] != pin + 1 && PCINT_pins_by_PCMSK_and_ISR[ a ][ b++ ][ c ] != pin + 1 )
            { 
                a = 0;
                b = b%8;
                if ( !( bool )b ) 
                { 
                    c = ( c + 1 ) % number_of_ISRs;
                    if ( !( bool )c ) break;
                }
            }
            if( !( a == b == c == 0 ) )
            { 
                pinMode( pin, OUTPUT );
                digitalWrite( pin, LOW );
            }
        }
    }
    digitalWrite( LED_BUILTIN, LOW ); //Telling any supporting circuitry ( if end-user added any ) that ISR discovery is done, so pins can now be connected on through to the field
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ) { 
      ; // wait for serial port to connect. Needed for Leonardo's native USB
    }
    bool any_wrong_digitalPinToPCICRbit_reports = false;
    bool any_wrong_digitalPinToPCMSKbit_reports = false;
    for ( u8 j = 0; j < number_of_ISRs; j++ )
    {
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
        Serial.print( F( "For this ISR ( ISR" ) );
        Serial.print( j );
        Serial.print( F( " with PCMSK" ) );
#ifndef PCMSK
        Serial.print( j );
#else
        if ( j > 0 )
            Serial.print( j-1 );
#endif
        Serial.print( F( " ), each PCMSK bit showing the pins that will trigger a pin change interrupt on it:" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        for ( u8 i = 0; i < 8; i++ )
        {
            Serial.print( i );
            Serial.print( F( ": " ) );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] )//true for every pin having an ISR
            { //real pin number = PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1, real ISR number = j, real PCMSK bit index = i
                Serial.print( F( "Can be triggered by each voltage toggle occurring on D" ) );
                Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] < 101 ) Serial.print( F( " " ) );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] < 11 ) Serial.print( F( " " ) );
                print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 );
                Serial.print( F( "( port PORT" ) );
                Serial.print( ( char ) ( digitalPinToPort( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ) + 64 ) );                   //By adding 64, this gets an alpha from the index, index 01 = A
                Serial.print( F( " bit mask " ) );
                Serial.print( digitalPinToBitMask( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ), BIN );
                Serial.print( F( " )" ) );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ j ] ) 
                { 
                    if( digitalPinToPort( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ) != digitalPinToPort( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ j ] - 1 ) )
                        Serial.print( F( " conflicts with D" ) );
                    else
                        Serial.print( F( " and on D" ) );
                    Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ j ] - 1 );
                    Serial.print( F( " " ) );
//                    Serial.print( F( ". Pinxref->PIN_xref[ PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ] = " ) );
//                    Serial.print( Pinxref->PIN_xref[ PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ] );
                    print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ j ] - 1 );
                    Serial.print( F( " ( port PORT" ) );
                    Serial.print( ( char ) ( digitalPinToPort( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ j ] - 1 ) + 64 ) );
                    Serial.print( F( " bit mask " ) );
                    Serial.print( digitalPinToBitMask( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ j ] - 1 ), BIN );
                    Serial.print( F( " )" ) );
                    if( digitalPinToPort( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ) != digitalPinToPort( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ j ] - 1 ) )
                        Serial.print( F( " only the first pin listed can be used with confidence with this software product" ) );
                }
                if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ) != j )
                { 
                    Serial.print( F( "!" ) );
                    any_wrong_digitalPinToPCICRbit_reports = true;
//                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
//                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ) ),BIN );
//                    Serial.print( F( ". Correct value is " ) );
//                    Serial.print( bit( j ),BIN );
                }
                if( digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ) != i )
                { 
                    Serial.print( F( "*" ) );
                    any_wrong_digitalPinToPCMSKbit_reports = true;
//                    Serial.print( F( ", digitalPinToPCMSKbit being reported wrong by macro as " ) );
//                    Serial.print( bit( digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ j ] - 1 ) ),BIN );
//                    Serial.print( F( ". Correct value is " ) );
//                    Serial.print( bit( i ),BIN );
                }
            }
            else Serial.print( F( "No PCINT-to-pin connection or the supported pin is declared protected" ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
        }
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.print( F( "Summary of ISR-to-pin information:" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
#ifdef PCMSK
    Serial.print( F( "ISR - D-pins by PCMSK bit" ) );         // j is ISR number
    #ifdef PCMSK0
        Serial.print( F( "    " ) );
    #endif
#endif
#ifdef PCMSK0
    Serial.print( F( "ISR0 - D-pins by PCMSK0 bit" ) );
#endif
#ifdef PCMSK1
    Serial.print( F( "       ISR1 - D-pins by PCMSK1 bit" ) );
#endif
#ifdef PCMSK2
    Serial.print( F( "       ISR2 - D-pins by PCMSK2 bit" ) );         // j is ISR number
#endif
#ifdef PCMSK3
    Serial.print( F( "       ISR3 - D-pins by PCMSK3 bit" ) );         // j is ISR number
#endif

    for ( u8 i = 0; i < 8; i++ )                                                                                         // i is PCMSK bit
    { 
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
        Serial.print( i );
        Serial.print( F( ": " ) );
#if defined ( PCMSK ) || defined ( PCMSK0 )
        if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] ) 
        { 
            Serial.print( F( "D" ) );
            Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1 );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] < 101 ) Serial.print( F( " " ) );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] < 11 ) Serial.print( F( " " ) );
            print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1 );
//            Serial.print( F( " - " ) );
            if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1 ) != 0 )
            { 
                Serial.print( F( "!" ) );
                any_wrong_digitalPinToPCICRbit_reports = true;
//                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
//                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1 ) ),BIN );
//                    Serial.print( F( ". Correct value is " ) );
//                    Serial.print( bit( 0 ),BIN );
            }
            else Serial.print( F( " " ) );
            if    ( digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1 ) != i )
            { 
                Serial.print( F( "*" ) );
                any_wrong_digitalPinToPCMSKbit_reports = true;
            }
            else Serial.print( F( " " ) );
            if ( ( bool )( Isrspec[ 0 ].mask_by_PCMSK_of_valid_devices & 1<<i ) )
            {
                pinMode( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1, OUTPUT );
                digitalWrite( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1, HIGH ); //This places the level on the DHT devices that they need to be prepared to imminently and immediately send their data, assuming an adequate rest period
                Serial.print( F( "DHT" ) );
            }
            else
            {
                Serial.print( F( "   " ) );
                if( !pin_in_protected_arrays( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1 ) ) pinMode( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 0 ] - 1, INPUT ); 
            }
            if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] ) 
            { 
                Serial.print( F( " and D" ) );
//                Serial.print( digitalPinToPCMSKbit[ 1 ][ i ][ 0 ] - 1 ); 
                Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] - 1 ); 
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] < 101 ) Serial.print( F( " " ) );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] < 11 ) Serial.print( F( " " ) );
                print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] - 1 );
//                Serial.print( F( " - " ) );
                if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] - 1 ) != 0 )
                { 
                    Serial.print( F( "!" ) );
                    any_wrong_digitalPinToPCICRbit_reports = true;
//                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
//                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] - 1 ) ),BIN );
//                    Serial.print( F( ". Correct value is " ) );
//                    Serial.print( bit( 0 ),BIN );
                }
                else Serial.print( F( " " ) );
                if    ( i != digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] - 1 ) )
                { 
                    Serial.print( F( "*" ) );
                    any_wrong_digitalPinToPCMSKbit_reports = true;
                }
                else Serial.print( F( " " ) );
//                Serial.print( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 0 ] - 1 ) );
                Serial.print( F( "    " ) );
            }
            else
            { 
                Serial.print( F( "              " ) );
            }
            Serial.print( F( "   " ) );
        }
        else Serial.print( F( "---                            " ) );
#endif
#if ( defined ( PCMSK ) && defined ( PCMSK0 ) ) || defined ( PCMSK1 )
        if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] ) 
        { 
            Serial.print( F( "D" ) );
            Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1 );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] < 101 ) Serial.print( F( " " ) );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] < 11 ) Serial.print( F( " " ) );
            print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1 );
//            Serial.print( F( " - " ) );
            if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1 ) != 1 )
            { 
                Serial.print( F( "!" ) );
                any_wrong_digitalPinToPCICRbit_reports = true;
//                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
//                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1 ) ),BIN );
//                    Serial.print( F( ". Correct value is " ) );
//                    Serial.print( bit( 1 ),BIN );
            }
            else Serial.print( F( " " ) );
            if    ( i != digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1 ) )
            { 
                Serial.print( F( "*" ) );
                any_wrong_digitalPinToPCMSKbit_reports = true;
            }
            else Serial.print( F( " " ) );
            if ( ( bool )( Isrspec[ 1 ].mask_by_PCMSK_of_valid_devices & 1<<i ) )
            {
                pinMode( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1, OUTPUT );
                digitalWrite( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1, HIGH ); //This places the level on the DHT devices that they need to be prepared to imminently and immediately send their data, assuming an adequate rest period
                Serial.print( F( "DHT" ) );
            }
            else
            {
                Serial.print( F( "   " ) );
                if( !pin_in_protected_arrays( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1 ) ) pinMode( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 1 ] - 1, INPUT );
            }
            if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] ) 
            { 
                Serial.print( F( " and D" ) );
                Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] - 1 );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] < 101 ) Serial.print( F( " " ) );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] < 11 ) Serial.print( F( " " ) );
                print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] - 1 );
//                Serial.print( F( " - " ) );
                if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] - 1 ) != 1 )
                { 
                    Serial.print( F( "!" ) );
                    any_wrong_digitalPinToPCICRbit_reports = true;
    //                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
    //                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] - 1 ) ),BIN );
    //                    Serial.print( F( ". Correct value is " ) );
    //                    Serial.print( bit( 1 ),BIN );
                }
                else Serial.print( F( " " ) );
                if    ( i != digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] - 1 ) )
                { 
                    Serial.print( F( "*" ) );
                    any_wrong_digitalPinToPCMSKbit_reports = true;
                }
                else Serial.print( F( " " ) );
//                Serial.print( digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 1 ] - 1 ) );
                Serial.print( F( "    " ) );
            }
            else
                    Serial.print( F( "              " ) );
            Serial.print( F( "      " ) );
        }
        else Serial.print( F( "---                               " ) );
#endif
#ifdef PCMSK2
        if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] ) 
        { 
            Serial.print( F( "D" ) );
            Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1 );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] < 101 ) Serial.print( F( " " ) );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] < 11 ) Serial.print( F( " " ) );
            print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1 );
//            Serial.print( F( " - " ) );
            if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1 ) != 2 )
            { 
                Serial.print( F( "!" ) );
                any_wrong_digitalPinToPCICRbit_reports = true;
//                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
//                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1 ) ),BIN );
//                    Serial.print( F( ". Correct value is " ) );
//                    Serial.print( bit( 2 ),BIN );
            }
            else Serial.print( F( " " ) );
            if    ( i != digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1 ) )
            { 
                Serial.print( F( "*" ) );
                any_wrong_digitalPinToPCMSKbit_reports = true;
            }
            else Serial.print( F( " " ) );
            if ( ( bool )( Isrspec[ 2 ].mask_by_PCMSK_of_valid_devices & 1<<i ) )
            {
                pinMode( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1, OUTPUT );
                digitalWrite( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1, HIGH ); //This places the level on the DHT devices that they need to be prepared to imminently and immediately send their data, assuming an adequate rest period
                Serial.print( F( "DHT" ) );
            }
            else
            {
                Serial.print( F( "   " ) );
                if( !pin_in_protected_arrays( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1 ) ) pinMode( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 2 ] - 1, INPUT );
            }
            if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] ) 
            { 
                Serial.print( F( " and D" ) );
                Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] - 1 );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] < 101 ) Serial.print( F( " " ) );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] < 11 ) Serial.print( F( " " ) );
                print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] - 1 );
//                Serial.print( F( " - " ) );
                if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] - 1 ) != 2 )
                { 
                    Serial.print( F( "!" ) );
                    any_wrong_digitalPinToPCICRbit_reports = true;
    //                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
    //                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] - 1 ) ),BIN );
    //                    Serial.print( F( ". Correct value is " ) );
    //                    Serial.print( bit( 2 ),BIN );
                }
                else Serial.print( F( " " ) );
                if    ( i != digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] - 1 ) )
                { 
                    Serial.print( F( "*" ) );
                    any_wrong_digitalPinToPCMSKbit_reports = true;
                }
                else Serial.print( F( " " ) );
                Serial.print( F( "    " ) );
//                Serial.print( digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 2 ] - 1 ) );
            }
            else
                    Serial.print( F( "              " ) );
            Serial.print( F( "   " ) );
        }
        else Serial.print( F( "---                               " ) );
#endif
#ifdef PCMSK3
        if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] != NULL ) 
        { 
            Serial.print( F( "D" ) );
            Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] - 1 );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] < 101 ) Serial.print( F( " " ) );
            if ( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] < 11 ) Serial.print( F( " " ) );
            print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ]-1 );
//            Serial.print( F( " - " ) );
            if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] - 1 ) != 3 )
            { 
                Serial.print( F( "!" ) );
                any_wrong_digitalPinToPCICRbit_reports = true;
//                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
//                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] - 1 ) ),BIN );
//                    Serial.print( F( ". Correct value is " ) );
//                    Serial.print( bit( 3 ),BIN );
            }
            else Serial.print( F( " " ) );
            if    ( i != digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] - 1 ) )
            { 
                Serial.print( F( "*" ) );
                any_wrong_digitalPinToPCMSKbit_reports = true;
            }
            else Serial.print( F( " " ) );
            if ( ( bool )( Isrspec[ 3 ].mask_by_PCMSK_of_valid_devices & 1<<i ) )
            {
                pinMode( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] - 1, OUTPUT );
                digitalWrite( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] - 1, HIGH ); //This places the level on the DHT devices that they need to be prepared to imminently and immediately send their data, assuming an adequate rest period
                Serial.print( F( "DHT" ) );
            }
            else
            {
                Serial.print( F( "   " ) );
                if( !pin_in_protected_arrays( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] - 1 ) ) pinMode( PCINT_pins_by_PCMSK_and_ISR[ 0 ][ i ][ 3 ] - 1, INPUT );
            }
            if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] != NULL ) 
            { 
                Serial.print( F( " and D" ) );
                Serial.print( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] - 1 );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] < 101 ) Serial.print( F( " " ) );
                if ( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] < 11 ) Serial.print( F( " " ) );
                print_analog_if_exists( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] - 1 );
//                Serial.print( F( " - " ) );
                if( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] - 1 ) != 3 )
                { 
                    Serial.print( F( "!" ) );
                    any_wrong_digitalPinToPCICRbit_reports = true;
    //                    Serial.print( F( ", digitalPinToPCICRbit being reported wrong by macro as " ) );
    //                    Serial.print( bit( digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] - 1 ) ),BIN );
    //                    Serial.print( F( ". Correct value is " ) );
    //                    Serial.print( bit( 3 ),BIN );
                }
                else Serial.print( F( " " ) );
                if    ( i != digitalPinToPCICRbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] - 1 ) )
                { 
                    Serial.print( F( "*" ) );
                    any_wrong_digitalPinToPCMSKbit_reports = true;
                }
                else Serial.print( F( " " ) );
                Serial.print( F( "    " ) );
//                Serial.print( digitalPinToPCMSKbit( PCINT_pins_by_PCMSK_and_ISR[ 1 ][ i ][ 3 ] - 1 ) );
            }
            else
                Serial.print( F( "   " ) );
//            if ( Isrspec[ 3 ].mask_by_PCMSK_of_valid_devices
//            else Serial.print( F( "              " ) );
//            Serial.print( F( "      " ) );
        }
        else Serial.print( F( "---" ) );
#endif
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    if ( any_wrong_digitalPinToPCICRbit_reports )
    { 
        Serial.print( F( "! = digitalPinToPCICRbit() function reports the wrong PCICR for this pin.  This software product will work around it; other software will likely not correct the report error" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    }
    if ( any_wrong_digitalPinToPCMSKbit_reports )
    { 
        Serial.print( F( "* = digitalPinToPCMSKbit() function reports the wrong PCMSK bit for this pin.  This software product will work around it; other software will likely not correct the report error" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    }
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 

    
/*
    
    for( u8 devspec_index = 0; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
    {
        Serial.print( Devspec[ devspec_index ].Dpin );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    }
*/
//    Serial.print( F( "ports_with_ISRs_string = " ) );
//    Serial.print( ports_with_ISRs_string );
//    strcpy( scratchpad, ports_with_ISRs_string );
//    scratchpad[ strchr( ports_with_ISRs_string, 0 ) - &ports_with_ISRs_string[ 0 ] ] = 0;
//    Serial.print( F( ", scratchpad = " ) );
//    Serial.print( scratchpad );
//    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
/* every element is populated: unsparsed */
/*
    for( u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++ )
    {
        Serial.print( Pinxref->PIN_xref[ pin ] );
        Serial.print( F( ": " ) );
//        Serial.print( Devspec[ Pinxref->PIN_xref[ pin ] ].Dpin );
//        Serial.print( F( ", " ) );
//        Serial.print( Devspec[ Pinxref->PIN_xref[ pin ] ].next_bit_coming_from_dht );
//        Serial.print( F( ", " ) );
//        Serial.print( Devspec[ Pinxref->PIN_xref[ pin ] ].devprot_index );
//        Serial.print( F( ", " ) );
//        Serial.print( Devspec[ Pinxref->PIN_xref[ pin ] ].portspec_index_for_pin );
//        Serial.print( F( ", " ) );
        Serial.print( Devspec[ Pinxref->PIN_xref[ pin ] ].devprot_index );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 ); 
    }
 */
    Serial.flush();
    Serial.end();
//WE FINALLY KNOW HOW MANY DHT DEVICES SERVED BY ISR THERE ARE.  MAKE AN ARRAY OF THEM WITH A XREF ARRAY
    for( u8 i = 0; i < number_of_ISRs; i++ )//free previous malloc of isrspec and refill isrspec_addr0 - 3 and reset Devspec.my_isrspec_addr
        if ( Isrspec[ i ].mask_by_PCMSK_of_valid_devices )
        {
            Isrxref->ISR_xref[ number_of_populated_isrs++ ] = i;
        }
//    for( u8 i = 0; i < ; i++ )
//        Devxref->DEV_xref[ i ] = i;

//Now we need to fix ISR_WITH_DHT_port_pinmask_stack_array array while ports_with_ISRs_string is in scope
//Extract each element in ports_with_ISRs_string from ISR_WITH_DHT_port_pinmask_stack_array and push it with counter++
//Then free ISR_WITH_DHT_port_pinmask_stack_array and re-malloc and fill it from pushed elements//wrong scope still
//free ( ISR_WITH_DHT_port_pinmask_stack_array );
//
/*
 *     previous_ISR_WITH_DHT_port_pinmask_stack_array = ( void* )pre_array_boards_ports_string;//Just as a matter of good practice, doesn't really do anything here. everywhere else before a free instruction, this line will better match the names of the pointers
    free ( previous_ISR_WITH_DHT_port_pinmask_stack_array ); //Just as a matter of good practice, doesn't really do anything here
    Elements_in = ( ELEMENTS_IN* )malloc( \
        sizeof( ELEMENTS_IN ) + \
        ( sizeof( ISRSPEC ) * number_of_ISRs ) + \
        ( sizeof( PORTSPEC ) * number_of_ports_found ) + \
        ( sizeof( DHT_NO_ISR ) * NUM_DIGITAL_PINS ) + \
        ( sizeof( PINSPEC ) * NUM_DIGITAL_PINS ) + \
         0 + 0 \
        );//not using DEVSPEC at this time */
         /* not using DEVICE_PROTOCOL at this time */
/*
 * Goal is to minimize memory useage by only storing ports, pins, and isrs that have DHT devices on them, let's call it stuffing
 * That means the indexes will no longer conform to the original board-defined indexes but rather to found order ( pick order ) by pin number traverse order
 * The ISR list starts in ISR index order for the purpose of ISR discovery, but must end up different as well.  It won't be in pick order necessarily, 
 * but holes in the index continuum for ISRs must be removed and thereafter ISR indexes will be decreased
 * 
 */
 /* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "line 2302" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */
    
//    if( Elements_in == NULL ) return ( false );
/* 
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( F( "line 2312" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
 */
//ELEMENTS_IN structure holds the following elements, each storing the number of elements in an array of types, for all the various arrays
/* 
ISR XREF TABLE: BYTE PER NUMBER OF ISRS
DEVSPEC XREF TABLE: BYTE PER ( NUMBER OF ISRS TIMES EIGHT ) 24 OR 32
    u8 ISRSPEC = 0;
    u8 PORTSPEC = 0;
    u8 DHT_NO_ISR = 0;
    u8 PINSPEC = 0;
    u8 DEVSPEC = 0;
    u8 DEVICE_PROTOCOL = 0;
ELEMENTS_IN* Elements_in;
PORTSPEC* Portspec;
DEVSPEC* Devspec;
ISRSPEC* Isrspec;
PINSPEC* Pinspec;
DHT_NO_ISR* Dht_no_isr;
*/

/*
Serial.print( F( "Scratchpad = " ) );
Serial.print( scratchpad );
Serial.print( F( ", Size of scratchpad = " ) );
Serial.print( number_of_ports_that_responded_to_ISR_probing );
Serial.print( F( ", number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR = " ) );
Serial.print( number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR );
Serial.print( F( ", number_of_ports_found = " ) );
Serial.print( number_of_ports_found );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
*/
/*
byte* ISR_WITH_DHT_port_pinmask_stack_array; //ptr to location
byte* DHT_without_ISR_port_pinmask_stack_array; //ptr to location
byte* previous_ISR_WITH_DHT_port_pinmask_stack_array = NULL; //ptr to location to check for memory fragmentation
byte* previous_DHT_without_ISR_port_pinmask_stack_array = NULL; //ptr to location to check for memory fragmentation
 */

/*
    u8 tmp1_index = 0;
    u8 tmp2_index = 0;
    for ( u8 tmp_index = 0; tmp_index < number_of_elements_in_ISR_part_of_port_pinmask_stack; tmp_index++ )
    { 
        bool dont = false;
        if ( ( bool ) *&ISR_WITH_DHT_port_pinmask_stack_array[ tmp_index + ( 4 * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ] )
        { 
            for( u8 tmp4_index = 0; tmp4_index < element_bytes_in_ISR_part_of_port_pinmask; tmp4_index++ )
                ISR_WITH_DHT_port_pinmask_stack_array_tmp[ tmp1_index + ( tmp4_index * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ] = *&ISR_WITH_DHT_port_pinmask_stack_array[ tmp_index + ( tmp4_index * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ];
            tmp1_index++;
        }
        else if ( ( bool ) *&ISR_WITH_DHT_port_pinmask_stack_array[ tmp_index + ( 3 * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ] )
        { 
            for( u8 tmp3_index = 0; tmp3_index < element_bytes_in_ISR_part_of_port_pinmask; tmp3_index++ )
                ISR_WITH_DHT_port_pinmask_stack_array_tmp[ tmp2_index + ( tmp3_index * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ] = *&ISR_WITH_DHT_port_pinmask_stack_array[ tmp_index + ( tmp3_index * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ];
            tmp2_index++;
        }
        else dont = true;
*/
        
/* 
        if ( !dont )
        { 
            Serial.begin( 57600 ); //This speed is very dependent on the host's ability
            Serial.setTimeout( 10 ); //
            while ( !Serial ) { 
              ; // wait for serial port to connect. Needed for Leonardo's native USB
            }
         */
        /* 
            Serial.print( F( "Transfering PORT" ) );
            Serial.print( ( char )( *&ISR_WITH_DHT_port_pinmask_stack_array[ tmp_index ] + 64 ) );
            Serial.print( F( ": " ) );
            Serial.print( ISR_WITH_DHT_port_pinmask_stack_array_tmp[ tmp_index + number_of_elements_in_ISR_part_of_port_pinmask_stack ], BIN );
            Serial.print( F( ", " ) );
            Serial.print( ISR_WITH_DHT_port_pinmask_stack_array_tmp[ tmp_index + ( 2 * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ], BIN );
            Serial.print( F( ", " ) );
            Serial.print( ISR_WITH_DHT_port_pinmask_stack_array_tmp[ tmp_index + ( 3 * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ], BIN );
            Serial.print( F( ", " ) );
            Serial.print( ISR_WITH_DHT_port_pinmask_stack_array_tmp[ tmp_index + ( 4 * number_of_elements_in_ISR_part_of_port_pinmask_stack ) ], BIN );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        */ /*
            Serial.flush();
            Serial.end();
        }
*/
/*
    }
*/

/*
Serial.print( F( "Reading temp storage back:" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
*/
/*
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * The following is not getting completed correctly
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */
//debugging delay, remove if able to later
/*
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ) { 
          ; // wait for serial port to connect. Needed for Leonardo's native USB
        }
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "Line 2403" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
*/

//    delay( 100 );
//    unsigned short sparsecheck = resistor_between_LED_BUILTIN_and_PIN_A1();
//    if ( sparsecheck > 0 && sparsecheck < 101 )
//        compact_the_heap( true );
    return ( true );
}// EDITOR IS WRONG ABOUT SHOWING THIS B/C IT DOESN'T KNOW OUTCOME OF IFDEFs


void setup() {
#ifndef OCR0A
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ); // wait for serial port to connect. Needed for Leonardo's native USB
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "A necessary feature is not available: Timer0's A comparison register." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "This sketch will end." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.flush();
    Serial.end();
    delay( 100000 );
    return ;
#endif
    if( ( ( TIMSK0 & 2 ) || TCCR0A != 3 ) && OCR0A != 0xA1 )
    {
        Serial.begin( 57600 ); //This speed is very dependent on the host's ability
        Serial.setTimeout( 10 ); //
        while ( !Serial ); // wait for serial port to connect. Needed for Leonardo's native USB
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "Judging from the contents of Timer0's A Output Compare Match registers, some other process is using the comparison feature this sketch is designed for" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.print( F( "Rather than disable anything else, this sketch will end because it is not sophisticated enough to use Match B as backup while other processes may interfere" ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        Serial.flush();
        Serial.end();
        return ;
    }
    
#ifdef TIMER0_COMPA_vect
OCR0A = 0xA1; //To enable the 1 mSec off-phase ( compare ) interrupt
TIFR0 &= 0xFD; // to avoid an immediate interrupt occurring.  Clear this like this before expecting full first cycle
#endif
    Serial.begin( 57600 ); //This speed is very dependent on the host's ability
    Serial.setTimeout( 10 ); //
    while ( !Serial ); // wait for serial port to connect. Needed for Leonardo's native USB
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "Arduino DHTs on Interrupt Steroids Sketch" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "This sketch will display the numbers of all digital pins with the ports and port masks" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "for them, detect and display all DHT devices connected (even those on pins not" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "supporting Pin Change Interrupts), and detect the existence of all Pin Change Interrupts" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "supported by the microcontroller board and display them for you." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "IT LEARNS THE INTERRUPT DETAIL BY TOGGLING PINS, so all pins must be free to toggle for" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "the results shown to be correct." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "If you need to, you may protect pins from being tested for devices by listing them in one" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "of the two protected pin arrays.  The built-in LED renders its pin useless for DHT use," ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "so that pin is included in the list of protected pins by default and is given an alternate" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "function, if you need it, of being high during the duration of the device detection process." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "The intent is so it can be used to control signal-gating circuitry of your design and" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "construction to effectively disconnect pin signals and protect driven devices from the" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "extraneous toggling occurring during device detection." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "If you see nonsense characters displayed associated with the detection process, please" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "take the time now to add your board's serial communication pins to one of these" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.print( F( "protecting arrays if you'll be using pins for serial communications." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    Serial.flush();
    Serial.end();
/* */
//Leonardo has 2344 bytes of free 
//ram here in unsparsed model
//Mega 2560 has 7904 bytes of free ram here in unsparsed model
//UNO has 1761 bytes of free ram here in unsparsed model: as follows
//Sketch uses 28810 bytes (89%) of program storage space. Maximum is 32256 bytes.
//Global variables use 379 bytes (18%) of dynamic memory, leaving 1669 bytes for local variables. Maximum is 2048 bytes.
    unsigned short wincheck = resistor_between_LED_BUILTIN_and_PIN_A0();
    if ( wincheck > 0 && wincheck < 101 ) mswindows = true ; else mswindows = false;
    build_from_nothing();
    delay( 2000 );
    number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR = 0;    //This tells reset_ISR_findings_and_reprobe() that we need this variable re-valued to know how large the ports_with_DHT_devices... array must be initialized for
    reset_ISR_findings_and_reprobe ( false ); //uses number_of_ports_with_functioning_DHT_devices_and_serviced_by_ISR to build ptr_to_portspecs_stack
    TIMSK0 |= 2;  //enables the compare value for match A , = 1 go back to normal.  First run through
Serial.flush();
Serial.end();
Serial.begin( 57600 ); //This speed is very dependent on the host's ability
Serial.setTimeout( 10 ); //
while ( !Serial );
Serial.flush();
Serial.end();

Serial.begin( 57600 ); //This speed is very dependent on the host's ability
Serial.setTimeout( 10 ); //
while ( !Serial ) ; // wait for serial port to connect. Needed for Leonardo's native USB
if( !( ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ) ) )
{
    Serial.print( F( "No DHT devices were detected, so the following statements are null and void:" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
}
Serial.print( F( "Factory sketch functions: enter the letter A or a number between 0 and " ) );
Serial.print( ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ) );
Serial.print( F( " with your entire" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "entry enclosed between these two characters: < and >.  Entering the letter A so enclosed" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "will list all DHT devices each with its last " ) );
Serial.print( confidence_depth );
Serial.print( F( " values obtained.  Entering the index" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "number of any selected device will do the same for the one device only.  Reading errors" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "that occur are displayed asynchronously by void loop() as they happen." ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.flush();
}

const byte numChars = 32;
char receivedChars[ numChars ];   // an array to store the received data
boolean newData = false;
void recvWithStartEndMarkers() //COURTESY Robin2 ON http://forum.arduino.cc/index.php?topic=396450
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() //COURTESY Robin2 ON http://forum.arduino.cc/index.php?topic=396450
{
    if( newData )
    {
//        Serial.print( receivedChars );
//        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        DEVSPEC* this_Devspec_address;
        u8 tmp_sandbox;
        u8 filled_vals;
        u8 ilvr;
        if( strstr( receivedChars, "a" ) || strstr( receivedChars, "A" ) )
        {
            for( u8 devspec_index = 0; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
            {
                this_Devspec_address = &Devspec[ devspec_index ];
                tmp_sandbox;
                filled_vals = 0;
                ilvr = this_Devspec_address->index_of_next_valid_readings_sets;
                Serial.print( F( "At location #" ) );
                if( devspec_index < 10 ) Serial.print( F( " " ) );
                Serial.print( devspec_index );
                if( this_Devspec_address->Dpin < 10 ) Serial.print( F( " " ) );
                Serial.print( F( " is pin D" ) );
                Serial.print( this_Devspec_address->Dpin );
                Serial.print( F( " " ) );
                print_analog_if_exists( this_Devspec_address->Dpin );
                Serial.print( F( "(DHT" ) );
                if( this_Devspec_address->devprot_index ) Serial.print( F( "22" ) );
                else Serial.print( F( "11" ) );
                Serial.print( F( "): " ) );
                Serial.flush();
                for( signed char ij = confidence_depth - 1; ij >= 0 ;ij-- )
                {
                    if( this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( u8 )( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] < 10 ) Serial.print( F( " " ) );
                    Serial.print( this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( u8 )( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] );
                    Serial.print( F( "." ) );
                    Serial.print( this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( u8 )( 1 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ) ] );
                    Serial.print( F( "% " ) );
            
                    for( u8 ik = filled_vals = 0; ik < confidence_depth; ik++ )
                        if( this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ik ) ] )
                            filled_vals++;
                    for( u8 ik = tmp_sandbox = 0; ik < confidence_depth; ik++ )
                        if( this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ik ) ] & 0x80 ) tmp_sandbox++;
                    if( ( this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] & 0x7F ) < 10 ) Serial.print( F( " " ) );
                    if( tmp_sandbox > ( filled_vals >> 1 ) )
                        Serial.print( F( "-" ) );
            
                    Serial.print( this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] & 0x7F );
                    Serial.print( F( "." ) );
                    Serial.print( this_Devspec_address->last_valid_data_bytes_from_dht_device[ 3 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] );
            
                    Serial.print( F( "C " ) );
                }
                Serial.print( F( "age in seconds = " ) );
                if( ( ( float )( ( unsigned long )( millis() - this_Devspec_address->timestamp_of_pin_valid_data_millis ) ) / 1000 ) < 10 ) Serial.print( F( " " ) );
                Serial.print( ( float )( ( unsigned long )( millis() - this_Devspec_address->timestamp_of_pin_valid_data_millis ) ) / 1000 );
                Serial.print( F( " " ) );
                if( ( ( float )( ( unsigned long )( millis() - this_Devspec_address->timestamp_of_pin_last_attempted_device_read_millis ) ) / 1000 ) < 10 ) Serial.print( F( " " ) );
                Serial.print( ( float )( ( unsigned long )( millis() - this_Devspec_address->timestamp_of_pin_last_attempted_device_read_millis ) ) / 1000 );
                Serial.print( F( " = last_attempted_read seconds ago. remaining rest: " ) );
                Serial.print( this_Devspec_address->device_busy_resting_this_more_millis );
                Serial.print( F( "mS " ) );
                Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
                Serial.flush();
            }
        }
        else if( atoi( receivedChars ) < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ) )
        {
            this_Devspec_address = &Devspec[ atoi( receivedChars ) ];
            tmp_sandbox;
            filled_vals = 0;
            ilvr = this_Devspec_address->index_of_next_valid_readings_sets;
            Serial.print( F( "At location #" ) );
            if( atoi( receivedChars ) < 10 ) Serial.print( F( " " ) );
            Serial.print( atoi( receivedChars ) );
            if( this_Devspec_address->Dpin < 10 ) Serial.print( F( " " ) );
            Serial.print( F( " is pin D" ) );
            Serial.print( this_Devspec_address->Dpin );
            Serial.print( F( " " ) );
            print_analog_if_exists( this_Devspec_address->Dpin );
            Serial.print( F( "(DHT" ) );
            if( this_Devspec_address->devprot_index ) Serial.print( F( "22" ) );
            else Serial.print( F( "11" ) );
            Serial.print( F( "): " ) );
            for( signed char ij = confidence_depth - 1; ij >= 0 ;ij-- )
            {
                Serial.print( this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( u8 )( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] );
                Serial.print( F( "." ) );
                Serial.print( this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( u8 )( 1 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ) ] );
                Serial.print( F( "% " ) );
        
                for(u8 ik = filled_vals = 0; ik < confidence_depth; ik++)
                    if( this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ik ) ] )
                        filled_vals++;
                for(u8 ik = tmp_sandbox = 0; ik < confidence_depth; ik++)
                    if( this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ik ) ] & 0x80 ) tmp_sandbox++;
                if( tmp_sandbox > ( filled_vals >> 1 ) )
                    Serial.print( F( "-" ) );
        
                if( ( this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] & 0x7F ) < 10 ) Serial.print( F( " " ) );
                Serial.print( this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] & 0x7F );
                Serial.print( F( "." ) );
                Serial.print( this_Devspec_address->last_valid_data_bytes_from_dht_device[ 3 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ( ( ij + ilvr ) % confidence_depth ) ) ] );
        
                Serial.print( F( "C " ) );
            }
            Serial.print( F( "age in seconds = " ) );
            if( ( ( float )( ( unsigned long )( millis() - this_Devspec_address->timestamp_of_pin_valid_data_millis ) ) / 1000 ) < 10 ) Serial.print( F( " " ) );
            Serial.print( ( float )( ( unsigned long )( millis() - this_Devspec_address->timestamp_of_pin_valid_data_millis ) ) / 1000 );
            if( ( ( float )( ( unsigned long )( millis() - this_Devspec_address->timestamp_of_pin_last_attempted_device_read_millis ) ) / 1000 ) < 10 ) Serial.print( F( " " ) );
            Serial.print( ( float )( ( unsigned long )( millis() - this_Devspec_address->timestamp_of_pin_last_attempted_device_read_millis ) ) / 1000 );
            Serial.print( F( " = last_attempted_read seconds ago. remaining rest: " ) );
            Serial.print( this_Devspec_address->device_busy_resting_this_more_millis );
            Serial.print( F( "mS " ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        }
        newData = false;
    }
}

void loop() 
{
Serial.flush();
recvWithStartEndMarkers();//COURTESY Robin2 ON http://forum.arduino.cc/index.php?topic=396450
showNewData();//COURTESY Robin2 ON http://forum.arduino.cc/index.php?topic=396450
for( u8 devspec_index = 0; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
{
    DEVSPEC* this_Devspec_address = &Devspec[ devspec_index ];
    if( this_Devspec_address->consecutive_read_failures )
    {
        Serial.print( this_Devspec_address->Dpin );
        Serial.print( F( " (DHT" ) );
        if( this_Devspec_address->devprot_index ) Serial.print( F( "22" ) );
        else Serial.print( F( "11" ) );
        Serial.print( F( ") failures " ) );
        Serial.print( this_Devspec_address->consecutive_read_failures );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    }
}
Serial.flush();
delay( 200 );//add 400 for loop execution time, gives us about 600 for loop interval time
}
