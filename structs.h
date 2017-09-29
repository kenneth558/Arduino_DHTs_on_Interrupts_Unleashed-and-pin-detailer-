/*
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
 * 
 * 
 * 
 * 
 * re-do the structs schema so that all interrupt service routines and start_the_DHT can locate their variable values without calculating
 * Those variables are: 
 *     mask_of_resting_devices_pins in order to quickly know which millis to check without iterating
 *     all DEVSPEC and ISRSPEC and Device_Timer elements
 *     PINSPEC has all these things
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

//structure description:
//each ISRSPEC has a structure specifying:
// - timestamp micros of last portwide device detection action by this port
// - pin mask of current device being actively communicated with on this port
// - high-going micros timestamp and error flag: starts at 0, becomes timestamp for latest rising edge (make 0 and 1=2), holds 1 if bit overrun error, remains 0 if no rising edge
// - low-going time interval micros since high-going for each bit of the current/latest data bit stream acquisition. Don't fill if no high-going timestamp
// - next bit in line in the current/latest data bit stream acquisition
// - const mask of safe to test pins 
// - mask of eligible device pins
// - need to read the associated PIN register and determine what has changed since the last interrupt IF needing to differentiate which pin caused interrupt OR needing to know voltage level of pin
//  address of PCMSK for that ISR 
//  address of device protocol struct in effect
// Intent is to save time by using the pin-to-port/mask/bit/etc functions ahead of time but still try to use small data_seg footprint
/*
The ISR needs to look up the port and pin mask of the pin that caused interrupt because not all boards are like the UNO having ISR serviced pins all on a single mask of a single port.

How this is used: A PICINT ISR is triggered by a pin change. The ISR code reads its own hardcoded PCMSK which has the interrupting pin's bit set in the octet.  The only way then to know the right pin after that is to get its port and bitmask from lookup array.
The possibly-sparse PCMSK can only correlate to the non-sparse index by this xref/lookup table that is freed/[re]built in the dht devices detected function


*/

volatile typedef struct port_specific
{
    u8* this_port_main_reg;
    u8* this_port_pin_reg;
    u8* this_port_mode_reg;                                                                         //Note that we aren't including the modes_original b/c we needed it BEFORE instantiating this structure, and ONLY before.  The modes_original
    u8 this_port_index_in_order_of_discovery_traversing_pins;
    u8 this_port_index_from_core_lookup_alphabetic_order;
    byte mask_of_unsafe_pins_even_to_make_low_Z = 0;
    byte mask_of_unsafe_pins_to_toggle_and_otherwise = 0;
    byte mask_of_safe_pins_to_detect_on = 0; //This should be an inversion of one of the above or both ANDed, which one?
    byte mask_of_real_pins_this_port = B11111111;
    byte mask_of_DHT_devices_this_port = B11111111;
    byte PCINT_pins_mask;//make sure this gets used somwhere.  I don't recall it being in the ISRs, so why do we really need it?
//    byte ( *mask_of_resting_devices_pins )( u8 );
    unsigned long timestamp_of_last_portwide_device_detection_action_by_thisport_micros = 0;
} PORTSPEC;

//When memory is malloc'd for array, one DEVICE_PROTOCOL entry is created in there for each protocol supported.  the elements are filled in at that time from the templates below this one
//When a dht device is identified on a pin, this

volatile typedef struct one_line_device_protocols_supported //Make array of these different types of device protocols, this holds specifics
{
    u8 millis_oldDHT_MCU_start_signal_bit;
    unsigned short millis_rest_length;
    unsigned short micros_data_acq_time_max;
    //0 = no device attached, 255 = yet to be identified, 1-254 indicate various protocol-rest period combinations with 1-3 being generic DHT protocol,  rest period 1 sec, 2 sec, or (3) TBD
    u8 dht_max_transitions_for_valid_acquisition_stream;
//    u8 ( *process_these_timestamps )( unsigned long* ); //, volatile u8*, byte);
//until we support other protocols, just DHT11 and DHT22 are allowed for because we won't know in all the ways that others can differ    
} DEVICE_PROTOCOL;

volatile struct ISR_specific typedef ISRSPEC;
const PROGMEM u8 confidence_depth = 5;

volatile typedef struct device_specific
{//The array of these elements gets built in the device-detection function, which only knows the port, doesn't even know how many devices have been found from previous runs to know the index where to put the ones found
//The indexes for this array need to be able to be reverse looked up from information in ISRSPEC, forward looked up from port masks
    volatile u8 last_valid_data_bytes_from_dht_device[ 4 * confidence_depth ];//here are the last bytes if they were proved valid, multiple sets for higher confidence
    volatile u8 index_of_next_valid_readings_sets;
    volatile unsigned long timestamp_of_pin_valid_data_millis = 0;
    volatile unsigned long timestamp_of_pin_last_attempted_device_read_millis = 0;
//    u16 low_going_interval_micros = 0;
    volatile unsigned long high_going_timestamp_micros = 0;                                                     //Don't corrupt this for error-communications.  The last one from device is needed to set the rest-time timer much later
    volatile u8 next_bit_coming_from_dht = 0;//DEBUG ONLY TODO: REMOVE
    volatile unsigned long device_busy_resting_until_this_system_millis;
    volatile unsigned long start_time_plus_max_acq_time_in_uSecs;//This will get null'd when data is done processing
    volatile bool millis_will_overflow;
    volatile bool micros_will_overflow;
    volatile u8 devprot_index;//to a Device Protocols array element
    volatile u8 portspec_index_for_pin;
    volatile u8 consecutive_read_failures;
    volatile u8 consecutive_read_successes;
    volatile byte mask_in_port;
    volatile u8 Dpin;
    volatile u8* output_port_reg_addr;
    volatile u8* ddr_port_reg_addr;
    volatile u8* pin_reg_addr;
    volatile ISRSPEC* my_isr_addr;
    volatile unsigned long debug_data[ 5 ];
/*
    volatile byte debug_PCICR;
    volatile byte debug_pcmsk;
    volatile unsigned long debug_device_busy_resting_until_this_system_millis;
    volatile unsigned long debug_timestamp_of_pin_last_attempted_device_read_millis;
    volatile unsigned long debug_start_time_plus_max_acq_time_in_uSecs;
    volatile byte debug_ddr_b4;
    volatile byte debug_active_pin_ddr_port_reg_addr;
    volatile byte debug_active_pin_output_port_reg_addr;
*/
} DEVSPEC;

volatile typedef struct PIN_xref
{
    u8 Dpin;
//    u8 dev_index;
//    u8 PIN_xref_dev[ NUM_DIGITAL_PINS ];
} PINXREF;

const u8 PROGMEM as_many_ports_as_a_board_could_ever_have = 26;
volatile typedef struct PORT_xref
{
    u8 PORT_xref[ as_many_ports_as_a_board_could_ever_have + 1 ];//Just remember when sparsing to adjust index one downward from original board port indexes so that PORTA if used would become index 0 even though the index for PORTA starts out as 1
} PORTXREF;

volatile typedef struct DEV_xref
{
    u8 DEV_xref[ NUM_DIGITAL_PINS ];
} DEVXREF;

volatile typedef struct ISR_xref
{
    volatile ISRSPEC* my_isr_addr[ number_of_ISRs ];
    volatile u8 ISR_xref[ number_of_ISRs ];
} ISRXREF;

//Need to develop a better overview understanding of the needed environment here:The is one of these per ISR with the purpose of holding needed items for fastest ISR lookup and use - ISR needs to read port's pin register and AND it on every transition with
struct ISR_specific
{
    volatile u8 sandbox_bytes[ 6 ];
    volatile u8 index_in_PCMSK_of_current_device_within_ISR = 0;  //circular, from 0 to 7, default for no current device = 255
    volatile u8 array_of_all_devspec_index_plus_1_this_ISR[ 8 ];
    volatile DEVSPEC* current_device_devspec_structure;
    volatile u8* active_pin_output_port_reg_addr;
    volatile u8* active_pin_ddr_port_reg_addr;
    volatile u8* active_pin_pin_reg_addr;//This pointer is the address of a portInputRegister of the PORT
    volatile byte mask_by_port_of_current_device_being_actively_communicated_with_thisISR = 0;//will ever only have single bit set at any time to AND against the port's pin register contents, so each bit not guarranteed to have a unique target device.  Use in combo with active_pin_pin_reg_addr
    volatile byte mask_by_PCMSK_of_current_device_within_ISR = 0;
    volatile bool* micros_will_overflow;
    volatile bool* millis_will_overflow;
    volatile unsigned long* start_time_plus_max_acq_time_in_uSecs;//for DHT the max acq time is 4416 uSecs for 40 data bits.   Calculated from the max number of one bits is 25 of the 40 bits plus add 4 uSec for resolution err
    volatile unsigned long* high_going_timestamp_micros  = 0;                                                     //Don't corrupt this for error-communications.  The last one from device is needed to set the rest-time timer much later
    volatile u8 next_bit_coming_from_dht = 0;//set to -1 (255) if acquisition complete? later
    volatile unsigned long timestamps[ ( ( dht_max_transitions_for_valid_acquisition_stream ) * 2 ) + 10 ]; //4 bytes per element x 3 ISRs x 43 
//    volatile unsigned long timestamps[dht_max_transitions_for_valid_acquisition_stream - 1]; //4 bytes per element x 3 ISRs x 43 
//    volatile u8 pin_numbers_in_PCMSK[8];
    volatile byte mask_by_PCMSK_of_real_pins = 0;
    volatile byte mask_of_resting_devices_pins = 0;
    volatile byte mask_by_PCMSK_of_valid_devices = 0; //used to know indexes within run-time-sized array of devices: this is to x-ref to devices array. also used to determine size of array of device characteristics
    volatile u8* pcmsk;
    volatile unsigned short millis_rest_length;
    volatile u8 array_of_all_pinnums_plus_one_this_ISR[ 8 ];//may not need all three of these in final version, never to be sparsed, no savings can be had by sparsing
    volatile u8 array_of_all_devprot_index_this_ISR[ 8 ];//This can only be determined ultimately from port in device discovery -> port mask -> 
    volatile unsigned short* val_tmp1;// = ( unsigned short* )&this_Isrspec_address->sandbox_bytes[ 1 ];
    volatile unsigned short* val_tmp2;// = ( unsigned short* )&this_Isrspec_address->sandbox_bytes[ 3 ];
//    volatile float val_tmp; //With float, the max time seen in isr is 220 uSec
//    volatile float* val_tmp; //With float*, the max time seen in isr is   uSec
    volatile u8 pwroftwo;
    volatile u8 interval = 2;
    volatile signed char offset = 0;
};

//There is one non-struct dynamic-sized array for each ISR, each element is a device-specific struct on one of the ISR's populated PCMSK bits

volatile typedef struct pin_specific
{
    u8 Dpin;
//    u8 duplicate_of_pin;
    byte mask_in_port;
    byte mask_by_PCMSK_in_ISR; //use these two fields to backwards-lookup
    ISRSPEC* ISR_for_pin; //use these two fields to backwards-lookup//or make it the index rather than the address???
    u8 portspec_index_for_pin;
    u8 devspec_index_for_pin;//or make it the index rather than the address???
//    DEVICE_PROTOCOL* device_protocol_for_pin;//dyn-sized array  //should this be here or in device-specific???????????????????
//    Device_Timer* Timer_for_pin; //There may be multiple of these
} PINSPEC;

volatile typedef struct registers_etc_this_dht_populated_pin_no_ISR
{
    PINSPEC* Dpin;
} DHT_NO_ISR;

volatile typedef struct ISR_pin_registers_xref_etc_unsparsed                                            // This is needed to provide which index the interrupt-generating pin is in the jam-packed non-sparse array (only pins with dht devices are members, zero-based, no holes)
{                                                                                                       // One per dht-populated ISR
    byte index_lookup_table[ 8 ];                                                                         // Requires minimum of 24 bits for 8 elements of minimum 3 bit size. Each element represents a pin from the PCMSK of this ISR, with same order as in PCMSK, naturally.
} retrieve_registers_etc_this_dht_pin;                                                                      // Making each element a full byte enables array-style, simplest, fastest access but uses 5 more bytes data_seg memory.  All depends on how desparate we are for data_seg memory
                                                                                                        // Populated as dht devices are discovered, malloc'd when a new ISR is found during that discovery
//typedef void (* ptr_to_function_type) ();

volatile typedef struct D_Timer //piggy-backed to Timer0 match A.  Used for timing device rest period, MCU start bit, device died watchdog, scheduling functions, etc
{//device rest period timers are in pin_spec
    volatile u8 on_hold_until_called_by_another_process = 0; //a number between 0 and 255 indicates a valid index+1 of the other process that will start this later. 0 indicates timer is available for use, 255 indicates timer in use solo
    volatile unsigned long timeset_millis = 0;
//    bool ( *function )( u8 ); //Makes this somewhat re-purposeable
    volatile bool millis_will_overflow;
    volatile bool micros_will_overflow;
    volatile u8 which_ISR;//which_ISR IS AFTER XREF, NOT NEEDING TO GET XREF'D AGAIN
} Device_Timer;
/*
 * Goal is to minimize memory useage by only storing ports, pins, and isrs that have DHT devices on them, let's call it stuffing or sparsing
 * That means the indexes will no longer conform to the original board-defined indexes but rather to found order (pick order) by pin number traverse order
 * The ISR list starts in ISR index order for the purpose of ISR discovery, but must end up different as well.  It won't be in pick order necessarily, 
 * but holes in the index continuum for ISRs must be removed and thereafter ISR indexes will be decreased
 * 
 */