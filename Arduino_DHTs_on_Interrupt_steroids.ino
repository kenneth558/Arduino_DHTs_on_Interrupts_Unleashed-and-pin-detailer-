#include "miscfunctions.h"
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
        u8 pin;
        if( ( receivedChars[ 0 ] == 'a' ) || ( receivedChars[ 0 ] == 'A' ) )
        {
            for( u8 devspec_index = 0; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
            {
                this_Devspec_address = &Devspec[ devspec_index ];
                filled_vals = 0;
                ilvr = this_Devspec_address->index_of_next_valid_readings_sets;
                Serial.print( F( "At location #" ) );
                if( devspec_index < 10 ) Serial.print( F( " " ) );
                Serial.print( devspec_index );
                if( this_Devspec_address->Dpin < 10 ) Serial.print( F( " " ) );
                Serial.print( F( " is pin D" ) );
                Serial.print( this_Devspec_address->Dpin );
                Serial.print( F( " (DHT" ) );
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
        else if( ( ( receivedChars[ 0 ] > 47 ) && ( receivedChars[ 0 ] < 58 ) ) && ( atoi( receivedChars ) < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ) ) )
        {
            this_Devspec_address = &Devspec[ atoi( receivedChars ) ];
SHOW_A_SENSOR:;
            filled_vals = 0;
            ilvr = this_Devspec_address->index_of_next_valid_readings_sets;
            Serial.print( F( "At location #" ) );
            if( atoi( receivedChars ) < 10 ) Serial.print( F( " " ) );
            Serial.print( atoi( receivedChars ) );
            if( this_Devspec_address->Dpin < 10 ) Serial.print( F( " " ) );
            Serial.print( F( " is pin D" ) );
            Serial.print( this_Devspec_address->Dpin );
            Serial.print( F( " (DHT" ) );
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
        else if( ( ( receivedChars[ 0 ] == 'd' ) || ( receivedChars[ 0 ] == 'D' ) ) )// && ( atoi( &receivedChars[ 1 ] ) < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ) ) )
        {
            for( u8 devspec_index = 0; devspec_index < ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ); devspec_index++ )
            {
                pin = atoi( &receivedChars[ 1 ] );
                if( Devspec[ devspec_index ].Dpin == pin )
                {
                    this_Devspec_address = &Devspec[ devspec_index ];
                    goto SHOW_A_SENSOR;
                }
            }
            Serial.print( F( "That pin has no DHT sensor on it" ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        }
        else
        {
            Serial.print( F( "Each DHT connected is using " ) );
            Serial.print(  sizeof( Devspec [ 0 ] ) );
            Serial.print( F( " bytes in memory" ) );
            Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
        }
        newData = false;
    }
}

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
    Serial.flush();
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
Serial.print( freeRam() );
Serial.print( F( " bytes of free RAM for variables" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
if( !( ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ) ) )
{
    Serial.print( F( "No DHT devices were detected, so the following statements are null and void:" ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
}
Serial.print( F( "Factory sketch functions: enter the letter A or an index number between 0 and " ) );
Serial.print( ( ( ( long unsigned int )ports_string_in_heap_array - ( long unsigned int )Devspec ) / sizeof( DEVSPEC ) ) - 1 );
Serial.print( F( " or the" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "letter D immediately followed by (no space between) a digital pin number with your entire" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "entry enclosed between these two characters: < and >.  Entering the letter A so enclosed" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "will list all DHT devices each with its last " ) );
Serial.print( confidence_depth );
Serial.print( F( " values obtained.  Entering the index" ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "number or D and pin number of any selected device will do the same for the one device only." ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.print( F( "Reading errors that occur are displayed asynchronously while the void loop() sees them." ) );
Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
Serial.flush();
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
        Serial.print( F( "Pin D" ) );
        Serial.print( this_Devspec_address->Dpin );
        Serial.print( F( " " ) );
        print_analog_if_exists( this_Devspec_address->Dpin );
        Serial.print( F( " (DHT" ) );
        if( this_Devspec_address->devprot_index ) Serial.print( F( "22" ) );
        else Serial.print( F( "11" ) );
        Serial.print( F( ") showing " ) );
        Serial.print( this_Devspec_address->consecutive_read_failures );
        Serial.print( F( " failures " ) );
        Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    }
}
Serial.flush();
delay( 200 );//add 400 for loop execution time, gives us about 600 for loop interval time
}
