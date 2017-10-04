#include "miscfunctions.h"

void setup() 
{
    unsigned short wincheck = resistor_between_LED_BUILTIN_and_PIN_A0();
    if ( wincheck > 0 && wincheck < 101 ) mswindows = true ; else mswindows = false;
    Isrxref = ( ISRXREF* )malloc( build_from( 0 ) );
    if( !DHT_driver_start() );//Error check here as desired

    Serial.flush();
}

void loop() 
{
demo_input_check();//This is not necessary for the DHT sensors to acquire their readings in the background.  That is happening all the time by virtue of interrupts
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
delay( 200 );//How often you want...
if( ( u8 )ports_string_in_heap_array[ strlen( ports_string_in_heap_array ) + 1 ] != 255 )//this check is not fool-proof, but fairly good. just looks for a null followed by 255 somewhere
{
    Serial.print( F( "Memory is corrupted due to out of memory condition from too many memory sensor structures to support this many sensor.  Expect undesirable operation." ) );
    Serial.print( ( char )10 );if( mswindows ) Serial.print( ( char )13 );
    delay( 12000 );//How long you want...
}
}
