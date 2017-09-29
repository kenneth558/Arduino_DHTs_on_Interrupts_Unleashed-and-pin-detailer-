const u8 number_of_device_timers = 10;

//NOTE: THE FOLLOWING GET SHARED BETWEEN ALL ISRDEVS !
volatile Device_Timer device_timer[ number_of_device_timers ]; //next_timer_index_in_series = 255 unless this is to kick off another timer when finished
volatile unsigned long maxISRTIMER0_COMPA_vect_executiontime_micros;//My tests resulted in 48 - 60 uSecs
volatile u8 ISR_index_in_isr = 0;
volatile u8 bit_counter_in_isr;
volatile u8 index_in_PCMSK_of_current_device_within_ISR_plus8;
//volatile u8 pin;
volatile u8 i;

ISR( TIMER0_COMPA_vect )
{
    sei();//Bear in mind atomic needs, but if we don't do this here, we miss PCI data transitions
    TIMSK0 &= 0xFD;//Turns this interrupt off so no re-entry while executing
    for( u8 devspec_index = 0; devspec_index < number_of_devices_found; devspec_index++ )
    {
        if( Devspec[ devspec_index ].device_busy_resting_this_more_millis ) --Devspec[ devspec_index ].device_busy_resting_this_more_millis;
    }

    Device_Timer* this_timer_address;
    long unsigned ISRTIMER0_COMPA_vect_executiontime_micros = micros();
    unsigned long timenowmillis = millis();//A single point of reference to prevent changing during the following
    unsigned long timenowmicros;
    u8 ISR_counter;
    u8 timerindex;
    bool should_be_zero = false;
    bool should_be_one = false;

    ISRSPEC* this_Isrspec_address;
    DEVSPEC* this_Devspec_address;

//Check for device needing to be triggered
    for( timerindex = 0; timerindex < number_of_device_timers; timerindex++ )
        if( device_timer[ timerindex ].on_hold_until_called_by_another_process == 254 )
            if( !( --device_timer[ timerindex ].timeset_millis ) )
            {
                this_timer_address = &device_timer[ timerindex ];
                this_Isrspec_address = &Isrspec[ this_timer_address->which_ISR ];
                this_Devspec_address = &Devspec[ this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR ] - 1 ];
                timenowmicros = micros();//A single point of reference to prevent changing during the following
                timenowmillis = millis();//A single point of reference to prevent changing during the following
                if( !( ( timenowmillis + Devprot[ this_Devspec_address->devprot_index ].millis_rest_length + 20 ) < timenowmillis ) && !( ( timenowmicros + Devprot[ this_Devspec_address->devprot_index ].micros_data_acq_time_max + 100 ) < timenowmicros ) ) //see if not enough time before overflow
                {
                    PCICR |= bit( this_timer_address->which_ISR );//This will not work on xref'd indexes!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
                    *this_Isrspec_address->pcmsk = this_Isrspec_address->mask_by_PCMSK_of_current_device_within_ISR;
                    PCIFR |= bit( this_timer_address->which_ISR );
//                    this_Devspec_address->micros_will_overflow = false;
                    this_Isrspec_address->next_bit_coming_from_dht = 0;
                    this_Devspec_address->device_busy_resting_this_more_millis = Devprot[ this_Devspec_address->devprot_index ].millis_rest_length + ( Devprot[ this_Devspec_address->devprot_index ].micros_data_acq_time_max / 1000 ) + 13;//The 13 is empirical determined
                    this_Devspec_address->timestamp_of_pin_last_attempted_device_read_millis = timenowmillis;
                    this_Devspec_address->start_time_plus_max_acq_time_in_uSecs = timenowmicros + Devprot[ this_Devspec_address->devprot_index ].micros_data_acq_time_max;
                    if( !this_Devspec_address->start_time_plus_max_acq_time_in_uSecs )
                    {
                        this_Devspec_address->start_time_plus_max_acq_time_in_uSecs++; //zero is not a valid value unless device is done sending data train
                    }
                    this_Isrspec_address->pwroftwo = 7;//wish we could place this only in the scope it is used, but that for loop already is doing all it can
                    for( u8 ij = 0; ij < sizeof( this_Isrspec_address->sandbox_bytes ); ij++ )
                        this_Isrspec_address->sandbox_bytes[ ij ] = 0;
                    this_Isrspec_address->interval = 2;
                    *this_Isrspec_address->active_pin_ddr_port_reg_addr &= ~this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;//MAKE THE PIN INTO INPUT
                    *this_Isrspec_address->active_pin_output_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;//MAKE THE PIN HAVE PULLUP
                    this_timer_address->on_hold_until_called_by_another_process = 0;//The code for timer is now available                
                    goto END;//this function must always enable interrupts when ending
                }
            }
//Check for device needing to be read
    this_Isrspec_address = Isrxref->my_isr_addr[ ISR_index_in_isr ];
    if( this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR ] )
    {
        this_Devspec_address = &Devspec[ this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR ] - 1 ];
    }
    else
        goto CONTINUE;

    this_Isrspec_address->mask_by_PCMSK_of_current_device_within_ISR = bit( this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR );
//Make sure this_Devspec_address and this_Isrspec_address point to existing structs
    if( ( !this_Devspec_address->start_time_plus_max_acq_time_in_uSecs ) || ( !this_Isrspec_address->mask_by_PCMSK_of_current_device_within_ISR ) )
    {
        goto CONTINUE;
    }
        
    if( this_Isrspec_address->interval == dht_max_transitions_for_valid_acquisition_stream ) goto STAGE_2;
    if( this_Isrspec_address->interval == 1 ) goto STAGE_3;

    if( ( this_Isrspec_address->next_bit_coming_from_dht != dht_max_transitions_for_valid_acquisition_stream ) && ( micros() > this_Devspec_address->start_time_plus_max_acq_time_in_uSecs ) )
    {
        ++this_Devspec_address->consecutive_read_failures;
//        if( ++this_Devspec_address->consecutive_read_failures > allowed_number_consecutive_read_failures ) this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR ] = 0;//NOT USED without auto-retry
        PCICR &= ~Isrxref->ISR_xref[ ISR_index_in_isr ];//Done with acquisition cycle, turn off PC Iinterrupts for this ISR
        *this_Isrspec_address->active_pin_ddr_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;
        *this_Isrspec_address->active_pin_output_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;//starts the resting
        this_Devspec_address->device_busy_resting_this_more_millis = Devprot[ this_Devspec_address->devprot_index ].millis_rest_length;
        goto DONE_WITH_READ;
    }
    this_Isrspec_address->offset = 0;
    while( true )
    {
        if( this_Isrspec_address->interval < this_Isrspec_address->next_bit_coming_from_dht ) // Starting at bit = 2 must not be protocol-dependent since the values get over-written during the for loop
        {
            cli(); //atomic safety of next line, timestamps are multi-byte and asynchronously written to by the Pin Change ISRs
            if( ( ( long unsigned int )this_Isrspec_address->timestamps[ this_Isrspec_address->interval ] - ( long unsigned int )this_Isrspec_address->timestamps[ this_Isrspec_address->interval - 1 ] ) > ( best_uSec_time_translate + this_Isrspec_address->offset ) )//Was best accuracy, derived empirically on 16 MHz board. Since uSec time values are always multiples of four this number could just as well be anything 100 - 103
            {
                sei();//Allow PC Ints ASAP
                this_Isrspec_address->sandbox_bytes[ ( ( this_Isrspec_address->interval - 2 ) - ( ( this_Isrspec_address->interval - 2 ) % 8 ) ) / 8 ] += ( 1<<this_Isrspec_address->pwroftwo );
            }
            sei();
            this_Isrspec_address->pwroftwo--;
            if( !( ( this_Isrspec_address->interval - 1 ) % 8 ) )
            {
                this_Isrspec_address->pwroftwo = 7;
            }
            this_Isrspec_address->interval++;
        }
        if( this_Isrspec_address->interval != dht_max_transitions_for_valid_acquisition_stream ) 
        {
            goto CONTINUE;
        }
        if( ( u8 )( this_Isrspec_address->sandbox_bytes[ 0 ] + this_Isrspec_address->sandbox_bytes[ 1 ] + this_Isrspec_address->sandbox_bytes[ 2 ] + this_Isrspec_address->sandbox_bytes[ 3 ] ) != this_Isrspec_address->sandbox_bytes[ 4 ] )
        {//FAIL DUE TO CRC or OUT OF BOUNDS
            if( !this_Isrspec_address->offset ) this_Isrspec_address->offset = 4;
            else if( this_Isrspec_address->offset == 4 ) this_Isrspec_address->offset = -4;
            else
            {
ERRD_OUT:;
                this_Devspec_address->consecutive_read_failures++;//array_of_all_devspec_index_plus_1_this_ISR
                if( this_Devspec_address->consecutive_read_successes != consecutive_reads_to_verify_device_type ) this_Devspec_address->consecutive_read_successes = 0;
                this_Isrspec_address->interval = 2;
                goto DONE_WITH_READ;
                goto CONTINUE;//This will try throughout resting period to recalculate timestamps, but must NOT increment the failure count again this same trigger cycle
            }
        }
        else break;
    }
    if( !this_Isrspec_address->sandbox_bytes[ 0 ] && !this_Isrspec_address->sandbox_bytes[ 1 ] && !this_Isrspec_address->sandbox_bytes[ 2 ] && !this_Isrspec_address->sandbox_bytes[ 3 ] && !this_Isrspec_address->sandbox_bytes[ 4 ] )
    {//This condition happens when the device did not have enough rest.  Double the device_busy_resting_this_more_millis time for this Devspec. If only the rest time was the problem the consecutive_read_failures will not accumulate
        this_Devspec_address->consecutive_read_failures++;
        if( this_Devspec_address->consecutive_read_successes != consecutive_reads_to_verify_device_type ) this_Devspec_address->consecutive_read_successes = 0;
        this_Devspec_address->device_busy_resting_this_more_millis += Devprot[ this_Devspec_address->devprot_index ].millis_rest_length;
        if( Devprot[ this_Devspec_address->devprot_index ].millis_rest_length < 5000 ) Devprot[ this_Devspec_address->devprot_index ].millis_rest_length += 3000;
        goto DONE_WITH_READ;
    }
    goto CONTINUE;
STAGE_2:;//*this_Isrspec_address->val_tmp1 is now where for dht11 data 10% RH will evaluate to 10 but dht22 data .1% RH will evaluate to 256, but values above 128 will equate to negatives
//If both values above are within dht11 range, assume dht11 until consecutive_read_successes reaches consecutive_reads_to_verify_device_type
    this_Isrspec_address->interval = 2;
    if( !this_Devspec_address->devprot_index && !this_Isrspec_address->sandbox_bytes[ 1 ] && !this_Isrspec_address->sandbox_bytes[ 3 ] && ( this_Isrspec_address->sandbox_bytes[ 0 ] < 60 ) && ( this_Isrspec_address->sandbox_bytes[ 2 ] < 100 ) )//had to check indices 1 and 3 seperately due to (invalid) negative values still equating to (valid) "less than"
    {;
        //change the devspec devprot index if necessary so the proper rest time is enforced
    }
    else if( this_Devspec_address->devprot_index || ( this_Devspec_address->consecutive_read_successes < consecutive_reads_to_verify_device_type ) )
    {//prep for check for being DHT22
        this_Isrspec_address->interval = 1;
        this_Isrspec_address->sandbox_bytes[ 4 ] = this_Isrspec_address->sandbox_bytes[ 2 ];//To make big endian last for val_tmp2. Index 4 becomes additional holder of dht22 temp sign bit.  this byte will be exactly temp value for dht11, 10x RH value for dht22
        this_Isrspec_address->sandbox_bytes[ 2 ] = this_Isrspec_address->sandbox_bytes[ 0 ];//To make big endian last for val_tmp1.  Overwrites original sign bit leaving index 4 only with sign bit.  this byte will be exactly temp value for dht11, 10x temp value for dht22
//last_valid_data_bytes_from_dht_device //Sometimes this gets executed by mistake, hence no reading faults for DHT11 devices
            //change the devspec devprot index if necessary so the proper rest time is enforced
        this_Isrspec_address->sandbox_bytes[ 0 ] = ( u8 )( ( unsigned short )*this_Isrspec_address->val_tmp1 / 10 );//The integer of RH.  Note that tmp1 includes indices 1 and 2
        this_Isrspec_address->sandbox_bytes[ 1 ] = ( u8 )( *this_Isrspec_address->val_tmp1 - ( ( this_Isrspec_address->sandbox_bytes[ 0 ] * 10 ) ) );//The decimal of RH
        this_Isrspec_address->sandbox_bytes[ 2 ] = this_Isrspec_address->sandbox_bytes[ 4 ];
        this_Isrspec_address->sandbox_bytes[ 4 ] &= 0x7F;
        this_Isrspec_address->sandbox_bytes[ 2 ] = ( ( u8 )( ( unsigned short )*this_Isrspec_address->val_tmp2 / 10 ) ) | ( this_Isrspec_address->sandbox_bytes[ 2 ] & 0x80 );//The integer and sign of temp
        this_Isrspec_address->sandbox_bytes[ 3 ] = ( u8 )( *this_Isrspec_address->val_tmp2 - ( ( this_Isrspec_address->sandbox_bytes[ 2 ] & 0x7F ) * 10 ) );//The decimal of temp
        goto CONTINUE;
STAGE_3:;
        this_Isrspec_address->interval = 2;
        if( !( ( ( ( this_Isrspec_address->sandbox_bytes[ 0 ] < 100 ) && ( this_Isrspec_address->sandbox_bytes[ 2 ] < 100 ) ) || ( ( this_Isrspec_address->sandbox_bytes[ 2 ] & 0x80 ) && ( ( this_Isrspec_address->sandbox_bytes[ 2 ] & 0x7F ) < 41 ) ) ) && !( !this_Devspec_address->devprot_index && ( this_Devspec_address->consecutive_read_successes == consecutive_reads_to_verify_device_type ) ) ) )//had to check indices 1 and 3 seperately due to (invalid) negative values still equating to (valid) "less than"
        {//ALL FAILED
            if( this_Devspec_address->consecutive_read_successes != consecutive_reads_to_verify_device_type )
                this_Devspec_address->devprot_index = 0;
            goto ERRD_OUT;// or?
            goto DONE_WITH_READ;
        }
        this_Devspec_address->devprot_index = 1;
    }
    else
    {
            goto ERRD_OUT;// or?
            goto DONE_WITH_READ;
    }
//Waited until here to replace the previous reading in case we want to add limit checking here
//TODO: do not change devprot_index based on a reading far from others in confidence depth
/*
    for( u8 ij = 0; ij < confidence_depth; ij++ )
    {
//
        if( ( this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] \
            || this_Devspec_address->last_valid_data_bytes_from_dht_device[ 1 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] \
            || this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] \
            || this_Devspec_address->last_valid_data_bytes_from_dht_device[ 3 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) \
            && ( ( ( ( unsigned short )( 10 * this_Isrspec_address->sandbox_bytes[ 0 ] ) + this_Isrspec_address->sandbox_bytes[ 1 ] ) \
            - ( ( unsigned short )( 10 * this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) \
            + this_Devspec_address->last_valid_data_bytes_from_dht_device[ 1 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) > 30 ) \
            && ( ( ( unsigned short )( 10 * this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) \
            + this_Devspec_address->last_valid_data_bytes_from_dht_device[ 1 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) \
            - ( ( unsigned short )( 10 * this_Isrspec_address->sandbox_bytes[ 0 ] ) + this_Isrspec_address->sandbox_bytes[ 1 ] ) > 30 ) ) \
            || \
            ( ( ( ( unsigned short )( 10 * this_Isrspec_address->sandbox_bytes[ 2 ] ) + this_Isrspec_address->sandbox_bytes[ 3 ] ) \
            - ( ( unsigned short )( 10 * this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) \
            + this_Devspec_address->last_valid_data_bytes_from_dht_device[ 3 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) > 30 ) \
            && ( ( ( unsigned short )( 10 * this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) \
            + this_Devspec_address->last_valid_data_bytes_from_dht_device[ 3 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * ij ) ] ) \
            - ( ( unsigned short )( 10 * this_Isrspec_address->sandbox_bytes[ 2 ] ) + this_Isrspec_address->sandbox_bytes[ 3 ] ) > 30 ) ) \
            )
        {
            should_be_zero = false;
            should_be_one = false;
            break;
        }
    }
*/
//Determine what in following logic is keeping valid DHT11 readings from being accepted
//    if( ( this_Isrspec_address->sandbox_bytes[ 0 ] > 100 ) || ( this_Isrspec_address->sandbox_bytes[ 1 ] > 9 ) || ( this_Isrspec_address->sandbox_bytes[ 3 ] > 9 ) ) //add other limit checking to expand dragnet as desired
//    {//dragnet...values arrived at are out of bounds 
//        if( this_Devspec_address->consecutive_read_failures > allowed_number_consecutive_read_failures ) this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR ] = 0;
//        goto CONTINUE;//This will try throughout resting period to recalculate timestamps
//    }
//    else
//    {
    if( this_Devspec_address->consecutive_read_failures )
        this_Devspec_address->consecutive_read_successes = this_Devspec_address->consecutive_read_failures = 0;//Single byte so no atomic concerns
    else if( this_Devspec_address->consecutive_read_successes == consecutive_reads_to_verify_device_type )
    {;
    }
    else
    {
        ++( this_Devspec_address->consecutive_read_successes );
    }
    this_Devspec_address->last_valid_data_bytes_from_dht_device[ ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * this_Devspec_address->index_of_next_valid_readings_sets ) ] = this_Isrspec_address->sandbox_bytes[ 0 ];
    this_Devspec_address->last_valid_data_bytes_from_dht_device[ 1 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * this_Devspec_address->index_of_next_valid_readings_sets ) ] = this_Isrspec_address->sandbox_bytes[ 1 ];
    this_Devspec_address->last_valid_data_bytes_from_dht_device[ 2 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * this_Devspec_address->index_of_next_valid_readings_sets ) ] = this_Isrspec_address->sandbox_bytes[ 2 ];
    this_Devspec_address->last_valid_data_bytes_from_dht_device[ 3 + ( ( sizeof( this_Devspec_address->last_valid_data_bytes_from_dht_device ) / confidence_depth ) * this_Devspec_address->index_of_next_valid_readings_sets ) ] = this_Isrspec_address->sandbox_bytes[ 3 ];
    this_Devspec_address->timestamp_of_pin_valid_data_millis = millis();
    this_Devspec_address->index_of_next_valid_readings_sets = ( this_Devspec_address->index_of_next_valid_readings_sets + 1 ) % confidence_depth;
//    goto DONE_WITH_READ;
//    }
DONE_WITH_READ:;
    this_Devspec_address->start_time_plus_max_acq_time_in_uSecs = 0;//to make the devspec available for triggering again.  Had to wait until no further need to translate timestamps.
CONTINUE:;//goto AFTER_TRIGGER_NEXT_DEVICE;
//Check for device needing to be read, but only on ISRs that are completely and successfully finished translating timestamps
//    if( millis( ) < 20000 ) goto AFTER_TRIGGER_NEXT_DEVICE;//Do not proceed until system is ready
//    if( micros() > ISRTIMER0_COMPA_vect_executiontime_micros + 20 ) ISR_counter = 0;
//    else ISR_counter = ISR_index_in_isr;
    ISR_counter = ISR_index_in_isr;
    for( ; /* hopefully this can be removed*/ true == true; ISR_counter = ( ISR_counter + 1 ) % number_of_ISRs )
    {
        if( ISR_counter != ISR_index_in_isr ) goto AFTER_TRIGGER_NEXT_DEVICE;
        this_Isrspec_address = &Isrspec[ Isrxref->ISR_xref[ ISR_counter ] ];
        if( this_Isrspec_address->start_time_plus_max_acq_time_in_uSecs && *this_Isrspec_address->start_time_plus_max_acq_time_in_uSecs ) continue;//so will skip any isr that points to devices with values in this
        if( micros() > ISRTIMER0_COMPA_vect_executiontime_micros + 70 ) goto AFTER_TRIGGER_NEXT_DEVICE;
        bit_counter_in_isr = this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR;
        index_in_PCMSK_of_current_device_within_ISR_plus8 = bit_counter_in_isr + 8;
    
        for( ; bit_counter_in_isr <= index_in_PCMSK_of_current_device_within_ISR_plus8; bit_counter_in_isr++ )
        {//the bit counter is used to traverse the PCMSK
            if( !this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ bit_counter_in_isr % 8 ] )
                continue;
            this_Devspec_address = &Devspec[ this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ bit_counter_in_isr % 8 ] - 1 ];
            if( this_Devspec_address->device_busy_resting_this_more_millis ) continue;
            if( ( ( bit_counter_in_isr == this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR ) && ( this_Isrspec_address->mask_by_PCMSK_of_current_device_within_ISR ) ) || ( !( this_Isrspec_address->mask_by_PCMSK_of_valid_devices & \
            ( bit( bit_counter_in_isr % 8 ) ) ) ) ) continue;//by continuing we are bypassing this bit
            if( bit_counter_in_isr == this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR && this_Isrspec_address->mask_by_PCMSK_of_current_device_within_ISR && !this_Isrspec_address->start_time_plus_max_acq_time_in_uSecs ) break;//this bypasses ISRs with devices still needing their data processed
            if( this_Isrspec_address->mask_by_PCMSK_of_valid_devices & bit( bit_counter_in_isr % 8 ) )
            {
                if( this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ bit_counter_in_isr % 8 ] ) i = this_Isrspec_address->array_of_all_devspec_index_plus_1_this_ISR[ bit_counter_in_isr % 8 ] - 1;
                else continue;

                for( u8 timer = 0; timer < number_of_device_timers;timer++ )
                {
                    if( !device_timer[ timer ].on_hold_until_called_by_another_process )
                    {
                        bit_counter_in_isr = bit_counter_in_isr % 8;
                        device_timer[ timer ].on_hold_until_called_by_another_process = 255; //stake a claim now then set current device everywhere
                        this_Isrspec_address->mask_by_PCMSK_of_current_device_within_ISR = bit( bit_counter_in_isr );
                        if( !this_Isrspec_address->mask_by_PCMSK_of_current_device_within_ISR ) this_Isrspec_address->mask_by_PCMSK_of_current_device_within_ISR = 1;
                        this_Isrspec_address->index_in_PCMSK_of_current_device_within_ISR = bit_counter_in_isr;
    
                        this_Isrspec_address->active_pin_ddr_port_reg_addr = Devspec[ i ].ddr_port_reg_addr;
                        this_Isrspec_address->active_pin_pin_reg_addr = Devspec[ i ].pin_reg_addr;// Use  active_pin_pin_reg_addr also:
                        this_Isrspec_address->active_pin_output_port_reg_addr = Devspec[ i ].output_port_reg_addr;
    
                        this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR = Devspec[ i ].mask_in_port;
                        this_Isrspec_address->millis_rest_length = Devprot[ Devspec[ i ].devprot_index ].millis_rest_length;//makes for slightly faster access inside the ISR
                        device_timer[ timer ].timeset_millis = ( unsigned long )millis_DHT_MCU_start_signal_bit[ 0 ] + 3;
//                        device_timer[ timer ].function = start_the_DHT;
                        device_timer[ timer ].which_ISR = Isrxref->ISR_xref[ ISR_counter ];//do the cross ref now instead of later
                        this_Isrspec_address->current_device_devspec_structure = &Devspec[ i ];
                        Devspec[ i ].start_time_plus_max_acq_time_in_uSecs = ( unsigned long )-1;//This action prevents this code from activating more devices simultaneously on this same isr
                        this_Isrspec_address->start_time_plus_max_acq_time_in_uSecs = &Devspec[ i ].start_time_plus_max_acq_time_in_uSecs;
                        *Devspec[ i ].ddr_port_reg_addr |= Devspec[ i ].mask_in_port; //Makes doggone sure the pin is an output, but if it wasn't the device probably didn't get its resting time
                        *Devspec[ i ].output_port_reg_addr &= ~Devspec[ i ].mask_in_port; //Assumes pin is already an output, this takes level to a low
                        device_timer[ timer ].on_hold_until_called_by_another_process = 254; //allow activation now
                        bit_counter_in_isr += 8;
                        goto NEXT_ISRDEV;
//                        break;//stop going through the timers
                    }
                }
            }//the if construct that determines if selected bit is eligible to trigger its device
        }//loop to traverse the PCMSK bits
NEXT_ISRDEV:;
    }
AFTER_TRIGGER_NEXT_DEVICE:;
    ISR_index_in_isr = ( ISR_index_in_isr + 1 ) % number_of_ISRs;
END:;
    unsigned long timemicros = micros() - ISRTIMER0_COMPA_vect_executiontime_micros;
    if( maxISRTIMER0_COMPA_vect_executiontime_micros < timemicros )
    {
        cli();
        maxISRTIMER0_COMPA_vect_executiontime_micros = timemicros;
    }
    TIMSK0 |= 2;//basically turns this ISR back on

//sei(); implicit upon return
}


//Pin change interrupts are supported on the following Leonardo/Micro pins - 8,9,10 and [not micro:]11.

//Pin change interrupts are supported on Arduino Mega pins 10,11,12,13,14,15 and analog pins 6 to 15
//Pin change interrupts supported on Arduino Mega are pins 10,11,12,13,14,15 and analog pins 6 to 15 and pins 50,51, 52, 53
//Pin change interrupts are supported on Arduino Mega pins 10,11,12,                        A7,A8,A11,A14,A15 50,51,52,53, 
//What I read from https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/variants/mega/pins_arduino.h -
// D0 = PCINT8,  which need reading PINE and PINJ, D10-13 = PCINT4-7, & D50-53 are PCINT3-0, D14-15 are PCINT10-9, A8-15 are PCINT16-23
// PCINT11-15 not brought out to any pins
// ISR0, 62-69 are ISR2
//Mega do not use ports C and D, Use J and K (and B)

#ifdef PCMSK
    volatile u8* indexwisePCMSK;
//Pin change interrupts are supported on the following Leonardo/Micro pins - 8,9,10 and [not micro:]11.

//Pin change interrupts are supported on Arduino Mega pins 10,11,12,13,14,15 and analog pins 8 to 15
//Pin change interrupts supported on Arduino Mega are pins 10,11,12,13,14,15 and analog pins 8 to 15 and pins 50,51,52,53
//Pin change interrupts are supported on Arduino Mega pins 10,11,12,                        A7,A8,A11,A14,A15 50,51,52,53, 
//What I read from https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/variants/mega/pins_arduino.h -
// D0 = PCINT8, D10-13 = PCINT4-7, & D50-53 are PCINT3-0, D14-15 are PCINT10-9, A8-15 are PCINT16-23
// PCINT11-15 not brought out to any pins
// ISR0, 62-69 are ISR2
//Mega do not use ports C and D, Use J and K (and B)
//
ISR( PCINT_vect )
{//MEGA
    //PINS   A8/D62  A9/D63  A10/D64   A11/D65    A12/D66  A13/D67  A14/D68   A15/D69  //THE D NAMING IS NOT SCREEN PRINTED ON THE BOARD
    //PORTK  1       10      100       1000       10000    100000   1000000   10000000
 //PCIMSK2   1       10      100       1000       10000    100000   1000000   10000000


//UNO:  D0-D7
//PORTD, SAME AS ABOVE
//MSK SAME AS ABOVE
    pin_change_reported_by_ISR = micros();
    sei();
//    [retrieve_ISR_working_index[2].unsparse_this_ISR_index]
    //Determine if we are here for good reason (in data acquisition process for pin in PCMSK)
//    const u16 PROGMEM myPCMSK = &PCMSK;// not really needed

//    changedbit = PINB ^         
        /*
        portOutputRegister( digitalPinToPort( pin ) ) gives a PORTx port (or primary register, if you'd like) address
        
        THE DDR REGISTER ADDRESSES ARE
        portModeRegister( digitalPinToPort( pin ) ) gives a DDRx register address
        
        THE PINx REGISTER ADDRESSES ARE
        portInputRegister( digitalPinToPort( pin ) ) gives a PINx register address
        
        digitalPinToBitMask( pin ) gives the mask on the port for the pin

        digitalPinToPCICRbit( pin ) gives the full mask of PCICR contents with a single bit set for this pin

        digitalPinToPCICR( pin ) gives the address of the only PCICR of the system.  Intended to be useful just to see if pin is served by a PCINT ISR

        digitalPinToPCMSK( pin ) gives the address of the correct PCMSK for this pin.

        digitalPinToPCMSKbit( pin ) gives the full mask of PCMSKx contents with a single bit set for this pin
        */
/*
//digitalWrite(LED_BUILTIN,1);                                                                                                       //high to pin 13 //help us know this line of code executed
//    timestamp_array[NUMBER_OF_DHT_BITS][0] == 9999;
  //  reti();
  if (PIN_MASK_IDENTIFYING_DEVICE_NOW_PROVIDING_BITSTREAM_THIS_PORT[this_port_index_in_order_of_discovery_traversing_pins] == B0) reti();
        if (timestamp_array[NUMBER_OF_DHT_BITS][this_port_index_in_order_of_discovery_traversing_pins] == 9999) reti();                                                          //check for bit over-run error already found
                                                                                                                              //array indices (aka, indexes) hardcoded due to memory shortage for variables, otherwise make this_port_index = 0, 1, or 2, depending on whether PORTD, PORTB or PORTC
                                                                                                                              // We then check bit number corresponding to the pin we're extracting the reading from (2 for testing) by doing a bitwise logical and (the C ‘&’ operator) with 1 << 2.
//digitalWrite(LED_BUILTIN,1);                                                                                                       //high to pin 13 //help us know this line of code executed
    if (PIN_MASK_IDENTIFYING_DEVICE_NOW_PROVIDING_BITSTREAM_THIS_PORT[this_port_index_in_order_of_discovery_traversing_pins] & (byte) this_port_pin_reg ) // ? NEVER/ALWAYS TRUE EVEN THOUGH INTERRUPT HAPPENS! true if active pin is now high
    {
//digitalWrite(LED_BUILTIN,1);                                                                                                       //high to pin 13 //help us know this line of code executed
                                                                                                                              //For noise filtering we could measure the time since last low transition was, at least....something to be determined when the bigger fish are fried        
high_going_timestamp[this_port_index_in_order_of_discovery_traversing_pins] = micros();  //just save this time
        reti();
    }
    else
    {
//digitalWrite(LED_BUILTIN,1);                                                                                                       //high to pin 13 //help us know this line of code executed
        timestamp_array[next_bit_in_dht_bitstream[0]][this_port_index_in_order_of_discovery_traversing_pins] = (micros() - high_going_timestamp[this_port_index_in_order_of_discovery_traversing_pins] );                  //The reason we have to do the math in the ISR is a shortage of memory to store timestamps for later math
        next_bit_in_dht_bitstream[this_port_index_in_order_of_discovery_traversing_pins] = ++next_bit_in_dht_bitstream[this_port_index_in_order_of_discovery_traversing_pins] %(NUMBER_OF_DHT_BITS+1);
    }
//        next_bit_in_dht_bitstream[this_port_index_in_order_of_discovery_traversing_pins] = ++next_bit_in_dht_bitstream[this_port_index_in_order_of_discovery_traversing_pins] %(NUMBER_OF_DHT_BITS-10);
    if ( next_bit_in_dht_bitstream[this_port_index_in_order_of_discovery_traversing_pins] == 0 ) timestamp_array[NUMBER_OF_DHT_BITS][this_port_index_in_order_of_discovery_traversing_pins] = 9999;                                //The chosen way to report a bit over-run error

//        if (timestamp_array[next_bit_in_dht_bitstream[0]][this_port_index_in_order_of_discovery_traversing_pins] >= 16 & timestamp_array[next_bit_in_dht_bitstream[0]][this_port_index_in_order_of_discovery_traversing_pins] <= 29) 
//        {
//            humi = humi << 1;
//              Serial.print(F("0"));
//        }
//        else if (timestamp_array[next_bit_in_dht_bitstream[0]][0] >= 64 & timestamp_array[next_bit_in_dht_bitstream[0]][this_port_index_in_order_of_discovery_traversing_pins] <= 76) 
//        {
//            humi = humi << 1;
//            humi |= 1;
//                Serial.print(F("1"));
//        } 
        //}
// if this finishes out the last bit expected, check the other ports for pending bits.  If none found compute all values received from devices on all ports.  Then determine when to go to next device every port by looking at shortest time difference from last device read attempt.
// set a ctc timer interrupt for this time.  When it happens, start a new data acquisition cycle, disabling the ctc timer.
                                                                                                                                //   FETCH TIME AND ALL PORT READINGS (VERY FIRST READING GOT POPULATED WHEN THE TEST STARTED). TIME AND PORT READINGS ONLY GET STORED IF PORT READINGS ARE DIFFERENT THAN PREVIOUS FOR ELIGIBLE PORTS
                                                                                                                                //Nothing executed until WDT is disabled by last dht device because there is only one WDT.  Too bad.
                                                                                                                                //ISR(portx_falling){ //make a version that is a noise tester only for testing or integrate both versions if we have the margin
                                                                                                                                //read micros() as a time-stamp, store time stamp in next array element, determine if noise in final version
                                                                                                                                //increment next array element modulo 40 if no noise
                                                                                                                                //if this was 40th bit no noise, put readings and millis() time-stamp in the persistent array and check other ports for this being the last device to report.  If last port, disable WDT.
                                                                                                                                ////determine if is within range for either a one or zero in the big analyzer
                                                                                                                                //put 0 in next buffer position if micros() - lastmicros < 80 to 115 or so, else just increase index of buffer bits and update lastmicros
    
}

*/
}
#endif
#if defined (PCMSK0) && defined (PCMSK)
volatile u8* indexwisePCMSK0;
ISR( PCINT0_vect )
{
}
#else
#ifdef PCMSK0
//    volatile unsigned long pin_change_reported_by_ISR0;//obsolete



volatile u8* indexwisePCMSK0;
ISR( PCINT0_vect )
{//APPEARS TO TAKE 84 uSec up to 28 uSec more
    ISRSPEC* this_Isrspec_address = Isrxref->my_isr_addr[ 0 ];
    //if( *this_Isrspec_address->active_pin_pin_reg_addr & this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR )//Rising edge.This for 43 transitions if the first is caught like it should be if Arduino not too busy.  Useful for testing timing margin if can catch that first rising edge! Should be 96 uSec.
    if( !( *this_Isrspec_address->active_pin_pin_reg_addr & this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR ) )//Falling edge.This for 42 transitions, lenient on first one, useful for operational robustness.  Need to adjust dht_max_transitions_for_valid_acquisition_stream for changing this
    {
        this_Isrspec_address->timestamps[ this_Isrspec_address->next_bit_coming_from_dht ] = micros();//adequate resolution, faster speed
        if( ++( this_Isrspec_address->next_bit_coming_from_dht ) == dht_max_transitions_for_valid_acquisition_stream ) //means this was final bit, now wrap it up
//TODO: compare pre increment w/post increment in line above
        {
            sei();
            PCICR &= 0xFE;//Done with acquisition cycle, turn off PC Iinterrupts for this ISR
            *this_Isrspec_address->active_pin_ddr_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;
            *this_Isrspec_address->active_pin_output_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;//starts the resting
//            *this_Isrspec_address->start_time_plus_max_acq_time_in_uSecs = 0;
        }
    }
}

#endif
#endif
#ifdef PCMSK1

    volatile u8* indexwisePCMSK1;
//Pin change interrupts are supported on the following Leonardo/Micro pins - 8,9,10 and [not micro:]11.

//Pin change interrupts are supported on Arduino Mega pins 10,11,12,13,14,15 and analog pins 8 to 15
//Pin change interrupts supported on Arduino Mega are pins 10,11,12,13,14,15 and analog pins 8 to 15 and pins 50,51, 52, 53
//Pin change interrupts are supported on Arduino Mega pins 10,11,12,                        A7,A8,A11,A14,A15 50,51,52,53, 
//What I read from https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/variants/mega/pins_arduino.h -
// D0 = PCINT8, D10-13 = PCINT4-7, & D50-53 are PCINT3-0, D14-15 are PCINT10-9, A8-15 are PCINT16-23
// PCINT11-15 not brought out to any pins
// ISR0, 62-69 are ISR2
//Mega do not use ports C and D, Use J and K (and B)
//

ISR( PCINT1_vect )
{
    ISRSPEC* this_Isrspec_address = Isrxref->my_isr_addr[ 1 ];
    //if( *this_Isrspec_address->active_pin_pin_reg_addr & this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR )//Rising edge.This for 43 transitions if the first is caught like it should be if Arduino not too busy.  Useful for testing timing margin if can catch that first rising edge! Should be 96 uSec.
    if( !( *this_Isrspec_address->active_pin_pin_reg_addr & this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR ) )//Falling edge.This for 42 transitions, lenient on first one, useful for operational robustness.  Need to adjust dht_max_transitions_for_valid_acquisition_stream for changing this
    {
        this_Isrspec_address->timestamps[ this_Isrspec_address->next_bit_coming_from_dht ] = micros();//adequate resolution, faster speed
        if( ++( this_Isrspec_address->next_bit_coming_from_dht ) == dht_max_transitions_for_valid_acquisition_stream ) //means this was final bit, now wrap it up
//TODO: compare pre increment w/post increment in line above
        {
            sei();
            PCICR &= 0xFD;//Done with acquisition cycle, turn off PC Iinterrupts for this ISR
            *this_Isrspec_address->active_pin_ddr_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;
            *this_Isrspec_address->active_pin_output_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;//starts the resting
//            *this_Isrspec_address->start_time_plus_max_acq_time_in_uSecs = 0;
        }
    }
}

#endif
#ifdef PCMSK2
    volatile u8* indexwisePCMSK2;
//Pin change interrupts are supported on the following Leonardo/Micro pins - 8,9,10 and [not micro:]11.

//Pin change interrupts are supported on Arduino Mega pins 10,11,12,13,14,15 and analog pins 8 to 15
//Pin change interrupts supported on Arduino Mega are pins 10,11,12,13,14,15 and analog pins 8 to 15 and pins 50,51,52,53
//Pin change interrupts are supported on Arduino Mega pins 10,11,12,                        A7,A8,A11,A14,A15 50,51,52,53, 
//What I read from https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/variants/mega/pins_arduino.h -
// D0 = PCINT8, D10-13 = PCINT4-7, & D50-53 are PCINT3-0, D14-15 are PCINT10-9, A8-15 are PCINT16-23
// PCINT11-15 not brought out to any pins
// ISR0, 62-69 are ISR2
//Mega do not use ports C and D, Use J and K (and B)
//
ISR( PCINT2_vect )
{
    ISRSPEC* this_Isrspec_address = Isrxref->my_isr_addr[ 2 ];
    //if( *this_Isrspec_address->active_pin_pin_reg_addr & this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR )//Rising edge.This for 43 transitions if the first is caught like it should be if Arduino not too busy.  Useful for testing timing margin if can catch that first rising edge! Should be 96 uSec.
    if( !( *this_Isrspec_address->active_pin_pin_reg_addr & this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR ) )//Falling edge.This for 42 transitions, lenient on first one, useful for operational robustness.  Need to adjust dht_max_transitions_for_valid_acquisition_stream for changing this
    {
        this_Isrspec_address->timestamps[ this_Isrspec_address->next_bit_coming_from_dht ] = micros();//adequate resolution, faster speed
        if( ++( this_Isrspec_address->next_bit_coming_from_dht ) == dht_max_transitions_for_valid_acquisition_stream ) //means this was final bit, now wrap it up
//TODO: compare pre increment w/post increment in line above
        {
            sei();
            PCICR &= 0xFB;//Done with acquisition cycle, turn off PC Iinterrupts for this ISR
            *this_Isrspec_address->active_pin_ddr_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;
            *this_Isrspec_address->active_pin_output_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;//starts the resting
//            *this_Isrspec_address->start_time_plus_max_acq_time_in_uSecs = 0;
        }
    }
}
#endif
#ifdef PCMSK3
    volatile u8* indexwisePCMSK3;
//Pin change interrupts are supported on the following Leonardo/Micro pins - 8,9,10 and [not micro:]11.

//Pin change interrupts are supported on Arduino Mega pins 10,11,12,13,14,15 and analog pins 8 to 15
//Pin change interrupts supported on Arduino Mega are pins 10,11,12,13,14,15 and analog pins 8 to 15 and pins 50,51,52,53
//Pin change interrupts are supported on Arduino Mega pins 10,11,12,                        A7,A8,A11,A14,A15 50,51,52,53, 
//What I read from https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/variants/mega/pins_arduino.h -
// D0 = PCINT8, D10-13 = PCINT4-7, & D50-53 are PCINT3-0, D14-15 are PCINT10-9, A8-15 are PCINT16-23
// PCINT11-15 not brought out to any pins
// ISR0, 62-69 are ISR2
//Mega do not use ports C and D, Use J and K (and B)
//
ISR(PCINT3_vect)
{
    ISRSPEC* this_Isrspec_address = Isrxref->my_isr_addr[ 3 ];
    //if( *this_Isrspec_address->active_pin_pin_reg_addr & this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR )//Rising edge.This for 43 transitions if the first is caught like it should be if Arduino not too busy.  Useful for testing timing margin if can catch that first rising edge! Should be 96 uSec.
    if( !( *this_Isrspec_address->active_pin_pin_reg_addr & this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR ) )//Falling edge.This for 42 transitions, lenient on first one, useful for operational robustness.  Need to adjust dht_max_transitions_for_valid_acquisition_stream for changing this
    {
        this_Isrspec_address->timestamps[ this_Isrspec_address->next_bit_coming_from_dht ] = micros();//adequate resolution, faster speed
        if( ++( this_Isrspec_address->next_bit_coming_from_dht ) == dht_max_transitions_for_valid_acquisition_stream ) //means this was final bit, now wrap it up
//TODO: compare pre increment w/post increment in line above
        {
            sei();
            PCICR &= 0xF7;//Done with acquisition cycle, turn off PC Iinterrupts for this ISR
            *this_Isrspec_address->active_pin_ddr_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;
            *this_Isrspec_address->active_pin_output_port_reg_addr |= this_Isrspec_address->mask_by_port_of_current_device_being_actively_communicated_with_thisISR;//starts the resting
//            *this_Isrspec_address->start_time_plus_max_acq_time_in_uSecs = 0;
        }
    }
}
#endif




