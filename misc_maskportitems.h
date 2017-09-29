/*
#if (NUM_DIGITAL_PINS > 50)//This is for the Mega per https://forum.arduino.cc/index.php?topic=63651.msg583254#msg583254
    //The following defines really belong in the pins_arduino.h for the mega, but the Arduino IDE compiler in Linux Mint is reluctant to incorporate its changes.  Easier done here.
    #define digitalPinToPCICR(p)    ( (((p) >= 10) && ((p) <= 13)) || \
                                      (((p) >= 50) && ((p) <= 53)) || \
                                      (((p) >= 62) && ((p) <= 69)) || \
                                      (((p) == 0)  || ((p) == 14) || ((p) == 15)) ? (&PCICR) : ((u8 *)0) )
    
    #define digitalPinToPCICRbit(p) ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 50) && ((p) <= 53)) ? 0 : \
                                    ( (((p) >= 62) && ((p) <= 69)) ? 2 : \
                                    ( (((p) == 0) || ((p) == 14) || ((p) == 15)) ? 1 : \
                                    255 ) )) //How else would we differentiate between a pin on ISR0 with mask B00000001 and a pin not supported by any ISR?  This would have been a second real good reason for digitalPinToPCMSKbit to return a real mask with a bit set in it instead of just returning a bit position that could be zero to indicate B00000001 like it does.
    
    #define digitalPinToPCMSK(p)    ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 50) && ((p) <= 53)) ? (&PCMSK0) : \
                                    ( (((p) == 0 || (p) == 14 || (p) == 15) ? (&PCMSK1) : \
                                    ( (((p) >= 62) && ((p) <= 69)) ? (&PCMSK2) : \
                                    ((u8 *)0) ) )
    
    #define digitalPinToPCMSKbit(p) ( (((p) >= 10) && ((p) <= 13)) ? ((p) - 6) : \
                                    ( ((p) == 50) ? 3 : \
                                    ( ((p) == 51) || ((p) == 14) ? 2 : \
                                    ( ((p) == 52) || ((p) == 15) ? 1 : \
                                    ( ((p) == 53) || ((p) == 0) ? 0 : \
                                    ( (((p) >= 62) && ((p) <= 69)) ? ((p) - 62) : \
                                    0 ) ) ) ) ) )
#endif  //Not sure if we want this here, re-evaluate it when we have mega board, etc. to test  
*/
volatile struct ISR_specific typedef ISRSPEC;
#ifdef PCMSK
    #ifdef PCMSK0 //The purpose of this entry is for rationale only, never expected to materialize
        const u8 PROGMEM number_of_ISRs = 2;
        ISRSPEC* isrspec_addr0;
        ISRSPEC* isrspec_addr1;
    #else
        const u8 PROGMEM number_of_ISRs = 1;
        ISRSPEC* isrspec_addr0;
    #endif
#else
    #if defined (PCMSK) || defined (PCMSK0) || defined (PCMSK1) || defined (PCMSK2) || defined (PCMSK3)
        #ifdef PCMSK3
            const u8 PROGMEM number_of_ISRs = 4;
                ISRSPEC* isrspec_addr0;
                ISRSPEC* isrspec_addr1;
                ISRSPEC* isrspec_addr2;
                ISRSPEC* isrspec_addr3;
        #else
            #ifdef PCMSK2
                const u8 PROGMEM number_of_ISRs = 3;
                ISRSPEC* isrspec_addr0;
                ISRSPEC* isrspec_addr1;
                ISRSPEC* isrspec_addr2;
            #else
                #ifdef PCMSK1
                    const u8 PROGMEM number_of_ISRs = 2;
                    ISRSPEC* isrspec_addr0;
                    ISRSPEC* isrspec_addr1;
                #else
                    #ifdef PCMSK0
                        const u8 PROGMEM number_of_ISRs = 1;
                        ISRSPEC* isrspec_addr0;
                    #endif
                #endif
            #endif
        #endif
    #else
        const u8 PROGMEM number_of_ISRs = 0;
        ISRSPEC* isrspec_addr0;
    #endif
#endif
