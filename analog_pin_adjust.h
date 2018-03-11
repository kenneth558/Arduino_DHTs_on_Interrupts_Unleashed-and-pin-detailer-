#ifdef NUM_ANALOG_INPUTS
    #ifndef PIN_A0
//if compiler errors here, you have a board the does not define its analog pins per de facto standards
//Just comment out ONLY the lines that cause compiler to error, leaving the lines prior to those intact
//Then save the modified file and re-compile
        #define PIN_A0 A0
        #define PIN_A1 A1
        #define PIN_A2 A2
        #define PIN_A3 A3
        #define PIN_A4 A4
        #define PIN_A5 A5
        #define PIN_A6 A6
        #define PIN_A7 A7
        #define PIN_A8 A8
        #define PIN_A9 A9
        #define PIN_A10 A10
        #define PIN_A11 A11
        #define PIN_A12 A12
        #define PIN_A13 A13
        #define PIN_A14 A14
        #define PIN_A15 A15
        #define PIN_A16 A16
        #define PIN_A17 A17
        #define PIN_A18 A18
        #define PIN_A19 A19
        #define PIN_A20 A20
    #endif
    #if defined ( PIN_A20 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A20
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20 };
    #endif
    #if defined ( PIN_A19 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A19
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19 };
    #endif
    #if defined ( PIN_A18 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A18
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18 };
    #endif
    #if defined ( PIN_A17 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A17
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17 };
    #endif
    #if defined ( PIN_A16 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A16
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16 };
    #endif
    #if defined ( PIN_A15 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A15
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15 };
    #endif
    #if defined ( PIN_A14 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A14
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14 };
    #endif
    #if defined ( PIN_A13 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A13
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13 };
    #endif
    #if defined ( PIN_A12 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A12
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12 };
    #endif
    #if defined ( PIN_A11 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A11
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11 };
    #endif
    #if defined ( PIN_A10 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A10
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10 };
    #endif
    #if defined ( PIN_A9 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A9
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9 };
    #endif
    #if defined ( PIN_A8 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A8
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8 };
    #endif
    #if defined ( PIN_A7 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A7
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
    #endif
    #if defined ( PIN_A6 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A6
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5, A6 };
    #endif
    #if defined ( PIN_A5 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A5
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4, A5 };
    #endif
    #if defined ( PIN_A4 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A4
        u8 analog_pin_list[] = { A0, A1, A2, A3, A4 };
    #endif
    #if defined ( PIN_A3 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A3
        u8 analog_pin_list[] = { A0, A1, A2, A3 };
    #endif
    #if defined ( PIN_A2 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A2
        u8 analog_pin_list[] = { A0, A1, A2 };
    #endif
    #if defined ( PIN_A1 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A1
        u8 analog_pin_list[] = { A0, A1 };
    #endif
    #if defined ( PIN_A0 ) && not defined ( PIN_Amax )
        #define PIN_Amax PIN_A0
        u8 analog_pin_list[] = { A0 };
    #endif
#endif
