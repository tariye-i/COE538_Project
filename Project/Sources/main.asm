;*****************************************************************************
; EEbot - Complete Maze Navigation Program
;*****************************************************************************
; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry                  ; for absolute assembly: mark this as application entry point          
; Include derivative-specific definitions 
		INCLUDE 'derivative.inc'     ; Use CodeWarrior's include file
;=============================================================================
; CONSTANTS / EQUATES SECTION
;=============================================================================
LCD_DAT         EQU   PORTB                     ; LCD data port, bits - PB7,...,PB0
LCD_CNTR        EQU   PTJ                       ; LCD control port, bits - PJ6(RS),PJ7(E)
LCD_E           EQU   $80                       ; LCD E-signal pin
LCD_RS          EQU   $40                       ; LCD RS-signal pin                                                                                                        
;=============================================================================
; GUIDER SECTION(VARIES FOR EACH EEBOT, CHANGE. MUST BE CALIBRATED!) 
;=============================================================================
; Motor Speed Constants
MOTOR_MIN       EQU 64     ; 25% duty
MOTOR_SLOW      EQU 102    ; 40% duty
MOTOR_MED       EQU 153    ; 60% duty
MOTOR_FAST      EQU 204    ; 80% duty

; Sensor Thresholds (CALIBRATE THESE!)
BOW_THRESH       EQU $99              ; SENSOR A
PORT_THRESH      EQU $99              ; SENSOR B
MIDD_THRESH      EQU $99              ; SENSOR C
STARBD_THRESH    EQU $99              ; SENSOR D
;line following tracking/ tuning
; +---------------------------------------------------------+
; ¦  E covered (E dark, F light) = $3C ? Robot FAR LEFT     ¦
; ¦  F covered (E light, F dark) = $C4 ? Robot FAR RIGHT    ¦
; ¦  Centered (E=F resistance)   = $80 ? Robot CENTERED     ¦
; +---------------------------------------------------------+
ELINE_THRESH        EQU $60    ; (3C + 80)/2 = 5E ~ 60      
RLINE_THRESH        EQU $A0    ; (C4 + 80)/2 = A2 ~ A0
LINE_CENTER         EQU $80     ; Centered ($3C + $C4) / 2 = 80
LINE_DEADBAND       EQU $08         ; Â±8 counts acceptable
;-----------------------------------------------------------------------------
; WHEEL COUNTS
;-----------------------------------------------------------------------------
COUNTS_90_DEG   EQU 26               ; 90Â° turn
COUNTS_180_DEG  EQU 52               ; 180Â° turn
REVERSE_DIST    EQU 100              ; Backup distance
; State Machine States
START          EQU 0               ; CHANGE NAME TO START
FLW            EQU 1
JUNCT          EQU 2        
TURN           EQU 3
COL            EQU 4
REV            EQU 5
DONE           EQU 6
BCKTRK         EQU 7
; Maze Constants
MAX_INTERSECTIONS   EQU 7
MODE_EXPLORING      EQU 0
MODE_RETRACING      EQU 1
; Directions / TURNS
NORTH               EQU 0
EAST                EQU 1
SOUTH               EQU 2
WEST                EQU 3
; Turn Types
TURN_RIGHT          EQU 0
TURN_STRAIGHT       EQU 1
TURN_LEFT           EQU 2
TURN_180            EQU 3
;-----------------------------------------------------------------------------
; PATTERN TYPES (simplified)
;-----------------------------------------------------------------------------
PATTERN_NONE    EQU 0                  ; ENDLINE         ; No line detected
PATTERN_LINE    EQU 1                ; Straight line (A, C)
PATTERN_LEFT    EQU 2                ; Left turn available (B, C), LTURN
PATTERN_RIGHT   EQU 3                ; Right turn available (C, D), RTURN
PATTERN_T       EQU 4                ; T-junction (B, C, D)
PATTERN_CROSS   EQU 5                ; Cross junction (A, B, C, D), CRSJUNC
; PATH ORDER
FIRST_PATH          EQU  0     ;at intersections and junctions
SECOND_PATH         EQU  1     ;at intersections and junctions
;=============================================================================
; RAM VARIABLES
;=============================================================================
; variable/data section
                    ORG $3800

TOF_COUNTER         dc.b 0          ; The timer, incremented at 23Hz
TEN_THOUS           ds.b 1          ; 10,000 digit
THOUSANDS           ds.b 1          ; 1,000 digit
HUNDREDS            ds.b 1          ; 100 digit
TENS                ds.b 1          ; 10 digit
UNITS               ds.b 1           ; 1 digit
NO_BLANK            ds.b 1          ; Used in ï¿½leading zeroï¿½ blanking by BCD2ASC
BCD_SPARE           RMB 10          ; Extra space for decimal point and string terminator
CURRENT_GUIDER_VALUE  DS.B 1
; State & Control
robot_state     DS.B 8               ; Current state
robot_mode      DS.B 1               ; Exploring or retracing
turn_type       DS.B 1               ; Current turn being executed
;-----------------------------------------------------------------------------
; Maze Learning
;-----------------------------------------------------------------------------
current_direction     DS.B 1         ; Current heading (N/E/S/W)
current_intersection  DS.B 1         ; 0-6
path_stack_ptr        DS.B 1
maze_solution         RMB MAX_INTERSECTIONS
tried_direction       RMB MAX_INTERSECTIONS
intersection_solved   RMB MAX_INTERSECTIONS
path_stack            RMB MAX_INTERSECTIONS
;-----------------------------------------------------------------------------
; Sensor Data
;-----------------------------------------------------------------------------
sensor_pattern  DS.B 1               ; Binary: bits ABCD (5432)
detected_pattern DS.B 1              ; Pattern type (PATTERN_*)
;-----------------------------------------------------------------------------
; E-F Zone Flags (mutually exclusive)
;-----------------------------------------------------------------------------
zone_left       DS.B 1               ; 1 = robot left of line
zone_right      DS.B 1               ; 1 = robot right of line
zone_centered   DS.B 1               ; 1 = robot centered
; Motor Control
port_motor_speed    DS.B 1
stbd_motor_speed    DS.B 1
target_counts    DS.W 1              ; Target rotation count for turns
; Wheel Counters , 16 Bits to avoid overflow, 2-byte initializated to $0000
rotation_count_port DS.W 1
rotation_count_stbd DS.W 1
; Temporaries
temp_a              DS.B 1

RETURN_PATH      DC.B  0                         ; RETURN (TRUE = 1, FALSE = 0)

;FOR PATTERN DETECTION
LINE_SENSOR     DC.B  $0                        ; E-F, (LINE ) Storage for guider sensor readings
BOW_SENSOR      DC.B  $0                        ; A, (FRONT) Initialized to test values
PORT_SENSOR     DC.B  $0                        ; B, (LEFT )
MIDD_SENSOR     DC.B  $0                        ; C, (MIDDLE)
STARBD_SENSOR   DC.B  $0                        ; D, (RIGHT)
SENSOR_NUM      DC.B  1                        ; Sensor number for reading loop
;=============================================================================
; PROGRAM ENTRY POINT / CODE SECTION  
;=============================================================================
                ORG $4000
Entry:
_Startup:
                LDS  #$4000                 ; Initialize stack pointer
            ; Initialize Hardware
                JSR  INIT_PORTS
                JSR   initAD                    ; Initialize ATD converter            
                JSR   initLCD                   ; Initialize the LCD                        
                JSR   clrLCD                    ; Clear LCD & home cursor   
                JSR  INIT_TCNT
            ; Initialize Variables
                 JSR  INIT_MAZE_DATA        
            ; Safety: Motors off
                JSR  STOP_MOTORS                                              
            ; Set initial state
                LDAA #START
                STAA robot_state
            
                CLI                         ; Enable interrupts
            
                LDX   #msg1                     ; Display msg1                              N
                JSR   putsLCD                   ; ""                                        |
                                                ;                                           |
                LDAA  #$8A                      ; Move LCD cursor to the end of msg1        |
                JSR   cmd2LCD                   ; ""                                        |
                LDX   #msg2                     ; Display msg2                              |
                JSR   putsLCD                   ; ""                                        |
                                                ;                                           |
                LDAA  #$C0                      ; Move LCD cursor to the 2nd row            |
                JSR   cmd2LCD                   ; ""                                        |
                LDX   #msg3                     ; Display msg3                              |
                JSR   putsLCD                   ; ""                                        |
                                                ;                                           |
                LDAA  #$C7                      ; Move LCD cursor to the end of msg3        |
                JSR   cmd2LCD                   ; ""                                        |
                LDX   #msg4                     ; Display msg4                              |
                JSR   putsLCD                   ; ""                            

        MainLoop:
                JSR   READ_ALL_SENSORS      ; 1. Read all raw sensors
                JSR   CHECK_BUMPERS          ; safety flags
                
                LDAA  robot_state        ; 2. State Machine Dispatcher
                JSR   DISPATCHER

                JSR  UPDT_READING      ; Read all sensors, detect patterns
                JSR  UPDT_DISP      ; Show status (not every loop in real code)
              
                BRA   MainLoop
;*******************************************************************
; data section
;*******************************************************************
msg1        dc.b "Battery volt ",0      
msg2        dc.b "State ",0
msg3        dc.b "Sensor ",0
msg4        dc.b "Bumper ",0

tab         dc.b "START",0                 
            dc.b "FLW",0
            dc.b "JUNCT",0
            dc.b "TURN",0
            dc.b "COL",0
            dc.b "REV",0
            dc.b "DONE",0
            dc.b "BCKTRK",0

; Turn Result Table: [current_dir * 3 + turn_type] new_direction
turn_result_table:
            DC.B EAST, NORTH, WEST      ; From NORTH: RIGHT, STRAIGHT, LEFT
            DC.B SOUTH, EAST, NORTH     ; From EAST
            DC.B WEST, SOUTH, EAST      ; From SOUTH
            DC.B NORTH, WEST, SOUTH     ; From WEST     
;subroutines section
;********************************************************************************************
;* NEW STATES DISPATCHER / STATE MACHINE                                                                        
;*******************************************************************
DISPATCHER      CMPA #START       
                BNE  NOT_START
                JSR  START_ST
                RTS
          
NOT_START       CMPA #FLW
                BNE  NOT_FLW
                JSR  FLW_ST
                RTS          

NOT_FLW        CMPA #JUNCT
                BNE  NOT_JUNCT
                JSR  JUNCT_ST
                RTS

NOT_JUNCT       CMPA #TURN
                BNE  NOT_TURN
                JSR  TURN_ST
                RTS

NOT_TURN        CMPA #COL
                BNE  NOT_COL
                JSR  COL_ST
                RTS     

NOT_COL         CMPA #DONE
                BNE  NOT_DONE
                JSR  DONE_ST
                RTS

NOT_DONE        CMPA #BCKTRK
                BNE  NOT_BCKTRK
                JSR  BCKTRK_ST
                RTS

NOT_BCKTRK      JSR STOP_MOTORS
                RTS

DISPATCHER_END  RTS 
; START STATE: Wait for bow bumper to start
START_ST            
            BRCLR PORTAD0, $04, START_FWD_BUMP
            RTS
START_FWD_BUMP             
            MOVB  #FLW, robot_state       ; Set state to FOLLOW

            JSR   FLW_INIT                       ; Initialize FOLLOW state             
            CLRA                    ; Both forward
            CLRB
            JSR  SET_MOTOR_DIRECTIONS
            
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            RTS

; STATE: FOLLOW - Track line and detect junctions
FLW_ST      ; Check for junction
            LDAA detected_pattern
            CMPA #PATTERN_LINE
            BEQ  FOLLOW_TRACK        ; Just a line, keep following
            
            CMPA #PATTERN_NONE
            BEQ  FOLLOW_TRACK        ; Lost line, keep trying       
            ; Pattern detected - it's a junction!
            MOVB  #JUNCT, robot_state 
            JSR  STOP_MOTORS
            RTS

FOLLOW_TRACK        
            JSR  LINE_FOLLOW_CONTROL            ; Do line following control
            RTS
;*****************************************************************************
;* STATE: JUNCTION - Decision point, choose turn direction
;*****************************************************************************
JUNCT_ST

            ; Increment intersection counter
            INC  current_intersection
            
            ; Check mode
            LDAA robot_mode
            CMPA #MODE_RETRACING
            BEQ  JUNCTION_RETRACE
            
            ;----- EXPLORING MODE -----     
            LDAA current_intersection   ; First check: Do we already know the answer?
            JSR  CHECK_IF_SOLVED
            CMPA #1
            BEQ  JUNCTION_USE_SOLUTION   ; We learned this already!
            
            JSR  PUSH_INTERSECTION       ; Not solved yet - explore new path
           
            LDAA detected_pattern        ; Determine available turns based on pattern

            CMPA #PATTERN_LEFT
            BEQ  JUNCTION_TRY_LEFT
            
            CMPA #PATTERN_RIGHT
            BEQ  JUNCTION_TRY_RIGHT
            
            CMPA #PATTERN_T
            BEQ  JUNCTION_TRY_LEFT    ; Default to left at T
            
            CMPA #PATTERN_CROSS
            BEQ  JUNCTION_TRY_LEFT    ; Default to left at cross
            ; Unknown - go straight
            MOVB  #FLW, robot_state
            RTS

JUNCTION_TRY_LEFT
            ; Record that we're trying left first
            LDAA current_intersection
            LDAB #TURN_LEFT
            JSR  RECORD_TRIED_DIRECTION 
            ; Setup left turn
            LDAA #TURN_LEFT
            STAA turn_type
            MOVB  #TURN, robot_state
            JSR  TRN_INIT
            RTS

JUNCTION_TRY_RIGHT
            ; Record that we're trying right first
            LDAA current_intersection
            LDAB #TURN_RIGHT
            JSR  RECORD_TRIED_DIRECTION
            
            ; Setup right turn
            LDAA #TURN_RIGHT
            STAA turn_type
            MOVB  #TURN, robot_state
            JSR  TRN_INIT
            RTS    

JUNCTION_USE_SOLUTION
            ; We already know the correct path from previous learning! ; B already contains the solution from CHECK_IF_SOLVED
            STAB turn_type
            MOVB  #TURN, robot_state
            JSR  TRN_INIT
            RTS            
            ;----- RETRACING MODE -----
JUNCTION_RETRACE
            ; Read stored solution
            LDAA current_intersection
            LDX  #maze_solution
            LDAB A,X
            STAB turn_type
            
            MOVB  #TURN, robot_state
            JSR  TRN_INIT
            RTS
;*****************************************************************************
;* STATE: TURN - Execute turn maneuver
;*****************************************************************************
TURN_ST
            LDAA turn_type              ; Check if turn complete
            
            CMPA #TURN_LEFT
            BEQ  TURN_CHECK_LEFT
            
            CMPA #TURN_RIGHT
            BEQ  TURN_CHECK_RIGHT
            
            CMPA #TURN_180
            BEQ  TURN_CHECK_180 
            ; Unknown turn - abort
            JSR  STOP_MOTORS
            MOVB #FLW, robot_state
            RTS

TURN_CHECK_LEFT
            LDD  rotation_count_port
            CPD  #COUNTS_90_DEG
            BLO  TURN_CONTINUE
            BRA  TURN_DONE

TURN_CHECK_RIGHT
            LDD  rotation_count_stbd
            CPD  #COUNTS_90_DEG
            BLO  TURN_CONTINUE
            BRA  TURN_DONE

TURN_CHECK_180
            LDD  rotation_count_port
            CPD  #COUNTS_180_DEG
            BLO  TURN_CONTINUE
            BRA  TURN_DONE

TURN_CONTINUE
            RTS                      ; Still turning

TURN_DONE
            JSR  STOP_MOTORS     
            ; Update direction
            JSR  UPDATE_DIRECTION          
            ; Return to following
            MOVB #FLW, robot_state
            
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            
            RTS          
;-----------------------------------------------------------------------------
; STATE: COLLISION - Hit dead end
;-----------------------------------------------------------------------------
COL_ST
        ; **KEY LEARNING STEP**: Record that we chose wrong path
            JSR  RECORD_CORRECTION   ; Mark opposite direction as correct!
            ; Pop back to previous intersection
            JSR  POP_INTERSECTION
            ; Start reversing
            JSR  STOP_MOTORS
            ; Both motors reverse
            BSET  PORTA,%00000001       ; PA0=1, PA1=1 (both REV)
            BSET  PORTA,%00000010

            ; Clear counters
            LDD #$0000
            STD rotation_count_port

            LDD #$0000
            STD rotation_count_stbd
            
            ; Start reversing
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            
            MOVB #REV, robot_state
            RTS
;*****************************************************************************
;* STATE: REVERSE - Executing 180-degree turn after collision
;*****************************************************************************
REV_ST
            ; Check if backed up enough
            LDD  rotation_count_port
            CPD  #REVERSE_DIST
            BLO  REVERSE_CONTINUE
            
            ; Reverse complete - do 180 turn
            JSR  STOP_MOTORS
            
            LDAA #TURN_180
            STAA turn_type
            MOVB #TURN, robot_state
            JSR  TRN_INIT
            RTS

REVERSE_CONTINUE
            RTS
;*****************************************************************************
;* STATE: BACKTRACK - Retracing path to start
;*****************************************************************************
BCKTRK_ST
            ;----- Check stern bumper (reached start), MODIFY THIS TO BACKTRACK WHEN STERN IS PRESSED -----
            BRCLR PORTAD0,$08,BCKTRK_DONE
            
            ; Check for junctions
            LDAA detected_pattern
            CMPA #PATTERN_LINE
            BEQ  BCKTRACK_FOLLOW
            
            CMPA #PATTERN_NONE
            BEQ  BCKTRACK_FOLLOW 
            ; Junction - use solution
            JSR  POP_INTERSECTION
            LDAA current_intersection
            LDX  #maze_solution
            LDAB A,X
            STAB turn_type
            
            MOVB #TURN, robot_state
            JSR  TRN_INIT
            RTS
 
BCKTRACK_FOLLOW
            JSR  LINE_FOLLOW_CONTROL
            RTS 

BCKTRK_DONE
            MOVB #DONE, robot_state
            JSR  STOP_MOTORS
            RTS
            
DONE_ST 
            JSR  STOP_MOTORS                  ; Keep motors stopped
            RTS                  
;*****************************************************************************
;* STATE INITIALIZATION ROUTINES
;*****************************************************************************
FLW_INIT                        ; Initialize FOLLOW state, Ensure motors are on, going forward
            BCLR  PORTA,%00000001   ; direction (1=ON. 0=OFF) ;port
            BCLR  PORTA,%00000010   ; star
            ; turn on both motors
            BSET  PTT,%00010000     ; speed (1=ON. 0=OFF) ;port
            BSET  PTT,%00100000     ; star
            ; Start motors at medium speed
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            
            ; Reset wheel counters
            ; Clear counters
            LDD #$0000
            STD rotation_count_port

            LDD #$0000
            STD rotation_count_stbd
            
            RTS

JUNCT_INIT                      ; Initialize JUNCTION state
            ; Stop motors temporarily while deciding
            JSR  STOP_MOTORS
            
            ; Small delay to stabilize at junction
            LDY  #1000
            JSR  del_50us
            
            RTS

TRN_INIT
            JSR  STOP_MOTORS

            ; Clear counters
            LDD #$0000
            STD rotation_count_port

            LDD #$0000
            STD rotation_count_stbd
            
            ; Set motor directions based on turn type
            LDAA turn_type
            
            CMPA #TURN_LEFT
            BEQ  SETUP_LEFT
            
            CMPA #TURN_RIGHT
            BEQ  SETUP_RIGHT
            
            CMPA #TURN_180
            BEQ  SETUP_180
            
            RTS
SETUP_LEFT
            ; Port REV, Stbd FWD
            BSET PORTA,%00000001
            BCLR PORTA,%00000010
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            RTS
SETUP_RIGHT
            ; Port FWD, Stbd REV
            BCLR PORTA,%00000001
            BSET PORTA,%00000010
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            RTS
SETUP_180
            ; Port FWD, Stbd REV (same as right, just longer)
            BCLR PORTA,%00000001
            BSET PORTA,%00000010
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            RTS

COL_INIT                                ; Initialize COLLISION state
            ; Set both motors to reverse
            BSET  PORTA,%00000001               ; Both REV (PA0=1, PA1=1)
            BSET  PORTA,%00000010
            ; Reset counters for reverse distance measurement
            LDD #$0000
            STD rotation_count_port

            LDD #$0000
            STD rotation_count_stbd
            
            ; Start reversing at medium speed
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            
            RTS

DONE_INIT
            ; Keep motors stopped
            JSR  STOP_MOTORS
            
            ; Check current mode
            LDAA robot_mode
            CMPA #MODE_EXPLORING
            BEQ  BCKTRK_INIT
            
             CLR  RETURN_PATH
            ; Already retraced - mission complete!
            ; Stay in COMPLETE state, do nothing
            RTS

BCKTRK_INIT                                ;Initialize BACKTRACK state
            ; Set mode to retracing
            LDAA #MODE_RETRACING
            STAA robot_mode
            
            ; Set return path flag
            LDAA #1
            STAA RETURN_PATH
            
            ; Reset intersection counter for return journey
            ; (Start from highest intersection and count down)
            LDAA intersection_solved
            STAA current_intersection
        
            ;Wait for rear bumper press to start return
            BRCLR PORTAD0, #%00001000, WAIT_FOR_REAR_BUMP 
           
            ; Start moving forward
            BCLR  PORTA,%00000001                ; Both FWD
            BCLR  PORTA,%00000010
            
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            
            RTS
            
WAIT_FOR_REAR_BUMP
        ; Just return here; BCKTRK will be called again next cycle
        RTS
;=============================================================================
; SENSOR PROCESSING
;=============================================================================
UPDT_READING
            JSR  LED_ON
            JSR  READ_ALL_SENSORS
            JSR  LED_OFF
            
            JSR  CONVERT_PATTERN     ; Convert ABCD to binary
            JSR  CHECK_EF_ZONES      ; Check E-F zones
            JSR  DETECT_PATTERN      ; Classify pattern type
            
            RTS
;============================================================
; SENSOR READING ROUTINES
; READ_ALL_SENSORS
;=============================================================================
READ_ALL_SENSORS        CLR SENSOR_NUM ; Select sensor number 0
                        LDX #LINE_SENSOR ; Point at the start of the sensor array
RS_MAIN_LOOP:
                        LDAA SENSOR_NUM ; Select the correct sensor input
                        JSR SELECT_SENSOR ; on the hardware
                        
                        LDY #400 ; 20 ms delay to allow the
                        JSR del_50us ; sensor to stabilize
                        
                        LDAA #%10000001 ; Start A/D conversion on AN1
                        STAA ATDCTL5
                        BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done

                        LDAA ATDDR0L ; A/D conversion is complete in ATDDR0L
                        STAA 0,X ; so copy it to the sensor register
                        CPX #STARBD_SENSOR ; If this is the last reading

                        BEQ RS_EXIT ; Then exit
                        INC SENSOR_NUM ; Else, increment the sensor number
                        INX ; and the pointer into the sensor array
                        BRA RS_MAIN_LOOP ; and do it again

RS_EXIT:     RTS

SELECT_SENSOR           PSHA ; Save the sensor number for the moment
                        
                        LDAA PORTA ; Clear the sensor selection bits to zeros
                        ANDA #%11100011 ;
                        STAA temp_a ; and save it into TEMP
                        PULA ; Get the sensor number
                        ASLA ; Shift the selection number left, twice
                        ASLA ;
                        ANDA #%00011100 ; Clear irrelevant bit positions
                        
                        ORAA temp_a ; OR it into the sensor bit positions
                        STAA PORTA ; Update the hardware
                        RTS
;============================================================
; LEDS ON AND OFF
;============================================================
LED_ON:     BSET PORTA,%00100000
            RTS

LED_OFF:    BCLR PORTA,%00100000
            RTS          
;============================================================
; SENSOR CONVERSION- Converts sensor_values[] into ABCDEF pattern bits
;===========================================================
CONVERT_PATTERN
            CLR  sensor_pattern
            
            ; Check A (bow)
            LDAA BOW_SENSOR
            CMPA #BOW_THRESH
            BLS  CHECK_B
            LDAA sensor_pattern
            ORAA #%00100000          ; Bit 5
            STAA sensor_pattern
CHECK_B
            LDAA PORT_SENSOR
            CMPA #PORT_THRESH
            BLS  CHECK_C
            LDAA sensor_pattern
            ORAA #%00010000          ; Bit 4
            STAA sensor_pattern
CHECK_C
            LDAA MIDD_SENSOR
            CMPA #MIDD_THRESH
            BLS  CHECK_D
            LDAA sensor_pattern
            ORAA #%00001000          ; Bit 3
            STAA sensor_pattern
CHECK_D
            LDAA STARBD_SENSOR
            CMPA #STARBD_THRESH
            BLS  CONVERT_DONE
            LDAA sensor_pattern
            ORAA #%00000100          ; Bit 2
            STAA sensor_pattern
CONVERT_DONE
            RTS
;-----------------------------------------------------------------------------
; E-F Zone Detection
; E-F is ONE sensor with ONE value - check which ZONE it's in
;-----------------------------------------------------------------------------
CHECK_EF_ZONES
            ; Clear all zone flags
            CLR  zone_left
            CLR  zone_right
            CLR  zone_centered
            
            LDAA LINE_SENSOR
            
            ; Check LEFT zone (< $60)
            CMPA #ELINE_THRESH
            BLO  IN_LEFT_ZONE
            
            ; Check RIGHT zone (> $A0)
            CMPA #RLINE_THRESH
            BHI  IN_RIGHT_ZONE
            
            ; Must be CENTERED zone
            INC  zone_centered
            RTS
IN_LEFT_ZONE
            LDAA #1
            STAA zone_left
            RTS

IN_RIGHT_ZONE
            LDAA #1
            STAA zone_right
            RTS
;-----------------------------------------------------------------------------
; Detect Pattern Type
;-----------------------------------------------------------------------------
DETECT_PATTERN
            LDAA sensor_pattern
            ; Cross junction (A, B, C, D)
            ANDA #%00111100
            CMPA #%00111100
            BNE  CHECK_T_PATTERN
            LDAA #PATTERN_CROSS
            STAA detected_pattern
            RTS

CHECK_T_PATTERN
            LDAA sensor_pattern
            ANDA #%00011100          ; B, C, D
            CMPA #%00011100
            BNE  CHECK_LEFT_PATTERN
            LDAA #PATTERN_T
            STAA detected_pattern
            RTS

CHECK_LEFT_PATTERN
            LDAA sensor_pattern
            ANDA #%00011000          ; B, C
            CMPA #%00011000
            BNE  CHECK_RIGHT_PATTERN
            LDAA #PATTERN_LEFT
            STAA detected_pattern
            RTS

CHECK_RIGHT_PATTERN
            LDAA sensor_pattern
            ANDA #%00001100          ; C, D
            CMPA #%00001100
            BNE  CHECK_LINE_PATTERN
            LDAA #PATTERN_RIGHT
            STAA detected_pattern
            RTS

CHECK_LINE_PATTERN
            LDAA sensor_pattern
            ANDA #%00101000          ; A, C
            CMPA #%00101000
            BNE  NO_PATTERN
            LDAA #PATTERN_LINE
            STAA detected_pattern
            RTS
NO_PATTERN
            LDAA #PATTERN_NONE
            STAA detected_pattern
            RTS
;=============================================================================
; LINE FOLLOWING CONTROL
;=============================================================================
LINE_FOLLOW_CONTROL
            ; Check which zone we're in
            LDAA zone_left
            CMPA #1
            BEQ  STEER_LEFT
            
            LDAA zone_right
            CMPA #1
            BEQ  STEER_RIGHT       
            ; Centered - go straight
            LDAA #MOTOR_FAST
            LDAB #MOTOR_FAST
            JSR  SET_MOTOR_SPEEDS
            RTS

STEER_LEFT
            ; Robot is LEFT of line â†’ turn LEFT to recenter ; Slow down left motor
            LDAA #MOTOR_SLOW
            LDAB #MOTOR_FAST
            JSR  SET_MOTOR_SPEEDS
            RTS

STEER_RIGHT
            ; Robot is RIGHT of line â†’ turn RIGHT to recenter ; Slow down right motor
            LDAA #MOTOR_FAST
            LDAB #MOTOR_SLOW
            JSR  SET_MOTOR_SPEEDS
;=============================================================================
; MOTOR CONTROL
;=============================================================================
SET_MOTOR_DIRECTIONS    ; Input: A = port dir (0=FWD,1=REV), B = stbd dir
            PSHA
            LDAA PORTA
            ANDA #%11111100         ; Clear PA0, PA1
            STAA temp_a
            
            PULA
            TSTA
            BEQ  SMD_PORT_FWD
            LDAA temp_a
            ORAA #%00000001
            STAA temp_a
SMD_PORT_FWD
            TSTB
            BEQ  SMD_STBD_FWD
            LDAA temp_a
            ORAA #%00000010
            STAA temp_a
SMD_STBD_FWD
            LDAA temp_a
            STAA PORTA
            RTS

SET_MOTOR_SPEEDS
            ; Input: A = port speed, B = stbd speed
            STAA port_motor_speed
            STAB stbd_motor_speed 
            ; Update PTT based on speeds
            CLR  temp_a
            ; Check port motor
            LDAA port_motor_speed
            CMPA #MOTOR_MIN
            BLO  CHECK_STBD_MOTOR
            LDAA #%00010000                ; PT4
            STAA temp_a

CHECK_STBD_MOTOR
            LDAA stbd_motor_speed
            CMPA #MOTOR_MIN
            BLO  APPLY_SPEEDS
            LDAA temp_a
            ORAA #%00100000                ; PT5
            STAA temp_a

APPLY_SPEEDS
            LDAA temp_a
            STAA PTT
            RTS

STOP_MOTORS
            CLR  port_motor_speed
            CLR  stbd_motor_speed
            BCLR PTT,%00110000    ; Port motor off
            RTS
;-----------------------------------------------------------------------------
; Update Direction after Turn
;-----------------------------------------------------------------------------
UPDATE_DIRECTION
            LDAA turn_type
            
            CMPA #TURN_LEFT
            BEQ  UPDATE_LEFT
            
            CMPA #TURN_RIGHT
            BEQ  UPDATE_RIGHT
            
            CMPA #TURN_180
            BEQ  UPDATE_180
            
            RTS

UPDATE_LEFT
            ; Rotate counterclockwise
            LDAA current_direction
            BEQ  SET_WEST
            DECA
            STAA current_direction
            RTS
SET_WEST
            LDAA #WEST
            STAA current_direction
            RTS

UPDATE_RIGHT
            ; Rotate clockwise
            LDAA current_direction
            CMPA #WEST
            BEQ  SET_NORTH
            INCA
            STAA current_direction
            RTS
SET_NORTH
            CLR  current_direction
            RTS

UPDATE_180
            ; Reverse direction
            LDAA current_direction
            ADDA #2
            CMPA #4
            BLO  STORE_180
            SUBA #4
STORE_180
            STAA current_direction
            RTS
;=============================================================================
; BUMPER DETECTION
;=============================================================================
CHECK_BUMPERS
            ; Check bow bumper (bit 2)
            BRCLR PORTAD0,%00000100,BOW_HIT      ; If bit 2 = 0, bumper pressed, MAY CHANGE BRCLR TO BRSET
            
            ; Check stern bumper (bit 3)
            BRCLR PORTAD0,%00001000,STERN_HIT    ; If bit 3 = 0, bumper pressed
            
            RTS                              ; No bumpers pressed
;-----------------------------------------------------------
; Bow bumper pressed => collision/dead end
;-----------------------------------------------------------
BOW_HIT
            LDAA robot_state
            CMPA #FLW
            BNE  BUMPER_EXIT         ; Only respond in follow state
            
            MOVB #COL, robot_state
BUMPER_EXIT
            RTS              
;-----------------------------------------------------------
; Stern bumper pressed ? reached destination
;-----------------------------------------------------------
STERN_HIT
            LDAA #DONE
            STAA robot_state
            JSR  STOP_MOTORS       
            ; Switch to retracing mode
            LDAA #MODE_RETRACING
            STAA robot_mode
            RTS

            BRA  START_BACKTRACK
            ; Already backtracking - done!
            MOVB #DONE, robot_state
            RTS

START_BACKTRACK
            JSR  BCKTRACK_FOLLOW
            MOVB #BCKTRK, robot_state

            RTS
;=============================================================================
; MAZE MANAGEMENT
;=============================================================================
PUSH_INTERSECTION
            LDAA path_stack_ptr
            CMPA #MAX_INTERSECTIONS
            BHS  PUSH_OVERFLOW
            
            ; Store current_intersection
            LDX  #path_stack
            LDAB path_stack_ptr
            LDAA current_intersection
            STAA B,X             
            INC  path_stack_ptr

PUSH_OVERFLOW
            RTS

POP_INTERSECTION
            LDAA path_stack_ptr
            BEQ  POP_EMPTY            
            DECA
            STAA path_stack_ptr
            
            LDX #path_stack
            LDAB path_stack_ptr
            LDAA B,X
            STAA current_intersection
            RTS

POP_EMPTY
            RTS
;-----------------------------------------------------------------------------
; Records the first direction attempted at an intersection. Called when exploring a new junction
; Input: A = intersection number, B = turn direction tried
;-----------------------------------------------------------------------------
RECORD_TRIED_DIRECTION
            LDX  #tried_direction
            STAB A,X                 ; Store which direction we tried first
            RTS

;-----------------------------------------------------------------------------
; RECORD_CORRECTION
; Called when we hit a dead end - records the OPPOSITE direction as correct. 
; Input: None (uses current_intersection)
; Logic: If we tried LEFT and hit dead end, RIGHT must be correct
;-----------------------------------------------------------------------------
RECORD_CORRECTION
            ; Mark current intersection as solved
            LDAA current_intersection
            LDX  #intersection_solved
            LDAB #1
            STAB A,X
            
            ; Get what we tried first
            LDAA current_intersection
            LDX  #tried_direction
            LDAB A,X                 ; B = direction we tried (and failed)
            ; Calculate opposite direction
            CMPB #TURN_LEFT
            BEQ  RC_SET_RIGHT

            CMPB #TURN_RIGHT
            BEQ  RC_SET_LEFT 
            
            RTS

RC_SET_LEFT
            LDAB #TURN_LEFT
            BRA  RC_STORE

RC_SET_RIGHT
            LDAB #TURN_RIGHT

RC_STORE
            ; Store correct direction
            LDAA current_intersection
            LDX  #maze_solution
            STAB A,X
            RTS
;-----------------------------------------------------------------------------
; CHECK_IF_SOLVED
; Checks if current intersection already has a known solution
; Input: A = intersection number. 
; Output: A = 0 (not solved) or 1 (solved), B = solution direction (if solved)
;-----------------------------------------------------------------------------
CHECK_IF_SOLVED
            LDX  #intersection_solved
            LDAB A,X                 ; Get solved flag
            TSTB
            BEQ  CIS_NOT_SOLVED
            
            ; It's solved - get the solution
            LDX  #maze_solution
            LDAB A,X                 ; B = correct direction
            LDAA #1                  ; A = solved flag
            RTS

CIS_NOT_SOLVED
            CLRA                     ; A = 0 (not solved)
            RTS           
;=============================================================================
; INITIALIZATION ROUTINES
;=============================================================================
INIT_PORTS
         ;----------------------------------------
         ; Configure PORTAD: analog inputs
         ;----------------------------------------
            BCLR  DDRAD, $FF           ; All PORTAD pins as input
            BSET  DDRA,  $FF
            BSET  DDRT,  $30       

            RTS
INIT_TCNT
            ; Enable timer system
            MOVB #$80,TSCR1            
            ; Prescaler and overflow interrupt
            MOVB #$00,TSCR2            ; TOI=1, prescaler      
            ; Configure IC0,IC1 for wheel counters
            MOVB #$FC,TIOS             ; All input capture initially
            MOVB #$05,TCTL4
            MOVB #$03,TFLG1 
            MOVB #$03,TIE              ; Enable IC0, IC1 interrupts
            
            RTS            

INIT_MAZE_DATA
            ; Set starting direction (East)
            LDAA #EAST
            STAA current_direction   
            ; Clear intersection counter
            CLR  current_intersection
            ; Clear stack pointer
            CLR  path_stack_ptr
            ; Set exploring mode
            LDAA #MODE_EXPLORING
            STAA robot_mode
            ; Initialize maze_solution to NO_INTERSECTION
            LDX  #maze_solution
            LDAA #MAX_INTERSECTIONS

INIT_MAZE_LOOP
            STAB 1,X+           
            DECA
            BNE  INIT_MAZE_LOOP
            
            ; Clear intersection_solved flags
            LDX  #intersection_solved
            LDAA #MAX_INTERSECTIONS

INIT_SOLVED_LOOP
            CLR  1,X+
            DECA
            BNE  INIT_SOLVED_LOOP
            
            RTS
; utility subroutines
;*******************************************************************
;* Initialization of the LCD: 4-bit data width, 2-line display,    *
;* turn on display, cursor and blinking off. Shift cursor right.   *
;*******************************************************************
initLCD     BSET DDRB,%11111111               ; configure pins PB7...PB0 for output
            BSET DDRJ,%11000000                ; configure pins PJ7,PJ6 for reset and enable
            LDY #2000                         ; wait for LCD to be ready
            JSR del_50us                       ; -"-
            LDAA #$28                         ; set 4-bit data, 2-line display
            JSR cmd2LCD                       ; -"-
            LDAA #$0C                         ; display on, cursor off, blinking off
            JSR cmd2LCD                        ; -"-
            LDAA #$06                         ; move cursor right after entering a character
            JSR cmd2LCD                        ; -"-
            RTS
;* Clear display and home cursor *
;*******************************************************************
clrLCD      LDAA #$01                       ; clear cursor and return to home position
            JSR cmd2LCD                     ; -"-
            LDY #40                         ; wait until "clear cursor" command is complete
            JSR del_50us                    ; -"-
            RTS
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns. *
;*******************************************************************
del_50us:   PSHX          ;2 E-clk
eloop:      LDX #30       ;2 E-clk -
iloop:      PSHA          ;2 E-clk |
            PULA          ;3 E-clk |
            PSHA          ;2 E-clk | 50us
            PULA          ;3 E-clk |
            PSHA          ;2 E-clk |
            PULA          ;3 E-clk |
            PSHA          ;2 E-clk |
            PULA          ;3 E-clk |
            PSHA          ;2 E-clk |
            PULA          ;3 E-clk |
            PSHA          ;2 E-clk |
            PULA          ;3 E-clk |
           
            NOP           ;1 E-clk |
            NOP           ;1 E-clk |
            DBNE X,iloop  ;3 E-clk -
            DBNE Y,eloop  ;3 E-clk
            PULX          ;3 E-clk
            RTS           ;5 E-clk          
;* This function sends a command in accumulator A to the LCD *
;*******************************************************************
cmd2LCD:    BCLR LCD_CNTR,LCD_RS  ; select the LCD Instruction Register (IR)
            JSR dataMov           ; send data to IR
            RTS
;* This function outputs a NULL-terminated string pointed to by X *
;*******************************************************************
putsLCD     LDAA 1,X+             ; get one character from the string
            BEQ donePS            ; reach NULL character?
            JSR putcLCD
            BRA putsLCD
donePS      RTS
;*******************************************************************
;* This function outputs the character in accumulator in A to LCD *
;*******************************************************************
putcLCD     BSET LCD_CNTR,LCD_RS      ; select the LCD Data register (DR)
            JSR dataMov               ; send data to DR
            RTS        
;* This function sends data to the LCD IR or DR depening on RS *
;*******************************************************************
dataMov     BSET LCD_CNTR,LCD_E       ; pull the LCD E-sigal high
            STAA LCD_DAT              ; send the upper 4 bits of data to LCD
            BCLR LCD_CNTR,LCD_E       ; pull the LCD E-signal low to complete the write oper.
            LSLA                      ; match the lower 4 bits with the LCD data pins
            LSLA                      ; -"-
            LSLA                      ; -"-
            LSLA  ;"-
            BSET LCD_CNTR,LCD_E ; pull the LCD E signal high
            STAA LCD_DAT ; send the lower 4 bits of data to LCD
            BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
            LDY #1 ; adding this delay will complete the internal
            JSR del_50us ; operation for most instructions
            RTS

initAD      MOVB #$C0,ATDCTL2 ;power up AD, select fast flag clear
            JSR del_50us ;wait for 50 us
            MOVB #$00,ATDCTL3 ;8 conversions in a sequence
            MOVB #$85,ATDCTL4 ;res=8, conv-clks=2, prescal=12
            BSET ATDDIEN,$0C ;configure pins AN03,AN02 as digital inputs
            RTS
;****************************************************************
int2BCD     XGDX                   ; Save the binary number into .X
            LDAA #0                ;lear the BCD_BUFFER
            STAA TEN_THOUS
            STAA THOUSANDS
            STAA HUNDREDS
            STAA TENS
            STAA UNITS
            STAA BCD_SPARE
            STAA BCD_SPARE+1
            
            CPX #0                 ;Check for a zero input
            BEQ CON_EXIT           ; and if so, exit

            XGDX                   ;Not zero, get the binary number back to .D as dividend
            LDX #10                ; Setup 10 (Decimal!) as the divisor
            IDIV                   ;Divide: Quotient is now in .X, remainder in .D
            STAB UNITS             ;Store remainder
            CPX #0                 ;If quotient is zero,
            BEQ CON_EXIT           ; then exit

            XGDX                   ;swap first quotient back into .D
            LDX #10                ;and setup for another divide by 10
            IDIV
            STAB TENS
            CPX #0
            BEQ CON_EXIT

            XGDX                   ;Swap quotient back into .D
            LDX #10                ;and setup for another divide by 10
            
            IDIV
            STAB HUNDREDS
            CPX #0
            BEQ CON_EXIT

            XGDX                   ;Swap quotient back into .D
            LDX #10                ;and setup for another divide by 10
            IDIV
            STAB THOUSANDS
            CPX #0
            BEQ CON_EXIT

            XGDX                    ;Swap quotient back into .D
            LDX #10                 ;and setup for another divide by 10
            IDIV
            STAB TEN_THOUS

CON_EXIT    RTS
;***************************************************************
BCD2ASC     LDAA  #0               ; Initialize the blanking flag
            STAA NO_BLANK
C_TTHOU     LDAA TEN_THOUS         ;Check the ï¿½ten_thousandsï¿½ digit
            ORAA NO_BLANK
            BNE NOT_BLANK1
ISBLANK1    LDAA #' '             ; It's blank
            STAA TEN_THOUS        ;so store a space
            BRA  C_THOU           ;and check the thousands digit
NOT_BLANK1  LDAA TEN_THOUS        ;Get the ten_thousands digit
            ORAA #$30             ;Convert to ascii
            STAA TEN_THOUS
            LDAA #$1              ;Signal that we have seen a non-blank digit
            STAA NO_BLANK
C_THOU      LDAA THOUSANDS        ;Check the thousands digit for blankness
            ORAA NO_BLANK         ;If itï¿½s blank and no-blank is still zero
            BNE  NOT_BLANK2
ISBLANK2    LDAA  #' '             ; Thousands digit is blank
            STAA THOUSANDS         ;so store a space
            BRA  C_HUNS            ;and check the hundreds digit
NOT_BLANK2  LDAA THOUSANDS         ;(similar to ten_thousands case)
            ORAA #$30
            STAA THOUSANDS
            LDAA #$1
            STAA NO_BLANK
C_HUNS      LDAA HUNDREDS           ;Check the hundreds digit for blankness
            ORAA NO_BLANK           ;If itï¿½s blank and no-blank is still zero
            BNE NOT_BLANK3
ISBLANK3    LDAA  #' '             ; Hundreds digit is blank
            STAA HUNDREDS          ;so store a space
            BRA C_TENS             ;and check the tens digit
NOT_BLANK3  LDAA HUNDREDS          ;(similar to ten_thousands case)
            ORAA #$30
            STAA HUNDREDS
            LDAA #$1
            STAA NO_BLANK
C_TENS      LDAA TENS               ;Check the tens digit for blankness
            ORAA NO_BLANK           ;If itï¿½s blank and no-blank is still zero
            BNE NOT_BLANK4  ;
ISBLANK4    LDAA  #' '             ; Tens digit is blank
            STAA TENS              ;so store a space
            BRA C_UNITS            ;and check the units digit
NOT_BLANK4  LDAA TENS              ;(similar to ten_thousands case)
            ORAA #$30
            STAA TENS
C_UNITS     LDAA UNITS              ;No blank check necessary, convert to ascii.
            ORAA #$30
            STAA UNITS

            RTS         
;************************************************************
ENABLE_TOF  LDAA #%10000000
            STAA TSCR1              ; Enable TCNT
            STAA TFLG2              ; Clear TOF
            LDAA #%10000100         ; Enable TOI and select prescale factor equal to 16
            STAA TSCR2
            RTS
;************************************************************
TOF_ISR     INC  TOF_COUNTER
            LDAA #%10000000          ; Clear
            STAA TFLG2                ; TOF
            RTI
;********************************************************************************************
;* INTERRUPT SERVICE ROUTINE 1  /  Port wheel counter                                                            
;********************************************************************************************
IC0_ISR         
                MOVB  #$01,TFLG1                ; clear the C0F input capture flag
                INC   rotation_count_port                    ; increment rotation_count_port
                RTI
;********************************************************************************************
;* INTERRUPT SERVICE ROUTINE 2   / Starboard wheel counter                                                         
;********************************************************************************************
IC1_ISR         
                MOVB  #$02,TFLG1                ; clear the C1F input capture flag
                INC   rotation_count_stbd                    ; increment rotation_count_stbd 
                RTI
;************************************************************
HEX_TABLE       FCC '0123456789ABCDEF'          ; Table for converting values
BIN2ASC         PSHA                            ; Save a copy of the input number on the stack
                TAB                             ; and copy it into ACCB
                ANDB #%00001111                 ; Strip off the upper nibble of ACCB
                CLRA                            ; D now contains 000n where n is the LSnibble
                ADDD #HEX_TABLE                 ; Set up for indexed load
                XGDX                
                LDAA 0,X                        ; Get the LSnibble character
                PULB                            ; Retrieve the input number into ACCB
                PSHA                            ; and push the LSnibble character in its place
                RORB                            ; Move the upper nibble of the input number
                RORB                            ;  into the lower nibble position.
                RORB
                RORB 
                ANDB #%00001111                 ; Strip off the upper nibble
                CLRA                            ; D now contains 000n where n is the MSnibble 
                ADDD #HEX_TABLE                 ; Set up for indexed load
                XGDX                                                               
                LDAA 0,X                        ; Get the MSnibble character into ACCA
                PULB                            ; Retrieve the LSnibble character into ACCB
                RTS         
;*******************************************************************
;* Update Display (Battery Voltage + Current State) *
;*******************************************************************
UPDT_DISP     
                LDAA  #$82                      ; Move LCD cursor to the end of msg1
                JSR   cmd2LCD                   ;
                
                LDAB  robot_state                ; Display current state
                LSLB                            ; "
                LSLB                            ; "
                LSLB                            ; "
                LDX   #tab                      ; "
                ABX                             ; "
                JSR   putsLCD                   ; "
;------------------------------------------------                
                LDAA  #$8F                      ; Move LCD cursor to the end of msg2
                JSR   cmd2LCD                   ; ""
                LDAA  BOW_SENSOR                ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""
                LDAA  #$92                      ; Move LCD cursor to Line position 
                JSR   cmd2LCD                   ; ""
                LDAA  LINE_SENSOR               ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$CC                      ; Move LCD cursor to Port position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  PORT_SENSOR               ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$CF                      ; Move LCD cursor to Mid position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  MIDD_SENSOR                ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$D2                      ; Move LCD cursor to Starboard position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  STARBD_SENSOR               ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""
          
                MOVB  #$90,ATDCTL5              ; R-just., uns., sing. conv., mult., ch=0, start
                BRCLR ATDSTAT0,$80,*            ; Wait until the conver. seq. is complete
                LDAA  ATDDR0L                   ; Load the ch0 result - battery volt - into A
                LDAB  #39                       ; AccB = 39
                MUL                             ; AccD = 1st result x 39
                ADDD #600                      ; AccD = 1st result x 39 + 600
                JSR   int2BCD
                JSR   BCD2ASC
                LDAA  #$C2                      ; move LCD cursor to the end of msg3
                JSR   cmd2LCD                   ; "                
                LDAA  TEN_THOUS                 ; output the TEN_THOUS ASCII character
                JSR   putcLCD                   ; "
                LDAA  THOUSANDS                 ; output the THOUSANDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  #$2E                      ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  HUNDREDS                  ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "                
;------------------------------------------------
                LDAA  #$C9                      ; Move LCD cursor to the end of msg4
                JSR   cmd2LCD
                
                BRCLR PORTAD0,#%00000100,bowON  ; If FWD_BUMP, then
                LDAA  #$20                      ;
                JSR   putcLCD                   ;
                BRA   stern_bump                ; Display 'B' on LCD, 42 IS B IN ASCII CODE
         
         bowON: LDAA  #$42                      ; ""
                JSR   putcLCD                   ; ""
          
        stern_bump: BRCLR PORTAD0,#%00001000,sternON    ; If REV_BUMP, then
                LDAA  #$20                      ;
                JSR   putcLCD                   ;
                BRA   UPDT_DISP_EXIT           ; Display 'S' on LCD, 53 IS S IN ASCII CODE
        
        sternON: LDAA  #$53                     ; ""
                JSR   putcLCD                   ; ""

UPDT_DISP_EXIT RTS                             ; and exit                
;=============================================================================
; INTERRUPT VECTORS
;=============================================================================
            ORG  $FFFE
            DC.W Entry                  ; Reset
            
            ORG  $FFDE
            DC.W TOF_ISR                ; Timer Overflow
            
            ORG  $FFEC
            DC.W IC0_ISR                ; Input Capture 0
            
            ORG  $FFEA
            DC.W IC1_ISR                ; Input Capture 1
;*****************************************************************************
; END OF PROGRAM
;*****************************************************************************