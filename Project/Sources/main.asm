;*****************************************************************************
; EEbot - CORRECTED Complete Maze Navigation Program
; MC9S12C32 (HCS12 family)
; 
; Features:
; - Guider sensor scanning with proper LED control
; - Proportional line following with deadband
; - Intersection detection and decision making
; - Maze learning and memory
; - Wheel-counted precise turns
; - Bumper collision detection
; - State machine navigation
;*****************************************************************************
           
; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry                  ; for absolute assembly: mark this as application entry point
            
; Include derivative-specific definitions 
		INCLUDE 'derivative.inc'     ; Use CodeWarrior's include file

;=============================================================================
; CONSTANTS / EQUATES SECTION
;=============================================================================

ROMStart        EQU  $4000                      ; absolute address to place my code/constant data

LCD_DAT         EQU   PORTB                     ; LCD data port, bits - PB7,...,PB0
LCD_CNTR        EQU   PTJ                       ; LCD control port, bits - PJ6(RS),PJ7(E)
LCD_E           EQU   $80                       ; LCD E-signal pin
LCD_RS          EQU   $40                       ; LCD RS-signal pin
                             
;=============================================================================
; GUIDER SECTION(VARIES FOR EACH EEBOT, CHANGE. MUST BE CALIBRATED!) 
;=============================================================================

; Motor Control
MOTOR_MIN       EQU 70               ; Calibrate: minimum to overcome friction
MOTOR_SLOW      EQU 102              ; $46
MOTOR_MED       EQU 153              ; $99
MOTOR_FAST      EQU 200              ; Calibrate: max safe speed , $C8
MOTOR_MAX       EQU 255              ; $FF

;WHEEL COUNTS (DISTANCE UNITS IN ENCODER COUNTS)
INCR_DIS         EQU   300                       ; INCREMENT distance
FRWD_DIS         EQU   2000                      ; FORWARD distance
REVR_DIS         EQU   1000                      ; REVERSE distance
STRT_DIS         EQU   1000                      ; STRAIGHT distance
TURN_DIS         EQU   13000                     ; TURN distance
UTURN_DIS        EQU   15000                     ; U-TURN distance

; Sensor Selection (PA2-PA4 values)
SLCT_LINE_LEFT      EQU 0                ; E differential, Line sensor
SLCT_LINE_RIGHT     EQU 1                ; F differential, Line sensor
SLCT_A              EQU 2                ; Bow absolute, Front sensor
SLCT_B              EQU 3                ; Port absolute, Port sensor
SLCT_C              EQU 4                ; Mid absolute, Middle Sensor
SLCT_D              EQU 5                ; Starboard absolute, Starboard sensor

; Sensor Thresholds (CALIBRATE THESE!)
EF_DARK          EQU $B4              ; Above = dark line           
EF_LIGHT         EQU $60              ; Below = light surface
BOW_THRESH       EQU $99              ; SENSOR A
PORT_THRESH      EQU $99              ; SENSOR B
MIDD_THRESH      EQU $99              ; SENSOR C
STARBD_THRESH    EQU $99              ; SENSOR D
EF_THRESH        EQU $B4              ; SENSOR E & F
RLINE_THRESH     EQU EF_THRESH

;line following tracking/ tuning
LINE_CENTER         EQU $80
LINE_DEADBAND       EQU 5

; Timing
SETTLE_DELAY    EQU 20               ; ms to wait for CdS settling
COUNTS_90_DEG   EQU 26               ; Wheel counts for 90� turn

; State Machine States
START_STAT          EQU 0
FLLW_STAT           EQU 1
JUNCT_STAT          EQU 2        
TURN_STAT           EQU 3
COL_STAT            EQU 4
REV_STAT            EQU 5
DONE_STAT           EQU 6
BCKTRK_STAT         EQU 7

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
TURN_REVERSE        EQU 3
NO_TURN             EQU $FF    ;Means remain in same direction   Delete if not relevant Do not add to turn implementation


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
NO_BLANK            ds.b 1          ; Used in �leading zero� blanking by BCD2ASC
BCD_SPARE           RMB 10          ; Extra space for decimal point and string terminator
CURRENT_GUIDER_VALUE  DS.B 1

; State & Control
robot_state           DS.B 1        ; CURRENT STATE
robot_mode            DS.B 1        ; EXPLORATION MODE OR RETRACING MODE
current_direction     DS.B 1        ; NORTH, SOUTH, EAST OR WEST
current_intersection  DS.B 1        ; 0 TO 6
path_stack_ptr        DS.B 1

; Maze Learning , RMB USED BECAUSE THESE ARE ARRAYS ie FOR DATA STORAGE
maze_solution       RMB MAX_INTERSECTIONS
tried_direction     RMB MAX_INTERSECTIONS
intersection_solved RMB MAX_INTERSECTIONS
path_stack          RMB MAX_INTERSECTIONS
intersection_attempts RMB MAX_INTERSECTIONS
next_path        DS.B 1       ; Stores the next path to attempt at an intersection


; Sensor Data
junction_type       DS.B 1
sensor_values       RMB 6            
sensor_pattern      DS.B 1           ; Binary pattern: bits ABCDEF (543210)
line_value          DS.B 1
 
;=============================================================================
; Sensor Index Map (in RAM)
;sensor_values[0] = A
;sensor_values[1] = B
;sensor_values[2] = C
;sensor_values[3] = D
;sensor_values[4] = E
;sensor_values[5] = F
;=============================================================================

CNT1          DC.W  0                         ; initialize 2-byte COUNT1 to $0000
CNT2          DC.W  0                         ; initialize 2-byte COUNT2 to $0000

; Motor Control
port_motor_speed    DS.B 1
stbd_motor_speed    DS.B 1

; Wheel Counters , 16 Bits to avoid overflow
rotation_count_port DS.W 1
rotation_count_stbd DS.W 1

; Temporaries
temp_a              DS.B 1
temp_b              DS.B 1
TEMP                DS.B 1            


RETURN_PATH      DC.B  0                         ; RETURN (TRUE = 1, FALSE = 0)
NEXT_DIR         DC.B  1                         ; Next direction instruction

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
Entry:
_Startup:
            SEI                         ; Disable interrupts during init
            LDS  #$4000                 ; Initialize stack pointer

            ; Initialize Hardware
            JSR  INIT_PORTS
            JSR  INIT_TCNT
            JSR   initAD                    ; Initialize ATD converter                  I                                                ;                                           A
            JSR   initLCD                   ; Initialize the LCD                        L
            JSR   clrLCD                    ; Clear LCD & home cursor   
            ; Initialize Variables
            JSR  INIT_MAZE_DATA
            
            ; Safety: Motors off
            JSR  STOP_MOTORS
                                                          ;                                           T
            ; Set initial state
            LDAA #START_STAT
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

;=============================================================================
; MAIN LOOP
;=============================================================================
MainLoop:

    ;==============================================================
    ; 1. Read all raw sensors
    ;==============================================================
    JSR   READ_ALL_SENSORS
    JSR   SENSOR_CONVERT

    ;==============================================================
    ; 2. Update all detection & decision flags
    ;==============================================================
    JSR   CHECK_BUMPERS          ; safety flags

    ;==============================================================
    ; 3. STATE MACHINE (decision + motor control)
    ;==============================================================
    JSR   DISPATCHER

    ;==============================================================
    ; 4. Display update timer
    ;==============================================================
    JSR   UPDT_READING
    JSR   UPDT_DISP
    BRA   MainLoop

;*******************************************************************
; data section, MAY RENAME LATER IF NEEDED
;*******************************************************************

msg1        dc.b "Battery volt ",0
msg2        dc.b "State ",0
msg3        dc.b "Sensor ",0
msg4        dc.b "Bumper ",0

tab         dc.b "START_STAT",0
            dc.b "FLLW_STAT",0
            dc.b "JUNCT_STAT",0
            dc.b "TURN_STAT",0
            dc.b "COL_STAT",0
            dc.b "REV_STAT",0
            dc.b "DONE_STAT",0
            dc.b "BCKTRK_STAT",0
        
STATE_TABLE:
            FDB START_STAT_INIT   ; 0
            FDB FLLW_STAT_INIT    ; 1
            FDB JUNCT_STAT_INIT   ; 2
            FDB TURN_STAT_INIT    ; 3
            FDB COL_STAT_INIT     ; 4
            FDB REV_STAT_INIT     ; 5
            FDB DONE_STAT_INIT    ; 6
            FDB BCKTRK_STAT_INIT  ; 7

; Turn Result Table: [current_dir * 3 + turn_type] ? new_direction
turn_result_table:
            DC.B EAST, NORTH, WEST      ; From NORTH: RIGHT, STRAIGHT, LEFT
            DC.B SOUTH, EAST, NORTH     ; From EAST
            DC.B WEST, SOUTH, EAST      ; From SOUTH
            DC.B NORTH, WEST, SOUTH     ; From WEST

; Required Turn Table: [current_dir * 4 + desired_dir] ? turn_type
required_turn_table:
            DC.B NO_TURN, TURN_RIGHT, TURN_REVERSE, TURN_LEFT
            DC.B TURN_LEFT, NO_TURN, TURN_RIGHT, TURN_REVERSE
            DC.B TURN_REVERSE, TURN_LEFT, NO_TURN, TURN_RIGHT
            DC.B TURN_RIGHT, TURN_REVERSE, TURN_LEFT, NO_TURN  
                                     
;=============================================================================
; INITIALIZATION ROUTINES
;=============================================================================
INIT_PORTS:
         ;----------------------------------------
         ; Configure PORTAD: analog inputs
         ;----------------------------------------
            BCLR  DDRAD, #$FF           ; All PORTAD pins as input
            BSET  DDRA,  #$FF
            BSET  DDRT,  $30       

            RTS

INIT_TCNT:
            ; Enable timer system
            MOVB #$80, TSCR1            ; TEN=1, TFFCA=1
            
            ; Prescaler and overflow interrupt
            MOVB #$00, TSCR2            ; TOI=1, prescaler
            
            ; Configure IC0,IC1 for wheel counters
            MOVB #$FC, TIOS             ; All input capture initially
            MOVB #$05, TCTL4
            MOVB #$03, TFLG1 
            MOVB #$03, TIE              ; Enable IC0, IC1 interrupts
            
            RTS            

INIT_MAZE_DATA:
            ; Set starting direction (East)
            LDAA #EAST
            STAA current_direction
            
            ; Clear intersection counter
            CLR  current_intersection
            
            ; Set exploring mode
            LDAA #MODE_EXPLORING
            STAA robot_mode
            
            ; Clear stack pointer
            CLR  path_stack_ptr
            
            ; Initialize maze_solution to NO_INTERSECTION
            LDX  #maze_solution
            LDAA #MAX_INTERSECTIONS

INIT_MAZE_LOOP:
            STAB 1,X+
            DECA
            BNE  INIT_MAZE_LOOP
            
            ; Clear intersection_solved flags
            LDX  #intersection_solved
            LDAA #MAX_INTERSECTIONS

INIT_SOLVED_LOOP:
            CLR  1,X+
            DECA
            BNE  INIT_SOLVED_LOOP
            
            ; init attempted counters
            LDX #intersection_attempts
            LDAA #MAX_INTERSECTIONS

INIT_ATTEMPTS:
            CLR  1,X+
            DECA
            BNE INIT_ATTEMPTS

            RTS

;********************************************************************************************
;* SUBROUTINES SECTION                                                                       *
;********************************************************************************************
;+------------------------------------------------------------------------------------------+
;| Starboard (Right) Motor ON                                                               |
;+------------------------------------------------------------------------------------------+
STRBDON          BSET  PTT,%00100000 ; 20
                RTS

;+------------------------------------------------------------------------------------------+
;| Starboard (Right) Motor OFF                                                              |
;+------------------------------------------------------------------------------------------+
STRBDOFF         BCLR  PTT,%00100000
                RTS

;+------------------------------------------------------------------------------------------+
;| Starboard (Right) Motor FWD                                                              |
;+------------------------------------------------------------------------------------------+
STRBDFWD        BCLR  PORTA,%00000010
                RTS

;+------------------------------------------------------------------------------------------+
;| Starboard (Right) Motor REV                                                              |
;+------------------------------------------------------------------------------------------+
STRBDREV        BSET  PORTA,%00000010
                RTS
;+------------------------------------------------------------------------------------------+

;+------------------------------------------------------------------------------------------+
;| Port (Left) Motor ON                                                                     |
;+------------------------------------------------------------------------------------------+
PORTON          BSET  PTT,%00010000
                RTS

;+------------------------------------------------------------------------------------------+
;| Port (Left) Motor OFF                                                                    |
;+------------------------------------------------------------------------------------------+
PORTOFF         BCLR  PTT,%00010000
                RTS

;+------------------------------------------------------------------------------------------+
;| Port (Left) Motor FWD                                                                    |
;+------------------------------------------------------------------------------------------+
PORTFWD         BCLR  PORTA,%00000001
                RTS

;+------------------------------------------------------------------------------------------+
;| Port (Left) Motor REV                                                                    |
;+------------------------------------------------------------------------------------------+
PORTREV         BSET  PORTA,%00000001
                RTS
;+------------------------------------------------------------------------------------------+

;********************************************************************************************
;* NEW STATES DISPATCHER / STATE MACHINE                                                                        
;*******************************************************************
DISPATCHER:
            LDAA robot_state
            CMPA #BCKTRK_STAT + 1   ; Bounds check
            BHS DISPATCHER_DEFAULT  ; If out of bounds, jump to default
            
            ; Compute address in jump table
            LSLA                    ; A = A * 2 (each entry is 2 bytes)
            LDX #STATE_TABLE
            ABX          ; X points to the correct entry instead of checking each one
            JMP 0,X      ; Jump to state initializer

DISPATCHER_DEFAULT:            
            ; Default: unknown state - stop motors for safety
            JSR  STOP_MOTORS
            RTS

;-----------------------------------------------------------------------------
; STATE: INIT - Wait for start
;-----------------------------------------------------------------------------
START_STAT_INIT:
            ; Option 1: Auto-start
            ;LDAA #FLLW_STAT
            ;STAA robot_state
            ;RTS
            
            ; Option 2: Wait for bumper press (uncomment if desired)
             BRCLR PORTAD0, #$04, START_PRESSED
             
             RTS

START_PRESSED:
                 LDAA #FLLW_STAT
                 STAA robot_state
                 RTS

;-----------------------------------------------------------------------------
; STATE: FOLLOWING - Line tracking and intersection detection
;-----------------------------------------------------------------------------
FLLW_STAT_INIT:

            ;Only check junction if not already at a junction
            LDAA junction_type
            CMPA #0
            BEQ CHECK_JUNCTION
            BRA LINE_FOLLOW  ;if at a junction, follow

CHECK_JUNCTION:
            ; Check for junction
            JSR  THRESHOLD_PATTERN
            TST  junction_type           ; Result flag in junction_type
            BNE  FOUND_JUNCTION
            
LINE_FOLLOW:
            ; No junction detected or at a current junction - do line following
            JSR  LINE_FOLLOW_CONTROL
            RTS 

FOUND_JUNCTION:
            ; Move into junction handling state
            LDAA #JUNCT_STAT
            STAA robot_state
            ; Reset next_path counter for this intersection
            CLR next_path
            RTS

;-----------------------------------------------------------------------------
; STATE: AT_JUNCTION - Intersection type detection and movement implementation
;-----------------------------------------------------------------------------
JUNCT_STAT_INIT:
            ; Reset attempt counter for this intersection if it's new
            LDAA current_intersection
            LDX #intersection_attempts
            STAB 0,X       ; ensure safe
            JSR HANDLE_JUNCTION
            RTS

;-----------------------------------------------------------------------------
; STATE: COLLISION - Hit dead end
;-----------------------------------------------------------------------------
COL_STAT_INIT:
            ; Reverse a bit before turning [JESSICA TO UPDATE]
            ; JSR REV_SMALL
            
            ; increment attempt counter for this intersection so next try uses next preference
            LDAA current_intersection
            LDX #intersection_attempts
            LDAB A,X                 ; load current attempt count
            INCB                     ; increment (wrap-around safe if >3)
            STAB A,X                 ; store back
            
            ; Execute 180-degree turn
            JSR  TURN_180
            
            ; Pop previous intersection from stack
            JSR  POP_INTERSECTION
            
            ; Record that this path was wrong
            ; (opposite direction is correct)
            JSR  RECORD_CORRECTION
            
            ; Resume following line back
            LDAA #FLLW_STAT
            STAA robot_state
            RTS

REV_STAT_INIT:
            RTS

TURN_STAT_INIT:
            RTS
;-----------------------------------------------------------------------------
; STATE: COMPLETE - Reached destination
;-----------------------------------------------------------------------------
DONE_STAT_INIT:
            ; Keep motors stopped
            JSR  STOP_MOTORS
            
            ; Check current mode
            LDAA robot_mode
            CMPA #MODE_EXPLORING
            BEQ  BCKTRK_STAT_INIT
            
            ; Already retraced - mission complete!
            ; Stay in COMPLETE state, do nothing
            RTS

BCKTRK_STAT_INIT:
             
             ; Check if rear bumper is pressed
            BRCLR PORTAD0, #$08, WAIT_FOR_REAR_BUMP  ; rear bumper assumed on bit 3
           
            ; Switch to retrace mode
            LDAA #MODE_RETRACING
            STAA robot_mode
            
            ; Reset intersection counter for return journey
            CLR  current_intersection
            
            ; Optional: Wait for bumper press to start return
            ; Or auto-start after delay , IMPLEMENT TO WAIT FOR REAR BUMPER TO BE PRESSED
            LDAA #FLLW_STAT
            STAA robot_state
            RTS
            

WAIT_FOR_REAR_BUMP:
        ; Just return here; BCKTRK will be called again next cycle
        RTS


; ===============================================================
; Update Reading
; ===============================================================

UPDT_READING:
        JSR LED_ON    
        JSR READ_ALL_SENSORS
        JSR LED_OFF
        JSR UPDATE_DECT_FLAGS
        RTS

;============================================================
; SENSOR READING ROUTINES
; READ_ALL_SENSORS
; Reads A,B,C,D,E,F individually into sensor_values[] , reads from left to right(543210)
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
                        STAA TEMP ; and save it into TEMP
                        PULA ; Get the sensor number
                        ASLA ; Shift the selection number left, twice
                        ASLA ;
                        ANDA #%00011100 ; Clear irrelevant bit positions
                        
                        ORAA TEMP ; OR it into the sensor bit positions
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
; SENSOR CONVERSION
; Converts sensor_values[] into ABCDEF pattern bits
;============================================================

SENSOR_CONVERT:
        CLR sensor_pattern
        LDY #sensor_values

        ;----- A (bit5) -----
        LDAA 0,Y
        CMPA #BOW_THRESH
        BLS TP_B                ; BRANCH IF LOWER OR THE SAME
        LDAA sensor_pattern
        ORAA #%00100000        ; bit 5
        STAA sensor_pattern

TP_B:
        ;----- B (bit4) -----
        LDAA 1,Y
        CMPA #PORT_THRESH
        BLS TP_C
        LDAA sensor_pattern
        ORAA #%00010000        ; bit 4, $10
        STAA sensor_pattern

TP_C:
        ;----- C (bit3) -----
        LDAA 2,Y
        CMPA #MIDD_THRESH
        BLS TP_D
        LDAA sensor_pattern
        ORAA #%00001000        ; bit 3 , $08
        STAA sensor_pattern

TP_D:
        ;----- D (bit2) -----
        LDAA 3,Y
        CMPA #STARBD_THRESH
        BLS TP_E
        LDAA sensor_pattern
        ORAA #%00000100        ; bit 2 $04
        STAA sensor_pattern

TP_E:
        ;----- E (bit1) -----
        LDAA 4,Y
        CMPA #EF_THRESH
        BLS TP_F
        LDAA sensor_pattern
        ORAA #%00000010        ; bit 1 , $02
        STAA sensor_pattern

TP_F:
        ;----- F (bit0) -----
        LDAA 5,Y
        CMPA #RLINE_THRESH
        BLS TP_DONE
        LDAA sensor_pattern
        ORAA #%00000001        ; bit 0 , $01
        STAA sensor_pattern

TP_DONE:
            ; Stored to be used to update detection flags
             LDAA 0,Y
            STAA BOW_SENSOR
            LDAA 4,Y
            STAA LINE_SENSOR
            LDAA 1,Y
            STAA PORT_SENSOR
            LDAA 2,Y
            STAA MIDD_SENSOR
            LDAA 3,Y
            STAA STARBD_SENSOR
            
            RTS
 
;=============================================================================
; BUMPER DETECTION - DIGITAL INPUT METHOD
; Checks PORTAD0 bits as configured digital inputs:
;   Bit 2 = Bow bumper   (0 = pressed, 1 = not pressed)
;   Bit 3 = Stern bumper (0 = pressed, 1 = not pressed)
;
; Note: Bumpers are active LOW (0 when pressed)
;=============================================================================
CHECK_BUMPERS:
            ; Check bow bumper (bit 2)
            BRCLR PORTAD0,$04,BOW_HIT      ; If bit 2 = 0, bumper pressed, MAY CHANGE BRCLR TO BRSET
            
            ; Check stern bumper (bit 3)
            BRCLR PORTAD0,$08,STERN_HIT    ; If bit 3 = 0, bumper pressed
            
            RTS                              ; No bumpers pressed

;-----------------------------------------------------------
; Bow bumper pressed ? collision/dead end
;-----------------------------------------------------------
BOW_HIT:
            LDAA #COL_STAT
            STAA robot_state
            JSR  STOP_MOTORS
            RTS

;-----------------------------------------------------------
; Stern bumper pressed ? reached destination
;-----------------------------------------------------------
STERN_HIT:
            LDAA #DONE_STAT
            STAA robot_state
            JSR  STOP_MOTORS
            
            ; Switch to retracing mode
            LDAA #MODE_RETRACING
            STAA robot_mode
            RTS

;=============================================================================
; LINE FOLLOWING CONTROL
;=============================================================================
LINE_FOLLOW_CONTROL:
            ; Proportional steering based on line_value
            LDAA line_value
            SUBA #LINE_CENTER           ; Error (signed)
            
            ; Check if error is positive or negative
            BMI  LFC_NEGATIVE_ERROR     ; BRANCH IF MINUS
            
LFC_POSITIVE_ERROR:
            ; Positive error = too far left THEN turn left
            CMPA #LINE_DEADBAND
            BLS  LFC_CENTERED           ; Within deadband, BRANCH IF SAME OR LOWER
            
            ; Determine correction strength
            CMPA #20
            BHI  LFC_HARD_LEFT
            
            ; Slight left turn
            LDAA #MOTOR_MED
            LDAB #MOTOR_FAST
            JSR  SET_MOTOR_SPEEDS
            RTS
            
LFC_HARD_LEFT:
            LDAA #MOTOR_SLOW
            LDAB #MOTOR_FAST
            JSR  SET_MOTOR_SPEEDS
            RTS

LFC_NEGATIVE_ERROR:
            ; Negative error = too far right THEN turn right
            NEGA                        ; Make positive for comparison
            CMPA #LINE_DEADBAND
            BLS  LFC_CENTERED
            
            CMPA #20
            BHI  LFC_HARD_RIGHT         ; BRANCH IF HIGHER
            
            ; Slight right turn
            LDAA #MOTOR_FAST
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            RTS
            
LFC_HARD_RIGHT:
            LDAA #MOTOR_FAST
            LDAB #MOTOR_SLOW
            JSR  SET_MOTOR_SPEEDS
            RTS

LFC_CENTERED:
            ; On target, full speed ahead
            LDAA #MOTOR_FAST
            LDAB #MOTOR_FAST
            JSR  SET_MOTOR_SPEEDS
            RTS
;=============================================================================
; THRESHOLD_PATTERN - wrapper that converts sensors and sets junction_type
;=============================================================================
THRESHOLD_PATTERN:
        
        JSR SENSOR_CONVERT
        JSR UPDATE_DECT_FLAGS
        RTS
        
 UPDATE_DECT_FLAGS:
        
        LDAA sensor_pattern
        STAA temp_a        ; keep pattern in temp_a
        
        ; Using copies
        LDAA LINE_SENSOR
        ADDA STARBD_SENSOR
        LSRA
        STAA line_value
        
        ; Determine junction_type:
        LDAA temp_a
        ANDA #%00100000         ; A
        BNE IS_JUNCTION
        LDAA temp_a
        ANDA #%00001000         ; C
        BEQ CHECK_EF_COMBO
        
        ; C set: check E or F
        LDAA temp_a
        ANDA #%00000011
        BNE IS_JUNCTION
        BRA NOT_JUNCTION
        
CHECK_EF_COMBO:
        
        LDAA temp_a
        ANDA #%00000011
        CMPA #%00000011
        BEQ IS_JUNCTION         ; both E and F
        
        LDAA temp_a
        ANDA #%00011110
        CMPA #2
        BLS NOT_JUNCTION        
        
        ; If multiple middle sensors triggered, treat as junction
        ; We'll approximate by checking if any two bits of mask are set:
        PSHA
        LDAA temp_a
        ANDB #%00011110
        DECA
        
        ; If above mask non-zero and not single bit then treat as junction
        LDAA temp_a
        ANDA #%00001000 ; C
        BNE CHECK_D
        LDAA temp_a
        ANDA #%00000100 ; D
        BNE CHECK_E
        LDAA temp_a
        ANDA #%00000010 ; E
        BNE CHECK_F
        BRA NOT_JUNCTION
        
CHECK_D:
        LDAA temp_a
        ANDA #%00000100
        BNE IS_JUNCTION
        BRA IS_JUNCTION
CHECK_E:
        LDAA temp_a
        ANDA #%00000010
        BNE IS_JUNCTION
        BRA NOT_JUNCTION
CHECK_F:
        LDAA temp_a
        ANDA #%00000001
        BNE IS_JUNCTION
        BRA NOT_JUNCTION

IS_JUNCTION:
        LDAA #1
        STAA junction_type
        BRA UDF_DONE

NOT_JUNCTION:
        LDAA #0
        STAA junction_type

UDF_DONE:
        RTS

; MOVE_FORWARD - move forward a short distance (FRWD_DIS) using wheel counts
MOVE_FORWARD:
        ; Set forward directions (both motors forward)
        BCLR PORTA, #$01   ; Port forward bit (PA0 = 0 for forward)
        BCLR PORTA, #$02   ; Starboard forward bit (PA1 = 0)
        ; set speeds to moderate
        LDAA #MOTOR_MED
        LDAB #MOTOR_MED
        JSR SET_MOTOR_SPEEDS

        CLR rotation_count_port
        CLR rotation_count_stbd

;NOT REALLY SURE HOW TO COMPLETE THIS @JESSICA

; UPDATE_DIRECTION_TABLE - update current_direction according to motion
; (for straight moves direction remains same; pivot/turn routines update direction themselves)
UPDATE_DIRECTION_TABLE:
        ; no-op here (kept for future expansion)
        RTS
;=============================================================================
; JUNCTION HANDLER LOGIC
;=============================================================================
HANDLE_JUNCTION:
    ; Determine which turn to attempt at this junction
    LDX #required_turn_table
    LDAA current_intersection
    ASLA
    ASLA           ; multiply by 4
    LDAB #0                     ; clear high byte of offset
    ABX                         ; X = required_turn_table + A
    JMP 0,X                     ; jump to handler

    LDAB next_path
    ABX            ; X = base + next_path offset
    LDAA 0,X       ; load turn type

    CMPA #TURN_RIGHT
    BEQ DO_RIGHT
    CMPA #TURN_STRAIGHT
    BEQ DO_STRAIGHT
    CMPA #TURN_LEFT
    BEQ DO_LEFT
    CMPA #TURN_REVERSE
    BEQ DO_REVERSE
    BRA END_HANDLE      ; No valid turn found

; Perform the turn
DO_LEFT:
    JSR PIVOT_LEFT_90
    JSR UPDATE_DIRECTION_TABLE
    BRA CHECK_BUMP

DO_STRAIGHT:
    ; Straight = move forward a little
    JSR MOVE_FORWARD
    JSR UPDATE_DIRECTION_TABLE
    BRA CHECK_BUMP

DO_RIGHT:
    JSR PIVOT_RIGHT_90
    JSR UPDATE_DIRECTION_TABLE
    BRA CHECK_BUMP

DO_REVERSE:
    JSR TURN_180
    JSR UPDATE_DIRECTION_TABLE
    BRA CHECK_BUMP

; Check bumpers
CHECK_BUMP:
    JSR CHECK_BUMPERS
    LDAA robot_state
    CMPA #COL_STAT
    BEQ NO_BUMP_DETECTED

    ; Collision detected ? set collision state
    LDAA #COL_STAT
    STAA robot_state
    RTS

NO_BUMP_DETECTED:
    ; Record intersection if not already solved
    LDX #intersection_solved
    LDAB current_intersection
    LDAA 0,X
    CMPA #1
    BEQ SKIP_RECORD

    ; Store direction in path stack
    LDX #path_stack
    LDAB path_stack_ptr
    LDAA current_direction
    STAA B,X       ; store at stack

    INC path_stack_ptr


    ; Increment intersection counter
    INC current_intersection

    ; Mark as solved    
    LDX #intersection_solved
    LDAB current_intersection
    LDAA #1
    STAA B,X


SKIP_RECORD:
    ; Increment next_path for next attempt at this junction
    INC next_path
    RTS

END_HANDLE:
    RTS

;=============================================================================
; MOTOR CONTROL
;=============================================================================
SET_MOTOR_SPEEDS:
            ; Input: A = port speed, B = starboard speed
            STAA port_motor_speed
            STAB stbd_motor_speed
            
            ; Simple on/off control (enhance with real PWM later)
            CLR  temp_a
            
            ; Port motor
            LDAA port_motor_speed
            CMPA #MOTOR_MIN
            BLO  SMS_CHECK_STBD         ;BRANCH IF LOWER
            LDAA #$10                   ; PT4 on
            STAA temp_a
            
SMS_CHECK_STBD:
            ; Starboard motor
            LDAA stbd_motor_speed
            CMPA #MOTOR_MIN
            BLO  SMS_APPLY
            LDAA temp_a
            ORAA #$20                   ; PT5 on
            STAA temp_a
            
SMS_APPLY:
            LDAA temp_a
            STAA PTT
            RTS

STOP_MOTORS:
            CLR  port_motor_speed
            CLR  stbd_motor_speed
            BCLR PTT, #$30            ; Clear PT4,PT5
            RTS

;=============================================================================
; TURN ROUTINES
;=============================================================================

PIVOT_LEFT_90:
            ; Port REV, StRbd FWD
            BSET PORTA, #$01            ; PA0=1 (Port REV)
            BCLR PORTA, #$02            ; PA1=0 (Stbd FWD)
            
            ; Set speeds
            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            
            ; Reset counters
            CLR  rotation_count_port
            CLR  rotation_count_stbd
            
PL90_LOOP:
            LDAA rotation_count_port
            CMPA #COUNTS_90_DEG
            BHS  PL90_DONE             ;BRANCH IF HIGHER OR SAME
            BRA  PL90_LOOP
            
PL90_DONE:
            JSR  STOP_MOTORS
            
            ; Update direction (rotate counterclockwise)
            LDAA current_direction
            BEQ  PL90_SET_WEST
            DECA
            STAA current_direction
            RTS
PL90_SET_WEST:
            LDAA #WEST
            STAA current_direction
            RTS

PIVOT_RIGHT_90:
            ; Port FWD, Stbd REV
            BCLR PORTA, #%00000001
            BSET PORTA, #$02
            

            LDAA #MOTOR_MED
            LDAB #MOTOR_MED
            JSR  SET_MOTOR_SPEEDS
            
            CLR  rotation_count_port
            CLR  rotation_count_stbd
            
PR90_LOOP:
            LDAA rotation_count_stbd
            CMPA #COUNTS_90_DEG
            BHS  PR90_DONE          ; BRANCH IF HIGHER OR THE SAME
            BRA  PR90_LOOP
            
PR90_DONE:
            JSR  STOP_MOTORS
            
            ; Update direction (rotate clockwise)
            LDAA current_direction
            CMPA #WEST
            BEQ  PR90_SET_NORTH
            INCA
            STAA current_direction
            RTS

PR90_SET_NORTH:
            CLR  current_direction
            RTS

TURN_180:
            ; Pivot 180 degrees
            JSR  PIVOT_RIGHT_90
            JSR  PIVOT_RIGHT_90
            RTS

;=============================================================================
; MAZE MANAGEMENT
;=============================================================================
PUSH_INTERSECTION:
            LDAA path_stack_ptr
            CMPA #MAX_INTERSECTIONS
            BHS  PUSH_OVERFLOW
            
            ; Store current_intersection
            LDX  #path_stack
            LDAB path_stack_ptr
            LDAA current_intersection
            STAA B,X             
            INC  path_stack_ptr

PUSH_OVERFLOW:
            RTS

POP_INTERSECTION:
            LDAA path_stack_ptr
            BEQ  POP_EMPTY            
            DECA
            STAA path_stack_ptr
            
            LDX #path_stack
            LDAB path_stack_ptr
            LDAA B,X
            STAA current_intersection
            RTS

POP_EMPTY:
            RTS

RECORD_CORRECTION:
            ; Mark current intersection as solved
            LDAA current_intersection
            LDX  #intersection_solved
            LDAB #1
            STAB A,X
            
            ; Store correct direction
            ; (Opposite of tried_direction)
            LDX  #tried_direction
            LDAB current_intersection
            LDAA B,X
            
            ; If tried LEFT, correct is RIGHT (and vice versa)
            CMPA #TURN_LEFT
            BEQ  RC_SET_RIGHT
            CMPB #TURN_RIGHT
            BEQ RC_SET_LEFT
            CMPB #TURN_STRAIGHT
            BEQ RC_SET_STRAIGHT
            CMPB #TURN_REVERSE
            BEQ RC_SET_REVERSE
            BRA RC_DONE


RC_SET_RIGHT:                             ;RIGHT DIRECTION
            LDAA #TURN_RIGHT
            BRA RC_STORE

RC_SET_LEFT:
            LDAA #TURN_LEFT
            BRA RC_STORE

RC_SET_STRAIGHT:
            LDAA #TURN_STRAIGHT
            BRA RC_STORE

RC_SET_REVERSE:
            LDAA #TURN_REVERSE
            BRA RC_STORE


RC_STORE:
            LDX  #maze_solution
            LDAB current_intersection
            STAA B,X
            BRA RC_DONE

RC_DONE:
        RTS

;=============================================================================
; LOOKUP TABLE FUNCTIONS
;=============================================================================
GET_REQUIRED_TURN:
            ; Input: A = current_dir, B = desired_dir
            ; Output: A = turn_type
            PSHB
            LDAB #4
            MUL                         ; D = current * 4
            PULB                        ; Get desired back
            ABA                         ; A = index
            TAB                         ; Move to B for indexing
            LDX  #required_turn_table
            LDAA B,X
            RTS

;********************************************************************************************
;* INTERRUPT SERVICE ROUTINE 1  /  Port wheel counter                                                            *
;********************************************************************************************
IC0_ISR:           
                MOVB  #$01,TFLG1                ; clear the C0F input capture flag
                INC   CNT1                    ; increment COUNT1
                RTI
;********************************************************************************************
;* INTERRUPT SERVICE ROUTINE 2   / Starboard wheel counter                                                           *
;********************************************************************************************
IC1_ISR:          
                MOVB  #$02,TFLG1                ; clear the C1F input capture flag
                INC   CNT2                    ; increment COUNT2 
                RTI

; utility subroutines  , FROM GOTTEN FRO PREVIOUS LABS
;*******************************************************************
;* Initialization of the LCD: 4-bit data width, 2-line display,    *
;* turn on display, cursor and blinking off. Shift cursor right.   *
;*******************************************************************

initLCD     BSET DDRB,%11111111               ; configure pins PS7,PS6,PS5,PS4 for output
            BSET DDRJ,%11000000                ; configure pins PE7,PE4 for output
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

;****************************************************************
BCD2ASC     LDAA  #0               ; Initialize the blanking flag
            STAA NO_BLANK

C_TTHOU     LDAA TEN_THOUS         ;Check the �ten_thousands� digit
            ORAA NO_BLANK
            BNE NOT_BLANK1

ISBLANK1    LDAA #' '             ; It's blank
            STAA TEN_THOUS        ;so store a space
            BRA  C_THOU           ;and check the �thousands� digit

NOT_BLANK1  LDAA TEN_THOUS        ;Get the �ten_thousands� digit
            ORAA #$30             ;Convert to ascii
            STAA TEN_THOUS
            LDAA #$1              ;Signal that we have seen a �non-blank� digit
            STAA NO_BLANK

C_THOU      LDAA THOUSANDS        ;Check the thousands digit for blankness
            ORAA NO_BLANK         ;If it�s blank and �no-blank� is still zero
            BNE  NOT_BLANK2

ISBLANK2    LDAA  #' '             ; Thousands digit is blank
            STAA THOUSANDS         ;so store a space
            BRA  C_HUNS            ;and check the hundreds digit

NOT_BLANK2  LDAA THOUSANDS         ;(similar to �ten_thousands� case)
            ORAA #$30
            STAA THOUSANDS
            LDAA #$1
            STAA NO_BLANK

C_HUNS      LDAA HUNDREDS           ;Check the hundreds digit for blankness
            ORAA NO_BLANK           ;If it�s blank and �no-blank� is still zero
            BNE NOT_BLANK3

ISBLANK3    LDAA  #' '             ; Hundreds digit is blank
            STAA HUNDREDS          ;so store a space
            BRA C_TENS             ;and check the tens digit

NOT_BLANK3  LDAA HUNDREDS          ;(similar to �ten_thousands� case)
            ORAA #$30
            STAA HUNDREDS
            LDAA #$1
            STAA NO_BLANK

C_TENS      LDAA TENS               ;Check the tens digit for blankness
            ORAA NO_BLANK           ;If it�s blank and �no-blank� is still zero
            BNE NOT_BLANK4  ;

ISBLANK4    LDAA  #' '             ; Tens digit is blank
            STAA TENS              ;so store a space
            BRA C_UNITS            ;and check the units digit

NOT_BLANK4  LDAA TENS              ;(similar to �ten_thousands� case)
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
;------------------------------------------------           
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
                BRA   UPDT_DISPL_EXIT           ; Display 'S' on LCD, 53 IS S IN ASCII CODE
        
        sternON: LDAA  #$53                     ; ""
                JSR   putcLCD                   ; ""

UPDT_DISPL_EXIT RTS                             ; and exit                

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