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
MOTOR_STOP      EQU 0
MOTOR_MIN       EQU 70               ; Calibrate: minimum to overcome friction
MOTOR_SLOW      EQU 102              ; $46
MOTOR_MED       EQU 153              ; $99
MOTOR_FAST      EQU 200              ; Calibrate: max safe speed , $C8
MOTOR_MAX       EQU 255              ; $FF

;WHEEL COUNTS (DISTANCE UNITS IN ENCODER COUNTS) ; take out
INCR_DIS         EQU   300                       ; INCREMENT distance
FRWD_DIS         EQU   2000                      ; FORWARD distance
REVR_DIS         EQU   1000                      ; REVERSE distance
STRT_DIS         EQU   1000                      ; STRAIGHT distance
TURN_DIS         EQU   13000                     ; TURN distance
UTURN_DIS        EQU   15000                     ; U-TURN distance

; Sensor Selection (PA2-PA4 values) ; change
SLCT_LINE_LEFT      EQU 0                ; E differential, Line sensor
SLCT_LINE_RIGHT     EQU 1                ; F differential, Line sensor
SLCT_A              EQU 2                ; Bow absolute, Front sensor
SLCT_B              EQU 3                ; Port absolute, Port sensor
SLCT_C              EQU 4                ; Mid absolute, Middle Sensor
SLCT_D              EQU 5                ; Starboard absolute, Starboard sensor

; Sensor Thresholds (CALIBRATE THESE!)  , 
DARK_THRESHOLD      EQU $99              ; Above = dark line , 153  ,take out
BOW_THRESHOLD       EQU $99              ; SENSOR A , 153
PORT_THRESHOLD      EQU $99              ; SENSOR B , 153
MIDD_THRESHOLD      EQU $99              ; SENSOR C , 153
STARBD_THRESHOLD    EQU $99              ; SENSOR D , 153
LLINE_THRESHOLD     EQU $60              ; SENSOR E  , 96
RLINE_THRESHOLD     EQU $B4              ; SENSOR F , 180

;line following tracking
LINE_CENTER         EQU $80 ; 128
LINE_DEADBAND       EQU 5

; Timing
SETTLE_DELAY    EQU 23               ; ms to wait for CdS settling
COUNTS_90_DEG   EQU 23               ; Wheel counts for 90° turn

; State Machine States
STATE_INIT          EQU 0 ; start 
STATE_FOLLOWING     EQU 1    ;st_fllw
STATE_AT_JUNCTION   EQU 2     ;st_junc
STATE_TURNING       EQU 3      ;st_turn
STATE_COLLISION     EQU 4     ;st_col
STATE_REVERSING     EQU 5    ;st_rev
STATE_COMPLETE      EQU 6     ;st_end
STATE_RETRACING     EQU 7     ;st_bck

; Maze Constants
MAX_INTERSECTIONS   EQU 7
NO_INTERSECTION     EQU $FF
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
NO_TURN             EQU $FF

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
NO_BLANK            ds.b 1          ; Used in ’leading zero’ blanking by BCD2ASC
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


; Sensor Data
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

CNT1          DC.W  0                         ; initialize 2-byte CNT1 to $0000
CNT2          DC.W  0                         ; initialize 2-byte CNT2 to $0000

; Motor Control            , implement
port_motor_speed    DS.B 1
stbd_motor_speed    DS.B 1

; Wheel Counters , 16 Bits to avoid overflow
rotation_count_port DS.W 1
rotation_count_stbd DS.W 1

; Temporaries
temp_a              DS.B 1
temp_b              DS.B 1
TEMP                DS.B 1            

;PATH DETECTION
                               
STRTLINE          DC.B  0                         ; Straight line pattern
CRSJUNC           DC.B  0                         ; Cross junction pattern
LTURN             DC.B  0                         ; Left turn pattern
RTURN             DC.B  0                         ; Right turn pattern
TJUNC             DC.B  0                         ; T-junction pattern
LTJUNC            DC.B  0                         ; Left T-junction pattern
RTJUNC            DC.B  0                         ; Right T-junction pattern
ENDLINE           DC.B  0                         ; End of line pattern

RETURN_PATH      DC.B  0                         ; RETURN (TRUE = 1, FALSE = 0)  , NOT USED, MAY REMOVE
NEXT_DIR         DC.B  1                         ; Next direction instruction   , NOT USED, MAY REMOVE

;FOR PATTERN DETECTION
LINE_SENSOR     DC.B  $0                        ; E-F, (LINE ) Storage for guider sensor readings
BOW_SENSOR      DC.B  $0                        ; A, (FRONT) Initialized to test values
PORT_SENSOR     DC.B  $0                        ; B, (LEFT )
MIDD_SENSOR     DC.B  $0                        ; C, (MIDDLE)
STARBD_SENSOR   DC.B  $0                        ; D, (RIGHT)


;=============================================================================
; PROGRAM ENTRY POINT / CODE SECTION  
;=============================================================================
Entry:
_Startup:
            ORG $4000
                                     ; Disable interrupts during init
            LDS  #$4000                 ; Initialize stack pointer

            ; Initialize Hardware
            JSR  INIT_PORTS
            
            JSR   initAD                    ; Initialize ATD converter                  I                                                ;                                           A
            
            JSR   initLCD                   ; Initialize the LCD                        L
            JSR   clrLCD                    ; Clear LCD & home cursor   
           
            JSR  INIT_TCNT
           
            ; Initialize Variables
                     
            JSR  INIT_MAZE_DATA
            
            ; Safety: Motors off
            JSR  STOP_MOTORS
                                                          ;                                           T
            ; Set initial state
            LDAA #STATE_INIT
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
 
                JSR   READ_ALL_SENSORS
                JSR   UPDT_READING              ;                                           M
                JSR   UPDT_DISP                ;                                           A
                LDAA  robot_state
                JSR   CHECK_BUMPERS                ;                                           I
                JSR   DISPATCHER                ;                                           N
                BRA   MainLoop   



;*******************************************************************
; data section
;*******************************************************************

msg1        dc.b "Battery volt ",0
msg2        dc.b "State ",0
msg3        dc.b "Sensor ",0
msg4        dc.b "Bumper ",0

tab         dc.b "STATE_INIT",0
            dc.b "STATE_FOLLOWING",0
            dc.b "STATE_AT_JUNCTION",0
            dc.b "STATE_TURNING",0
            dc.b "STATE_COLLISION",0
            dc.b "STATE_REVERSING",0
            dc.b "STATE_COMPLETE",0
            dc.b "STATE_RETRACING",0
 
 ;=============================================================================
; CODE SECTION - LOOKUP TABLES and data structure
;=============================================================================
           

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
STRBDFWD         BCLR  PORTA,%00000010
                RTS

;+------------------------------------------------------------------------------------------+
;| Starboard (Right) Motor REV                                                              |
;+------------------------------------------------------------------------------------------+
STRBDREV         BSET  PORTA,%00000010
                RTS

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
;********************************************************************************************
;* NEW STATES DISPATCHER / STATE MACHINE                                                                   *     
;*******************************************************************
DISPATCHER:
            LDAA robot_state
            
            CMPA #STATE_INIT
            BEQ  STATE_INIT_HANDLER
            
            CMPA #STATE_FOLLOWING
            BEQ  STATE_FOLLOWING_HANDLER
            
            CMPA #STATE_AT_JUNCTION
            BEQ  STATE_JUNCTION_HANDLER
            
            CMPA #STATE_COLLISION
            BEQ  STATE_COLLISION_HANDLER
            
            CMPA #STATE_COMPLETE
            BEQ  STATE_COMPLETE_HANDLER
            
            ; Default: unknown state - stop motors for safety
            JSR  STOP_MOTORS
            RTS

;-----------------------------------------------------------------------------
; STATE: INIT - Wait for start
;-----------------------------------------------------------------------------
STATE_INIT_HANDLER:
            ; Option 1: Auto-start
            ;LDAA #STATE_FOLLOWING
            ;STAA robot_state
            ;RTS
            
            ; Option 2: Wait for bumper press (uncomment if desired)
             BRCLR PORTAD0,#$04,START_PRESSED
             RTS
             START_PRESSED:
                 LDAA #STATE_FOLLOWING
                STAA robot_state
                 RTS

;-----------------------------------------------------------------------------
; STATE: FOLLOWING - Line tracking and intersection detection
;-----------------------------------------------------------------------------
STATE_FOLLOWING_HANDLER:
            ; Check for junction
            JSR  DETECT_JUNCTION
            TST  temp_a                     ; Result flag in temp_a
            BNE  FOUND_JUNCTION
            
            ; No junction - do line following
            JSR  LINE_FOLLOW_CONTROL
            RTS

FOUND_JUNCTION:
            LDAA #STATE_AT_JUNCTION
            STAA robot_state
            RTS

;-----------------------------------------------------------------------------
; STATE: AT_JUNCTION - Decision point
;-----------------------------------------------------------------------------
STATE_JUNCTION_HANDLER:
            ; Decide which way to turn based on mode
            JSR  DECIDE_TURN_DIRECTION
            
            ; DECIDE_TURN_DIRECTION should:
            ; 1. Check if intersection is solved
            ; 2. If exploring: choose direction, push to stack
            ; 3. If retracing: read from solution array
            ; 4. Execute turn
            ; 5. Transition back to STATE_FOLLOWING
            
            RTS

;-----------------------------------------------------------------------------
; STATE: COLLISION - Hit dead end
;-----------------------------------------------------------------------------
STATE_COLLISION_HANDLER:
            ; Execute 180-degree turn
            JSR  TURN_180
            
            ; Pop previous intersection from stack
            JSR  POP_INTERSECTION
            
            ; Record that this path was wrong
            ; (opposite direction is correct)
            JSR  RECORD_CORRECTION
            
            ; Resume following line back
            LDAA #STATE_FOLLOWING
            STAA robot_state
            RTS

STATE_REVERSING_HANDLER:
            RTS

STATE_LEAVING_HANDLER:
            RTS

STATE_TURNING_HANDLER:
            RTS
;-----------------------------------------------------------------------------
; STATE: COMPLETE - Reached destination
;-----------------------------------------------------------------------------
STATE_COMPLETE_HANDLER:
            ; Keep motors stopped
            JSR  STOP_MOTORS
            
            ; Check current mode
            LDAA robot_mode
            CMPA #MODE_EXPLORING
            BEQ  STATE_RETRACING_HANDLER
            
            ; Already retraced - mission complete!
            ; Stay in COMPLETE state, do nothing
            RTS

STATE_RETRACING_HANDLER:
             
             ; Check if rear bumper is pressed
            BRCLR PORTAD0, #$08, WAIT_FOR_REAR_BUMP  ; rear bumper assumed on bit 3
           
            ; Switch to retrace mode
            LDAA #MODE_RETRACING
            STAA robot_mode
            
            ; Reset intersection counter for return journey
            CLR  current_intersection
            
            ; Optional: Wait for bumper press to start return
            ; Or auto-start after delay , IMPLEMENT TO WAIT FOR REAR BUMPER TO BE PRESSED
            LDAA #STATE_FOLLOWING
            STAA robot_state
            RTS
            

WAIT_FOR_REAR_BUMP:
        ; Just return here; STATE_RETRACING will be called again next cycle
        RTS

;============================================================
; SENSOR READING ROUTINES
; READ_ALL_SENSORS
; Reads A,B,C,D,E,F individually into sensor_values[] , reads from left to right(543210)
;=============================================================================

READ_ALL_SENSORS:
        LDY  #sensor_values     ; Y ? start of sensor array

        ;----- Read A -----
        LDAA #SLCT_A
        JSR  SELECT_AND_READ
        STAA 0,Y               ; A

        ;----- Read B -----
        LDAA #SLCT_B
        JSR  SELECT_AND_READ
        STAA 1,Y               ; B

        ;----- Read C -----
        LDAA #SLCT_C
        JSR  SELECT_AND_READ
        STAA 2,Y               ; C

        ;----- Read D -----
        LDAA #SLCT_D
        JSR  SELECT_AND_READ
        STAA 3,Y               ; D

        ;----- Read E (left line) -----
        LDAA #SLCT_LINE_LEFT
        JSR  SELECT_AND_READ
        STAA 4,Y               ; E

        ;----- Read F (right line) -----
        LDAA #SLCT_LINE_RIGHT
        JSR  SELECT_AND_READ
        STAA 5,Y               ; F

        LDAA 0,Y
        STAA BOW_SENSOR
        LDAA 1,Y
        STAA PORT_SENSOR
        LDAA 2,Y
        STAA MIDD_SENSOR
        LDAA 3,Y
        STAA STARBD_SENSOR
        LDAA 4,Y
        STAA LINE_SENSOR

      ; Disable LEDs
        BCLR PORTA, #$20
            
        RTS

SELECT_AND_READ:
            ; Input: A = sensor select code (0-5)
            ; Output: A = averaged sensor reading
            
            ; Save sensor code
            PSHA
            
            ; Configure PORTA for this sensor
            ; Preserve motor direction bits (PA0,PA1)
            LDAA PORTA
            ANDA #%11000011             ; Keep bits 7-6,1-0 , C3
            STAA temp_a
            
            ; Get sensor code and shift to PA2-PA4
            PULA
            ASLA
            ASLA
            ANDA #%00011100             ; Isolate PA2-PA4,  1C
            
            ; Combine with preserved bits
            ORAA temp_a
            
            ; Enable LED (PA5)
            ORAA #%00100000       ; 20
            STAA PORTA
            
            ; Wait for CdS settling (20ms)
            LDY  #400                   ; 400 * 50us = 20ms
            JSR  del_50us
            
            ; Start A/D conversion on AN1
            MOVB #$81, ATDCTL5          ; Right justified, single, AN1
            
SAR_WAIT:
            BRCLR ATDSTAT0, #$80, SAR_WAIT
            
            ; Average 4 samples
            LDAA ATDDR0L
            LDAB ATDDR1L
            ABA            ; A = A + B
            ADDA ATDDR2L
            ADDA ATDDR3L
            LSRA
            LSRA                        ; Divide by 4
            STAA CURRENT_GUIDER_VALUE    ; STORES AVERAGE VALUE / RESULT
            
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
        CMPA #BOW_THRESHOLD
        BLS TP_B                ; BRANCH IF LOWER OR THE SAME
        LDAA sensor_pattern
        ORAA #%00100000        ; bit 5
        STAA sensor_pattern

TP_B:
        ;----- B (bit4) -----
        LDAA 1,Y
        CMPA #PORT_THRESHOLD
        BLS TP_C
        LDAA sensor_pattern
        ORAA #%00010000        ; bit 4, $10
        STAA sensor_pattern

TP_C:
        ;----- C (bit3) -----
        LDAA 2,Y
        CMPA #MIDD_THRESHOLD
        BLS TP_D
        LDAA sensor_pattern
        ORAA #%00001000        ; bit 3 , $08
        STAA sensor_pattern

TP_D:
        ;----- D (bit2) -----
        LDAA 3,Y
        CMPA #STARBD_THRESHOLD
        BLS TP_E
        LDAA sensor_pattern
        ORAA #%00000100        ; bit 2 $04
        STAA sensor_pattern

TP_E:
        ;----- E (bit1) -----
        LDAA 4,Y
        CMPA #LLINE_THRESHOLD
        BLS TP_F
        LDAA sensor_pattern
        ORAA #%00000010        ; bit 1 , $02
        STAA sensor_pattern

TP_F:
        ;----- F (bit0) -----
        LDAA 5,Y
        CMPA #RLINE_THRESHOLD
        BLS TP_DONE
        LDAA sensor_pattern
        ORAA #%00000001        ; bit 0 , $01
        STAA sensor_pattern

TP_DONE:
            RTS

;============================================================
; THRESHOLD_PATTERN
; Stores patterns to Names for detection
;============================================================
THRESHOLD_PATTERN:

        ; Cross Junction pattern
        LDAA sensor_pattern
        ANDA #%00111100     ; bit 5,4,3,2 -> A,B,C,D
        CMPA #%00111100
        BEQ  CRSJUNC           


        ; Left T-junction pattern
        LDAA sensor_pattern
        ANDA #%00111000     ; bit 5,4,3 -> A,B,C
        CMPA #%00111000
        BEQ  LTJUNC            


        ; Right T-junction pattern
        LDAA sensor_pattern
        ANDA #%00101100     ; bit 5,3,2 -> A,C,D
        CMPA #%00101100
        BEQ  RTJUNC            

        ; T-junction pattern
        LDAA sensor_pattern
        ANDA #%00011100     ; bit 4,3,2 -> B,C,D
        CMPA #%00011100
        BEQ  TJUNC             


        ; Left turn pattern
        LDAA sensor_pattern
        ANDA #%00011000     ; bit 4,3 -> B,C
        CMPA #%00011000
        BEQ  LTURN             


        ; Right turn pattern
        LDAA sensor_pattern
        ANDA #%00001100     ; bit 3,2 -> C,D
        CMPA #%00001100
        BEQ  RTURN            


        ; Straight line pattern
        LDAA sensor_pattern
        ANDA #%00101000     ; bit 5,3 -> A,C
        CMPA #%00101000
        BEQ  STRTLINE


        ; End of line pattern
        LDAA sensor_pattern
        ANDA #%00000011     ; bit 1,0 -> E,F
        CMPA #%00000011
        BEQ  ENDLINE           

        RTS


; ===============================================================
; Missing user-defined symbols added here
; ===============================================================

UPDT_READING:
        JSR READ_ALL_SENSORS
        JSR UPDATE_DECT_FLAGS
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
            LDAA #STATE_COLLISION
            STAA robot_state
            JSR  STOP_MOTORS     ; reverse 180 and go back to intersection
            RTS

;-----------------------------------------------------------
; Stern bumper pressed ? reached destination
;-----------------------------------------------------------
STERN_HIT:
            LDAA #STATE_COMPLETE
            STAA robot_state
            JSR  STOP_MOTORS
            
            ; Switch to retracing mode
            LDAA #MODE_RETRACING
            STAA robot_mode
            RTS

;=============================================================================
; JUNCTION DETECTION
;=============================================================================
DETECT_JUNCTION:
            ; Check if outrigger sensors (B or D) detect line
            ; While center sensors (A,C) also detect line
            
            LDAA sensor_pattern
            ANDA #%00000110             ; Check A,C
            CMPA #%00000110             ; Both must be dark
            BNE  DJ_NOT_JUNCTION
            
            ; Check if B or D also dark
            LDAA sensor_pattern
            ANDA #%00001001             ; Check B,D
            BEQ  DJ_NOT_JUNCTION
            
            ; Junction detected!
            LDAA #1
            STAA temp_a
            RTS
            
DJ_NOT_JUNCTION:
            CLR  temp_a
            RTS

;=============================================================================
; LINE FOLLOWING CONTROL    / modify line control
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
; TURN DECISION LOGIC
;=============================================================================
DECIDE_TURN_DIRECTION:
            ; Check if this intersection is solved
            LDAA current_intersection
            LDX  #intersection_solved
            LDAB A,X                    ; Indexed load
            CMPB #1
            BEQ  DTD_USE_SOLUTION
            
            ; Not solved - explore
            ; Simple strategy: try left first
            LDAA #TURN_LEFT
            STAA temp_a
            
            ; Record that we tried left
            LDAA current_intersection
            LDX  #tried_direction
            LDAB #TURN_LEFT
            STAB A,X
            
            ; Push intersection on stack
            JSR  PUSH_INTERSECTION
            
            ; Execute turn
            JSR  PIVOT_LEFT_90
            
            ; Increment intersection counter
            INC  current_intersection
            
            ; Return to following
            LDAA #STATE_FOLLOWING
            STAA robot_state
            RTS

DTD_USE_SOLUTION:
            ; Read stored solution
            LDAA current_intersection
            LDX  #maze_solution
            LDAB A,X                    ; Get desired direction
            
            ; Calculate required turn
            LDAA current_direction      ; Current
            ; LDAB already has desired
            JSR  GET_REQUIRED_TURN      ; Returns turn type in A
            
            CMPA #TURN_LEFT
            BEQ  DTD_DO_LEFT
            CMPA #TURN_RIGHT
            BEQ  DTD_DO_RIGHT
            CMPA #TURN_REVERSE
            BEQ  DTD_DO_180
            
            ; Otherwise straight - continue following
            LDAA #STATE_FOLLOWING
            STAA robot_state
            RTS

DTD_DO_LEFT:
            JSR  PIVOT_LEFT_90
            LDAA #STATE_FOLLOWING
            STAA robot_state
            RTS

DTD_DO_RIGHT:
            JSR  PIVOT_RIGHT_90
            LDAA #STATE_FOLLOWING
            STAA robot_state
            RTS

DTD_DO_180:
            JSR  TURN_180
            LDAA #STATE_FOLLOWING
            STAA robot_state
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
            ; Port REV, Stbd FWD
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
            BCLR PORTA, #$01
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
            
            DEC  path_stack_ptr
            LDX  #path_stack
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
            LDAA A,X
            
            ; If tried LEFT, correct is RIGHT (and vice versa)
            CMPA #TURN_LEFT
            BEQ  RC_SET_RIGHT
            
            LDAA #TURN_LEFT
            BRA  RC_STORE

RC_SET_RIGHT:                             ;RIGHT DIRECTION
            LDAA #TURN_RIGHT

RC_STORE:
            LDX  #maze_solution
            LDAB current_intersection
            STAA B,X
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

;=============================================================================
; INITIALIZATION ROUTINES
;=============================================================================
INIT_PORTS:
            ;----------------------------------------
            ; Configure PORTAD: analog inputs
            ;----------------------------------------
            BCLR  DDRAD, #$FF           ; All PORTAD pins as input
            BSET  DDRA,  #$FF
            BSET  DDRT, $30     
                 
                 
            RTS

INIT_TCNT:
            ; Enable timer system
            MOVB #$80, TSCR1            ; TEN=1, TFFCA=1
            
            ; Prescaler and overflow interrupt
            MOVB #$00, TSCR2            ; TOI=1, prescaler
            
            ; Configure IC0,IC1 for wheel counters
            MOVB #$FC, TIOS             ; All input capture initially
            MOVB #$03, TCTL4
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
            LDAB #NO_INTERSECTION

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
            
            RTS

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

C_TTHOU     LDAA TEN_THOUS         ;Check the ’ten_thousands’ digit
            ORAA NO_BLANK
            BNE NOT_BLANK1

ISBLANK1    LDAA #' '             ; It's blank
            STAA TEN_THOUS        ;so store a space
            BRA  C_THOU           ;and check the ’thousands’ digit

NOT_BLANK1  LDAA TEN_THOUS        ;Get the ’ten_thousands’ digit
            ORAA #$30             ;Convert to ascii
            STAA TEN_THOUS
            LDAA #$1              ;Signal that we have seen a ’non-blank’ digit
            STAA NO_BLANK

C_THOU      LDAA THOUSANDS        ;Check the thousands digit for blankness
            ORAA NO_BLANK         ;If it’s blank and ’no-blank’ is still zero
            BNE  NOT_BLANK2

ISBLANK2    LDAA  #' '             ; Thousands digit is blank
            STAA THOUSANDS         ;so store a space
            BRA  C_HUNS            ;and check the hundreds digit

NOT_BLANK2  LDAA THOUSANDS         ;(similar to ’ten_thousands’ case)
            ORAA #$30
            STAA THOUSANDS
            LDAA #$1
            STAA NO_BLANK

C_HUNS      LDAA HUNDREDS           ;Check the hundreds digit for blankness
            ORAA NO_BLANK           ;If it’s blank and ’no-blank’ is still zero
            BNE NOT_BLANK3

ISBLANK3    LDAA  #' '             ; Hundreds digit is blank
            STAA HUNDREDS          ;so store a space
            BRA C_TENS             ;and check the tens digit

NOT_BLANK3  LDAA HUNDREDS          ;(similar to ’ten_thousands’ case)
            ORAA #$30
            STAA HUNDREDS
            LDAA #$1
            STAA NO_BLANK

C_TENS      LDAA TENS               ;Check the tens digit for blankness
            ORAA NO_BLANK           ;If it’s blank and ’no-blank’ is still zero
            BNE NOT_BLANK4  ;

ISBLANK4    LDAA  #' '             ; Tens digit is blank
            STAA TENS              ;so store a space
            BRA C_UNITS            ;and check the units digit

NOT_BLANK4  LDAA TENS              ;(similar to ’ten_thousands’ case)
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

UPDT_DISP      LDAA  #$82                      ; Move LCD cursor to the end of msg1
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
                ADDD  #600                      ; AccD = 1st result x 39 + 600
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
                BRA   stern_bump                ; Display 'B' on LCD
         bowON: LDAA  #$42                      ; ""
                JSR   putcLCD                   ; ""
          
    stern_bump: BRCLR PORTAD0,#%00001000,sternON; If REV_BUMP, then
                LDAA  #$20                      ;
                JSR   putcLCD                   ;
                BRA   UPDT_DISPL_EXIT           ; Display 'S' on LCD
       sternON: LDAA  #$53                      ; ""
                JSR   putcLCD                   ; ""
UPDT_DISP_EXIT RTS  

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

;=============================================================================
; INTERRUPT VECTORS
;=============================================================================
            ORG  $FFFE
            DC.W Entry                  ; Reset
            
            ORG  $FFEC
            DC.W IC0_ISR                ; Input Capture 0
            
            ORG  $FFEA
            DC.W IC1_ISR                ; Input Capture 1

;*****************************************************************************
; END OF PROGRAM
;*****************************************************************************