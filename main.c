//Andre Vojtenyi
//Hsien-I Tsou

// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "uart.h"
#include "spi.h"
#include "systick.h"
#include "timer.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "i2c_if.h"
#include "timer_if.h"

#include "pin_mux_config.h"

// Adafruit
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "oled_test.h"

// Custom includes
#include "utils/network_utils.h"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//*****************************************************************************
// Interrupt Handlers & Global variable declarations
//*****************************************************************************
#define SYSTICK_WRAP_TICKS      16777216 // 2^24

#define SYS_CLK             80000000
#define MILLISECONDS_TO_TICKS(ms)   ((SYS_CLK/1000) * (ms))
#define TICKS_TO_MILLISECONDS(ts)   (((double) 1000/(double) SYS_CLK) * (double) (ts))

#define NONE  0
#define SHORT   1
#define LONG    2

#define DATE                5    /* Current Date */
#define MONTH               6     /* Month 1-12 */
#define YEAR                2025  /* Current year */
#define HOUR                12    /* Time - hours */
#define MINUTE              50    /* Time - minutes */
#define SECOND              0     /* Time - seconds */


#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "a1x1eeamluscnq-ats.iot.us-east-1.amazonaws.com"
#define GOOGLE_DST_PORT       8443


#define POSTHEADER "POST /things/avojtenyi_CC3200Board/shadow HTTP/1.1\r\n"
#define GETHEADER "GET /things/avojtenyi_CC3200Board/shadow HTTP/1.1\r\n"
#define HOSTHEADER "Host: a1x1eeamluscnq-ats.iot.us-east-1.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA_F "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"var\" :\""

#define DATA_B           "\"\r\n"                                            \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"

#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define MAX_STRING_LENGTH    80

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define ADDRESS                 "18"
#define X_OFFSET                "3"
#define Y_OFFSET                "5"
#define LENGTH                  "1"

#define PADDLE_LENGTH           20
#define PADDLE_WIDTH            1
#define PLAYER1_PADDLE_X_POS    10
#define PLAYER2_PADDLE_X_POS    118
#define RADIUS                  2

#define INNER_DASH_WIDTH        1
#define INNER_DASH_HEIGHT       4
#define INNER_DASH_X_POS        64

#define WINNING_SCORE           3 //11

volatile int g_iCounter = 0;
char cString[MAX_STRING_LENGTH+100];

void interruptDisable();
void EdgeTriggerHandler();
void interruptInit();
void interruptEnable();
void interruptDisable();
void SystickReset();
unsigned long tickDiff();
void TransitionTimerIntStart(unsigned long);
void FinishTimerIntStart(unsigned long);
void TransitionTimerIntStop();
void FinishTimerIntStop();
void TransitionTimerIntHandler();
void FinishTimerIntHandler();
void RepeatTimerIntHandler();

unsigned int        g_signalMode    = NONE;
unsigned int        g_finishMode    = NONE;
unsigned long       g_ticksLast     = SYSTICK_WRAP_TICKS;
unsigned long       g_ticksElapsed  = 0UL;
unsigned long long  g_bit_rep   = 0ULL;
unsigned char       g_last_toggle   = 0;
unsigned long long  g_last_bit_rep  = 0ULL;
unsigned char       g_repeat        = 0;
unsigned char       g_send_key  = 0;

int player1_score = 0;
int player2_score = 0;
int player1_paddle;
int player1_paddle_prev;
int player1_paddle_vel;
int player2_paddle;
int player2_paddle_prev;

bool player1_scored = false;

int x_tilt = 0;

int ball_x_pos;
int ball_y_pos;
int ball_x_pos_prev;
int ball_y_pos_prev;
float ball_x_vel;
float ball_y_vel;

unsigned char buffer[100] = {0};
char name[100];

bool rec_new_message = 0;

void SystickReset()
{
    SysTickPeriodSet(SYSTICK_WRAP_TICKS);
    SysTickEnable();
}

unsigned long tickDiff()
{
    // Note that systick counts down, so lesser tick values
    // represent a succeeding point in time

    unsigned long ticksCurrent = SysTickValueGet();
    unsigned long ticksLast = g_ticksLast;

    // Set up for next call
    g_ticksLast = ticksCurrent;

    // No wrap; most probable case, so we short circuit a branch here
    if (ticksCurrent < ticksLast) return ticksLast - ticksCurrent;

    // Wrapped ticks. We assume a max wrap of SYSTICK_MAX_TICKS.
    // The order of operations here is important to prevent arithmetic overflow.
    return (SYSTICK_WRAP_TICKS - ticksCurrent) + ticksLast;
}

void EdgeTriggerHandler()
{
    interruptDisable();

    if (!GPIOPinRead(GPIOA1_BASE,GPIO_PIN_0))
    {
        // Falling edges
        switch (g_signalMode)
        {
            case NONE:
                g_bit_rep = 0ULL;
                g_signalMode = SHORT;
                TransitionTimerIntStart(3);
                break;
            case SHORT:
                TransitionTimerIntStop();
                FinishTimerIntStart(3);
                g_ticksElapsed = tickDiff();
                g_bit_rep <<= 2;
                if (g_ticksElapsed > 50000) g_bit_rep |= 3ULL;
                else if (g_ticksElapsed > 37500) g_bit_rep |= 2ULL;
                else if (g_ticksElapsed > 25000) g_bit_rep |= 1ULL;
                break;
            case LONG:
                TransitionTimerIntStop();
                FinishTimerIntStart(5);
                g_ticksElapsed = tickDiff();
                g_bit_rep <<= 1;
                g_bit_rep |= g_ticksElapsed > 90000 ? 1ULL : 0ULL;
                break;
            default:
                break;
        }
    } else {
        // Rising edges
        g_ticksLast = SysTickValueGet();
    }

    interruptEnable();
}

void interruptInit()
{
    GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_0,GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIOA1_BASE, EdgeTriggerHandler);

    IntPrioritySet(INT_GPIOA1, INT_PRIORITY_LVL_3);

    GPIOIntClear(GPIOA1_BASE,GPIO_PIN_0);
    GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_0);
}

void interruptEnable()
{
    //Enable GPIO Interrupt
    GPIOIntClear(GPIOA1_BASE,GPIO_PIN_0);
    IntPendClear(INT_GPIOA1);
    IntEnable(INT_GPIOA1);
    GPIOIntEnable(GPIOA1_BASE,GPIO_PIN_0);
}

void interruptDisable()
{
    //Clear and Disable GPIO Interrupt
    GPIOIntDisable(GPIOA1_BASE,GPIO_PIN_0);
    GPIOIntClear(GPIOA1_BASE,GPIO_PIN_0);
    IntDisable(INT_GPIOA1);
}

void TransitionTimerIntStart(unsigned long ms)
{
    Timer_IF_Start(TIMERA0_BASE, TIMER_BOTH, ms); // in milliseconds, not ticks.
}

void FinishTimerIntStart(unsigned long ms)
{
    Timer_IF_Start(TIMERA1_BASE, TIMER_BOTH, ms); // in milliseconds, not ticks.
}

void RepeatTimerIntStart(unsigned long ms)
{
    Timer_IF_Start(TIMERA2_BASE, TIMER_BOTH, ms); // in milliseconds, not ticks.
}

void TransitionTimerIntStop()
{
    Timer_IF_Stop(TIMERA0_BASE, TIMER_BOTH);
}

void FinishTimerIntStop()
{
    Timer_IF_Stop(TIMERA1_BASE, TIMER_BOTH);
}

void RepeatTimerIntStop()
{
    Timer_IF_Stop(TIMERA2_BASE, TIMER_BOTH);
}

void TransitionTimerIntHandler()
{
    Timer_IF_InterruptClear(TIMERA0_BASE);

    switch (g_signalMode)
    {
        case SHORT:
            g_signalMode = LONG;
            g_bit_rep = 0ULL;
            TransitionTimerIntStart(15);
            break;
        case LONG:
            // Something is wrong after 3+15=18 ms!
            g_signalMode = NONE;
            FinishTimerIntStop();
            break;
    }
}

void FinishTimerIntHandler()
{
    Timer_IF_InterruptClear(TIMERA1_BASE);

    g_finishMode = g_signalMode;
    g_signalMode = NONE;
    g_ticksElapsed = 0UL;
}

void RepeatTimerIntHandler()
{
    Timer_IF_InterruptClear(TIMERA2_BASE);

    g_repeat = 0;
    g_send_key = 1;
}
//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

static int set_time();
static int http_post(int);

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

void DebugRoutine()
{
    unsigned long risingTickWidths[100];
    unsigned long i = 0;

    while (1)
    {
        i = 0;

        while (i < 100)
            if (g_ticksElapsed)
            {
                risingTickWidths[i] = g_ticksElapsed;
                ++i;
                g_ticksElapsed = 0UL;
            }

        for (i = 0; i < 100; ++i)
            Report("%d: %lu ticks\n\r", i + 1, risingTickWidths[i]);
    }
}

unsigned long long SignalRoutine()
{
    unsigned long long bit_rep;
    unsigned char toggle_bit;

    while(!g_finishMode);

    switch (g_finishMode)
    {
        case SHORT:
            // Get toggle bit
            toggle_bit = (g_bit_rep & 0x8000ULL) >> 15;
            // Always set the 16th least-significant bit, the toggle bit, to high
            g_bit_rep |= 0x8000ULL;
            // Only keep the 32+4=36 least significant bits
            g_bit_rep &= 0xFFFFFFFFFULL;
            break;
        case LONG:
            // Only keep the 32 least significant bits
            g_bit_rep &= 0xFFFFFFFFULL;
            break;
    }

    g_finishMode = NONE;

    bit_rep = g_bit_rep;
    g_bit_rep = 0ULL;

    if (bit_rep == g_last_bit_rep && toggle_bit == g_last_toggle)
        return 0ULL;

    g_last_bit_rep = bit_rep;
    g_last_toggle = toggle_bit;

    return bit_rep;
}

unsigned long long SignalRoutineModified()
{
    unsigned long long bit_rep;
    unsigned char toggle_bit;

    switch (g_finishMode)
    {
        case SHORT:
            // Get toggle bit
            toggle_bit = (g_bit_rep & 0x8000ULL) >> 15;
            // Always set the 16th least-significant bit, the toggle bit, to high
            g_bit_rep |= 0x8000ULL;
            // Only keep the 32+4=36 least significant bits
            g_bit_rep &= 0xFFFFFFFFFULL;
            break;
        case LONG:
            // Only keep the 32 least significant bits
            g_bit_rep &= 0xFFFFFFFFULL;
            break;
    }

    g_finishMode = NONE;

    bit_rep = g_bit_rep;
    g_bit_rep = 0ULL;

    if (bit_rep == g_last_bit_rep && toggle_bit == g_last_toggle)
        return 0ULL;

    g_last_bit_rep = bit_rep;
    g_last_toggle = toggle_bit;

    return bit_rep;
}

typedef enum
{
    BUTTON_INVALID = 0,

    BUTTON_1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4,
    BUTTON_5,
    BUTTON_6,
    BUTTON_7,
    BUTTON_8,
    BUTTON_9,
    BUTTON_0,

    BUTTON_LEFT,
    BUTTON_RIGHT,

    BUTTON_LAST,
    BUTTON_ENTER,
} button_type;

button_type ButtonRoutine()
{
    unsigned long long bit_rep = SignalRoutine();

    switch (bit_rep)
    {
        case 551487735ULL: return BUTTON_0;
        case 551504055ULL: return BUTTON_2;
        case 551536695ULL: return BUTTON_3;
        case 551495895ULL: return BUTTON_4;
        case 551528535ULL: return BUTTON_5;
        case 551512215ULL: return BUTTON_6;
        case 551544855ULL: return BUTTON_7;
        case 551491815ULL: return BUTTON_8;
        case 551524455ULL: return BUTTON_9;

        case 591439450ULL: return BUTTON_LEFT;
        case 591439451ULL: return BUTTON_RIGHT;

        case 591439518ULL: return BUTTON_LAST;
        case 591439585ULL: return BUTTON_ENTER;

        default: return BUTTON_INVALID;
    }
}

typedef struct button_press
{
    button_type     button;
    unsigned char   control;
    unsigned char   cycle;
    unsigned char   ascii;
} button_press;

button_type g_last_type = BUTTON_INVALID;

button_press WriteRoutine()
{
    button_press press;
    button_type button = ButtonRoutine();

    press.button = button;

    if (!button) return press;

    switch (button)
    {
        case BUTTON_2:
        case BUTTON_3:
        case BUTTON_4:
        case BUTTON_5:
        case BUTTON_6:
        case BUTTON_7:
        case BUTTON_8:
        case BUTTON_9:
        case BUTTON_0:
        case BUTTON_ENTER:
        case BUTTON_LAST:
            press.control = 0;
            break;
        default:
            press.control = 1;
            break;
    }

    if(g_last_type == button)
    {
        press.cycle = g_repeat;
        ++g_repeat;
    }
    else
    {
        press.cycle = 0;
        g_repeat = 1;
        g_last_type = button;
        g_send_key = 1;
    }
    RepeatTimerIntStart(1000);

    // Do not read from flag g_repeat after this point.
    // Use press.cycle instead.

    switch (button)
    {
        case BUTTON_2:
            press.ascii = (unsigned char[]){'A', 'B', 'C'}[press.cycle % 3];
            break;
        case BUTTON_3:
            press.ascii = (unsigned char[]){'D', 'E', 'F'}[press.cycle % 3];
            break;
        case BUTTON_4:
            press.ascii = (unsigned char[]){'G', 'H', 'I'}[press.cycle % 3];
            break;
        case BUTTON_5:
            press.ascii = (unsigned char[]){'J', 'K', 'L'}[press.cycle % 3];
            break;
        case BUTTON_6:
            press.ascii = (unsigned char[]){'M', 'N', 'O'}[press.cycle % 3];
            break;
        case BUTTON_7:
            press.ascii = (unsigned char[]){'P', 'Q', 'R', 'S'}[press.cycle % 4];
            break;
        case BUTTON_8:
            press.ascii = (unsigned char[]){'T', 'U', 'V'}[press.cycle % 3];
            break;
        case BUTTON_9:
            press.ascii = (unsigned char[]){'W', 'X', 'Y', 'Z'}[press.cycle % 4];
            break;
        case BUTTON_ENTER:
            press.ascii = '\n';
            break;
        case BUTTON_LAST:
            press.ascii = '\b';
            break;
    }

    return press;
}

void DemoRoutine()
{
    button_press p;

    while (1)
    {
        // Remember that the underlying representation of enum values
        // are implementation specific. It's usually an int, though.
        p = WriteRoutine();

        if (p.cycle) Message("\r");
        else Message("\n");
        if (p.button && !p.control) Report("%c", (char) p.ascii);
    }
}

void GetName()
{
    unsigned char cChar;
    button_press curr;

    unsigned int index = 0;

    int x_cursor = 49;
    int y_cursor = 80;

    drawChar(55, y_cursor, '_', WHITE, BLACK, (unsigned char) 1);
    drawChar(61, y_cursor, '_', WHITE, BLACK, (unsigned char) 1);
    drawChar(67, y_cursor, '_', WHITE, BLACK, (unsigned char) 1);

    // As you write a character to the terminal it will
    // immediately be reflected on the receivers OLED display.
    while(1)
    {
        curr = WriteRoutine();
        interruptDisable();

        if(curr.button != BUTTON_INVALID)
        {
            cChar = curr.ascii;

            if(curr.cycle != 0 && cChar != '\b' && cChar != '\n') // Multiple presses.
            {
                drawChar(x_cursor, y_cursor, cChar, WHITE, BLACK, (unsigned char) 1);
                buffer[index-1] = cChar;
            }
            else
            {
                if(cChar == '\n') // end of message.
                {
                    if (index == 3) {
                        buffer[index] = '\0';
                        strcpy(name, (char*)buffer);
                        strcat(name, DATA_B);
                        strcat(DATA_F, name);
                        break;
                    }
                }
                else if (cChar == '\b') {
                    if (index > 0) {
                        drawChar(x_cursor, y_cursor, '_', WHITE, BLACK, (unsigned char) 1);
                        index -= 1;
                        x_cursor -= 6;     // go back one
                    }
                }
                else
                {
                    if (index < 3) {
                        x_cursor += 6;
                        drawChar(x_cursor, y_cursor, cChar, WHITE, BLACK, (unsigned char) 1);
                        buffer[index] = cChar;
                        index++;
                    }
                }
            }
        }

        interruptEnable();
    }
    interruptEnable();
}

void PongSetup()
{
    fillScreen(BLACK);
    int temp;

    //place inner dashes
    int curr_y = INNER_DASH_HEIGHT/2;
    while (curr_y <= HEIGHT) {
        fillRect(INNER_DASH_X_POS - (INNER_DASH_WIDTH/2), curr_y, INNER_DASH_WIDTH, INNER_DASH_HEIGHT, WHITE);
        curr_y += INNER_DASH_HEIGHT;
        curr_y += INNER_DASH_HEIGHT;
    }
    //place ball
    ball_x_pos = WIDTH/2;
    ball_y_pos = HEIGHT/2;
    drawBall(ball_x_pos, ball_y_pos, RADIUS, WHITE);

    //generate ball velocity, towards last point loser
    ball_x_vel = -1.5;
    if (player1_scored) {
        player1_scored = false;
        ball_x_vel *= -1;

        temp = player1_score % 10 + 48 - 1;
        if (temp == 47) {
            temp += 1;
        }
        drawChar(30, 10, temp, 0xFFFF, 0x0000, 2);
        if (player1_score - 1 >= 10) {
            drawChar(18, 10, 49, 0xFFFF, 0x0000, 2);
        }

        //place player2 score
        temp = player2_score % 10 + 48;
        drawChar(98, 10, temp, 0xFFFF, 0x0000, 2);
        if (player2_score >= 10) {
            drawChar(86, 10, 49, 0xFFFF, 0x0000, 2);
        }

    }
    else {

        //place player1 score
        temp = player1_score % 10 + 48;
        drawChar(30, 10, temp, 0xFFFF, 0x0000, 2);
        if (player1_score >= 10) {
            drawChar(18, 10, 49, 0xFFFF, 0x0000, 2);
        }

        temp = player2_score % 10 + 48 - 1;
        if (temp == 47) {
            temp += 1;
        }
        drawChar(98, 10, temp, 0xFFFF, 0x0000, 2);
        if (player2_score - 1 >= 10) {
            drawChar(86, 10, 49, 0xFFFF, 0x0000, 2);
        }
    }

    ball_y_vel = (rand() % 3) - 1;

    if (ball_y_vel == 0) {
        ball_y_vel = 1;
    }

    //place player1 paddle
    player1_paddle = HEIGHT/2;
    fillRect(PLAYER1_PADDLE_X_POS + (PADDLE_WIDTH/2), player1_paddle - (PADDLE_LENGTH/2), PADDLE_WIDTH, PADDLE_LENGTH, WHITE);
    //place player2 paddle
    player2_paddle = HEIGHT/2;
    fillRect(PLAYER2_PADDLE_X_POS - (PADDLE_WIDTH/2), player2_paddle - (PADDLE_LENGTH/2), PADDLE_WIDTH, PADDLE_LENGTH, WHITE);

    MAP_UtilsDelay(10000000);

    //place player1 score
    temp = player1_score % 10 + 48;
    drawChar(30, 10, temp, 0xFFFF, 0x0000, 2);
    if (player1_score >= 10) {
        drawChar(18, 10, 49, 0xFFFF, 0x0000, 2);
    }
    //place player2 score
    temp = player2_score % 10 + 48;
    drawChar(98, 10, temp, 0xFFFF, 0x0000, 2);
    if (player2_score >= 10) {
        drawChar(86, 10, 49, 0xFFFF, 0x0000, 2);
    }

    MAP_UtilsDelay(10000000);

    drawChar(30, 10, ' ', 0xFFFF, 0x0000, 2);
    if (player1_score >= 10) {
        drawChar(18, 10, ' ', 0xFFFF, 0x0000, 2);
    }
    //place player2 score
    temp = player2_score % 10 + 48;
    drawChar(98, 10, ' ', 0xFFFF, 0x0000, 2);
    if (player2_score >= 10) {
        drawChar(86, 10, ' ', 0xFFFF, 0x0000, 2);
    }

    MAP_UtilsDelay(10000000);
}

void PongRoutine()
{
    unsigned char ucDevAddr, ucXRegOffset, ucRdLen;
    unsigned char aucRdDataBuf[256];
    char *pcErrPtr;

    float speedup = 1.10;
    float max_speed = 5.0;
    float player2_paddle_vel = 0;

    ucDevAddr = (unsigned char)strtoul(ADDRESS, &pcErrPtr, 16);

    ucXRegOffset = (unsigned char)strtoul(X_OFFSET, &pcErrPtr, 16);

    ucRdLen = (unsigned char)strtoul(LENGTH, &pcErrPtr, 10);

    while (player1_score < WINNING_SCORE && player2_score < WINNING_SCORE) {

        unsigned long long bit_rep = SignalRoutineModified();

        //write acc_x from accelerometer
        I2C_IF_Write(ucDevAddr, &ucXRegOffset, 1, 0);

        //read acc_x into buffer
        I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen);

        //store acc_x into variable
        x_tilt = (char) aucRdDataBuf[0];

        player1_paddle_prev = player1_paddle;

        //get x_vel
        if (0 <= (int)x_tilt && (int)x_tilt <= 66) {
            player1_paddle_vel = x_tilt;
        }
        else {
            player1_paddle_vel = -(256-x_tilt);
        }
        //ensures paddle doesn't move when board is flat
        if (-3 <= player1_paddle_vel && 3 >= player1_paddle_vel) {
            player1_paddle_vel = 0;
        }

        player1_paddle = player1_paddle + player1_paddle_vel;

        if (player1_paddle + (PADDLE_LENGTH/2) > HEIGHT) {
            player1_paddle = HEIGHT - (PADDLE_LENGTH/2);
        }
        else if (player1_paddle - (PADDLE_LENGTH/2) < 0) {
            player1_paddle = (PADDLE_LENGTH/2);
        }

        //draw player1_paddle
        if (player1_paddle != player1_paddle_prev) {
            fillRect(PLAYER1_PADDLE_X_POS + (PADDLE_WIDTH/2), player1_paddle_prev - (PADDLE_LENGTH/2), PADDLE_WIDTH, PADDLE_LENGTH, BLACK);
        }
        fillRect(PLAYER1_PADDLE_X_POS + (PADDLE_WIDTH/2), player1_paddle - (PADDLE_LENGTH/2), PADDLE_WIDTH, PADDLE_LENGTH, WHITE);

        //store player2_paddle
        player2_paddle_prev = player2_paddle;

        player2_paddle_vel = (player2_paddle - player2_paddle_prev)/2;

        //get new player2_paddle location
        if (bit_rep == 591439448) {
            player2_paddle -= 8;
        }
        else if (bit_rep == 591439449) {
            player2_paddle += 8;
        }

        if (player2_paddle + (PADDLE_LENGTH/2) > HEIGHT) {
            player2_paddle = HEIGHT - (PADDLE_LENGTH/2);
        }
        else if (player2_paddle - (PADDLE_LENGTH/2) < 0) {
            player2_paddle = (PADDLE_LENGTH/2);
        }
        //draw player2_paddle
        if (player2_paddle != player2_paddle_prev) {
            fillRect(PLAYER2_PADDLE_X_POS - (PADDLE_WIDTH/2), player2_paddle_prev - (PADDLE_LENGTH/2), PADDLE_WIDTH, PADDLE_LENGTH, BLACK);
        }
        fillRect(PLAYER2_PADDLE_X_POS - (PADDLE_WIDTH/2), player2_paddle - (PADDLE_LENGTH/2), PADDLE_WIDTH, PADDLE_LENGTH, WHITE);

        //save ball's previous position
        ball_x_pos_prev = ball_x_pos;
        ball_y_pos_prev = ball_y_pos;

        //update x and y positions of ball
        ball_x_pos = ball_x_pos + ball_x_vel;
        ball_y_pos = ball_y_pos + ball_y_vel;

        //if ball hits wall, redirect
        if (ball_y_pos + RADIUS > HEIGHT) {
            ball_y_vel = -ball_y_vel;
            ball_y_pos =  HEIGHT - RADIUS;
        }
        else if (ball_y_pos - RADIUS < 0) {
            ball_y_vel = -ball_y_vel;
            ball_y_pos = RADIUS;
        }

        if (ball_x_pos_prev > PLAYER1_PADDLE_X_POS && ball_x_pos <= PLAYER1_PADDLE_X_POS) {
            float t = (PLAYER1_PADDLE_X_POS - ball_x_pos_prev) / (ball_x_pos - ball_x_pos_prev);
            float y_at_cross = ball_y_pos_prev + t * (ball_y_pos - ball_y_pos_prev);

            if (y_at_cross + RADIUS >= player1_paddle - PADDLE_LENGTH/2 &&
                y_at_cross - RADIUS <= player1_paddle + PADDLE_LENGTH/2) {

                ball_x_pos = PLAYER1_PADDLE_X_POS + RADIUS + 1;

                ball_x_vel *= -1;
                if (fabs(ball_x_vel) < max_speed) {
                    ball_x_vel *= speedup;
                }
                if (fabs(ball_y_vel) < max_speed) {
                    ball_y_vel *= speedup;
                }

                ball_y_vel += player1_paddle_vel * 0.1f;
            }
        }

        if (ball_x_pos_prev < PLAYER2_PADDLE_X_POS && ball_x_pos >= PLAYER2_PADDLE_X_POS) {
            float t = (PLAYER2_PADDLE_X_POS - ball_x_pos_prev) / (ball_x_pos - ball_x_pos_prev);
            float y_at_cross = ball_y_pos_prev + t * (ball_y_pos - ball_y_pos_prev);

            if (y_at_cross + RADIUS >= player2_paddle - PADDLE_LENGTH/2 &&
                y_at_cross - RADIUS <= player2_paddle + PADDLE_LENGTH/2) {

                ball_x_pos = PLAYER2_PADDLE_X_POS - RADIUS - 1;

                ball_x_vel *= -1;
                if (fabs(ball_x_vel) < max_speed) {
                    ball_x_vel *= speedup;
                }
                if (fabs(ball_y_vel) < max_speed) {
                    ball_y_vel *= speedup;
                }

                ball_y_vel += player2_paddle_vel * 0.1f;
            }
        }
        //check if ball has moved since last loop
        if (ball_x_pos != ball_x_pos_prev || ball_y_pos != ball_y_pos_prev) {
            //erase previous frame of ball
            drawBall(ball_x_pos_prev, ball_y_pos_prev, RADIUS, BLACK);
            //draw ball at updated position
            drawBall(ball_x_pos, ball_y_pos, RADIUS, WHITE);
        }

        //if ball hits goal, player score goes up
        if (ball_x_pos + RADIUS > WIDTH) {
            player1_score += 1;
            player1_scored = true;
            PongSetup();
        }
        else if (ball_x_pos - RADIUS < 0) {
            player2_score += 1;
            PongSetup();
        }


        MAP_UtilsDelay(100000);
    }
}

void WinRoutine()
{
    fillScreen(BLACK);

    drawChar(25, 32, 0x50, 0xFFFF, 0x0000, 1); // P
    drawChar(31, 32, 0x6C, 0xFFFF, 0x0000, 1); // l
    drawChar(37, 32, 0x61, 0xFFFF, 0x0000, 1); // a
    drawChar(43, 32, 0x79, 0xFFFF, 0x0000, 1); // y
    drawChar(49, 32, 0x65, 0xFFFF, 0x0000, 1); // e
    drawChar(55, 32, 0x72, 0xFFFF, 0x0000, 1); // r
    if (player1_score == WINNING_SCORE) {
        drawChar(67, 32, 0x31, 0xFFFF, 0x0000, 1); // 1
    }
    else {
        drawChar(67, 32, 0x32, 0xFFFF, 0x0000, 1); // 2
    }
    drawChar(79, 32, 0x57, 0xFFFF, 0x0000, 1); // W
    drawChar(85, 32, 0x69, 0xFFFF, 0x0000, 1); // i
    drawChar(91, 32, 0x6E, 0xFFFF, 0x0000, 1); // n
    drawChar(97, 32, 0x73, 0xFFFF, 0x0000, 1); // s
    drawChar(103, 32, 0x21, 0xFFFF, 0x0000, 1); // !

    drawChar(19, 64, 0x45, 0xFFFF, 0x0000, 1); // E
    drawChar(25, 64, 0x6E, 0xFFFF, 0x0000, 1); // n
    drawChar(31, 64, 0x74, 0xFFFF, 0x0000, 1); // t
    drawChar(37, 64, 0x65, 0xFFFF, 0x0000, 1); // e
    drawChar(43, 64, 0x72, 0xFFFF, 0x0000, 1); // r
    drawChar(55, 64, 0x79, 0xFFFF, 0x0000, 1); // y
    drawChar(61, 64, 0x6F, 0xFFFF, 0x0000, 1); // o
    drawChar(67, 64, 0x75, 0xFFFF, 0x0000, 1); // u
    drawChar(73, 64, 0x72, 0xFFFF, 0x0000, 1); // r
    drawChar(85, 64, 0x6E, 0xFFFF, 0x0000, 1); // n
    drawChar(91, 64, 0x61, 0xFFFF, 0x0000, 1); // a
    drawChar(97, 64, 0x6D, 0xFFFF, 0x0000, 1); // m
    drawChar(103, 64, 0x65, 0xFFFF, 0x0000, 1); // e
    drawChar(109, 64, 0x3A, 0xFFFF, 0x0000, 1); // :

}
//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//! This function
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
int
main() {
    long lRetVal = -1;
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Power on
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();

    // Enable SPI clock
    PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //Initialize UART1 connection

    // Reset SPI peripheral
    PRCMPeripheralReset(PRCM_GSPI);

    //Reset SPI
    MAP_SPIReset(GSPI_BASE);

    //Configure SPI
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                           SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                           (SPI_SW_CTRL_CS |
                           SPI_4PIN_MODE |
                           SPI_TURBO_OFF |
                           SPI_CS_ACTIVELOW |
                           SPI_WL_8));

    //Enable SPI protocol
    MAP_SPIEnable(GSPI_BASE);

    //Initialize I2C
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //Initialize Adafruit
    Adafruit_Init();

    //Initialize and clear terminal
    InitTerm();
    ClearTerm();

    //Display the banner
    Message("****************************************************\n\n\r");
    Message("CC3200 Dual Controller Pong\n\n\r");
    Message("****************************************************\n\n\r");
    Message("****************************************************\n\n\r");

    SysTickPeriodSet(SYSTICK_WRAP_TICKS);
    SysTickIntRegister(SystickReset);
    SysTickEnable();

    interruptInit();
    interruptEnable();

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT, TIMER_BOTH, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_BOTH, TransitionTimerIntHandler);

    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT, TIMER_BOTH, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_BOTH, FinishTimerIntHandler);

    Timer_IF_Init(PRCM_TIMERA2, TIMERA2_BASE, TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT, TIMER_BOTH, 0);
    Timer_IF_IntSetup(TIMERA2_BASE, TIMER_BOTH, RepeatTimerIntHandler);

    PongSetup();

    PongRoutine();

    WinRoutine();

    GetName();

    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }

    UART_PRINT(DATA_F);

    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }

    http_post(lRetVal);

    sl_Stop(SL_STOP_TIMEOUT);
    LOOP_FOREVER();
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA_F);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA_F);
    pcBufHeaders += strlen(DATA_F);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}
