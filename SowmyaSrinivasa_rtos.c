// Basic RTOS Framework - Spring 2022
// No memory protection, no privilege enforcement
// J Losh

// Student Name:
// Sowmya Srinivasa

// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"
#include <string.h>

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board orange LED


#define PUSH_BUTTON0   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON1   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PUSH_BUTTON2   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON3   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define PUSH_BUTTON4   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON5   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4)))


//Port masks
#define BLUE_LED_MASK 4
#define RED_LED_MASK 4
#define YELLOW_LED_MASK 16
#define GREEN_LED_MASK 1
#define ORANGE_LED_MASK 8

#define PUSH_BUTTON0_MASK 16
#define PUSH_BUTTON1_MASK 32
#define PUSH_BUTTON2_MASK 64
#define PUSH_BUTTON3_MASK 128
#define PUSH_BUTTON4_MASK 64
#define PUSH_BUTTON5_MASK 128



extern void setASP(uint8_t);
extern void setPSP(void *);
extern void *getPSP(void);
extern void pushReglist();
extern void popReglist();
extern void createHWpushContext(int temp_xpsr, void *sp);
extern void *getSVCnumber();
extern int getR0();

#define DEBUG
char str[30];

#define PR         1
#define RR         2
#define DELIMETER 'd'
#define ALPHA     'a'
#define NUMBER    'n'
#define MAX_CHARS 80
#define MAX_FIELDS 5
#define MAX_DATAPACKETS 10
#define MAX_COUNT 5

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

// UI message variables
#define UI_BUFFER_LENGTH 30
char UImessage[UI_BUFFER_LENGTH];
uint8_t rxIndex = 0;
uint8_t txIndex = 0;




//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 2
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];
#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_HOLD       5 // supports kill command

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t pidCounter = 0;   // incremented on each thread created
#define MAX_PRIORITIES 8

uint32_t *heap = (uint32_t*) 0x20002000;// question : why do we need to do x*4?
uint32_t allocated_heap = 0;

//#define DEBUG

#define YIELD 16
#define SLEEP 32
#define POST  64
#define WAIT  128

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------


bool preemption = true;
uint8_t scheduler = PR;
uint32_t taskCycle[2][MAX_TASKS];
uint8_t bufferBlock;


// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    uint32_t pid;                  // PID
    fn pFn;                        // function pointer
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 7=lowest
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    uint8_t s;                     // index of semaphore that is blocking the thread
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pFn = 0;
    }
}


// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    if(scheduler == RR)
    {
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
    }
    else
    {
        uint8_t level = 0;
        uint8_t temp = taskCurrent;//+1)%MAX_TASKS;
        while(level<MAX_PRIORITIES)
        {
            temp = (temp+1) % MAX_TASKS;
            if((tcb[temp].state == (STATE_READY) || tcb[temp].state == (STATE_UNRUN)) && tcb[temp].priority == level)
            {
                return temp;
            }
            if(temp == taskCurrent)
                level++;
        }
    }
    return task;
}

bool createThread(fn task, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0, j = 0;
    bool found = false;
    // REQUIRED:
    // store the thread name
    // allocate stack space and store top of stack in sp and spInit
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure task not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pFn == task);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = pidCounter++;
            tcb[i].pFn = task;
            tcb[i].sp = &heap[allocated_heap+(stackBytes>>2)];
            allocated_heap += stackBytes>>2;
            tcb[i].spInit = tcb[i].sp;
#ifdef DEBUG
            sprintf(str, "stackbase = %p\t, %p\r\n", tcb[i].sp, tcb[i].spInit);
            putsUart0(str);
#endif
            // name copy
            for(j=0; name[j]!='\0'; j++)
            {
                tcb[i].name[j] = name[j];
            }
            tcb[i].priority = priority;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(fn task)
{
    int i;
    for(i = 0; i<MAX_TASKS; i++)
    {
        if (tcb[i].pFn == task)
        {
            tcb[i].pid = pidCounter++;
            tcb[i].sp =tcb[i].spInit;
            tcb[i].ticks=0;
            tcb[i].state = STATE_UNRUN;
            break;
        }
    }
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(fn task)
{
    uint8_t tempPID, i, j;
    for(i = 0; i<MAX_TASKS; i++)
    {
        if (tcb[i].pFn == task)
        {
            tempPID = i;
            break;
        }
    }
    if(tcb[tempPID].state == STATE_BLOCKED)
    {
        for(i = 1; i < 5; i++)
        {
            for(j=0; j<2; j++)
            {
                if(semaphores[i].processQueue[j] == tempPID)
                {
                    if(j == 0)
                    {
                        semaphores[i].processQueue[0] = semaphores[i].processQueue[1];
                        semaphores[i].processQueue[1] = 0;
                        semaphores[i].queueSize--;
                    }
                    else
                    {
                        semaphores[i].processQueue[1] = 0;
                        semaphores[i].queueSize--;
                    }
                }
            }
        }
    }
    for(i = 1; i <5; i++)
    {
        if(tcb[tempPID].s == i && (tcb[tempPID].state == STATE_READY || tcb[tempPID].state == STATE_DELAYED))
        {
            // tcb[i].s = 0;
            if(semaphores[i].queueSize > 0)
            {
                tcb[semaphores[i].processQueue[0]].state = STATE_READY;
                //tcb[semaphores[i].processQueue[0]].s = 0;
                semaphores[i].processQueue[0] = semaphores[i].processQueue[1];
                //semaphores[i].processQueue[1] = 0;
                semaphores[i].queueSize--;
            }
        }
    }
    tcb[tempPID].state = STATE_HOLD;
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(fn task, uint8_t priority)
{
    uint8_t i;
    for(i = 0; i<MAX_TASKS; i++)
    {
        if(tcb[i].pFn == task)
        {
            tcb[i].priority = priority;
        }
    }
}

bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
    }
    return ok;
}

void getsUart0(USER_DATA *data)
{
    int count = 0;
    char ch;

    while (count <= (MAX_CHARS))
    {
        ch = getcUart0();
        if (ch == 8 || ch == 127)
        {
            if (count > 0)
            {
                count--;

            }
            continue;
        }

        if (ch == 13)
        {
            data->buffer[count] = '\0';
            break;                       // exit when enter key is pressed
        }

        data->buffer[count] = ch;       // Update buffer array
        count++;

        if (count == MAX_CHARS)
        {
            data->buffer[count + 1] = '\0';
            break;                        // exit when max chars reached
        }
    }
}

void parseFields(USER_DATA *data)
{
    char prev_type, current_type;
    prev_type = DELIMETER;
    data->fieldCount = 0;
    int i = 0;

    while (data->buffer[i] != '\0')
    {

        if ((data->buffer[i] >= 97 && data->buffer[i] <= 122)
                || (data->buffer[i] >= 65 && data->buffer[i] <= 90))
        {
            current_type = ALPHA; // Setting fieldType = 'a';
        }

        else if ((data->buffer[i] >= 48 && data->buffer[i] <= 57)
                || data->buffer[i] == 45 || data->buffer[i] == 46)
        {
            current_type = NUMBER; // Setting fieldType = 'n';
        }
        else
        {
            current_type = DELIMETER;
            data->buffer[i] = '\0';
        }

        if (prev_type == DELIMETER
                && (current_type == ALPHA || current_type == NUMBER))
        {
            data->fieldType[data->fieldCount] = current_type;
            data->fieldPosition[data->fieldCount] = i;
            data->fieldCount++;
        }

        prev_type = current_type;
        i++;

        if (data->fieldCount == MAX_FIELDS)
        {
            break;
        }
    }
}

char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{

    if (fieldNumber <= data->fieldCount)
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return '\0';
    }
}

int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    if ((fieldNumber <= data->fieldCount)
            && (data->fieldType[fieldNumber] == 'n'))
    {
        uint8_t temp, i = 0;
        uint32_t val = 0;

        char *str = &data->buffer[data->fieldPosition[fieldNumber]];
        while (str[i] != '\0')
        {
            temp = str[i] - 48;
            val = (val * 10) + temp;
            i++;
        }
        return val;
    }
    return 0;
}

bool strCompare(char *str1, const char *str2)
{
    int i = 0;
    while (str1[i] == str2[i])
    {
        if (str1[i] == '\0' || str2[i] == '\0')
        {
            break;
        }
        i++;
    }
    if (str1[i] == '\0' && str2[i] == '\0')
    {
        return true;
    }
    return false;
}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    if (strCompare((&data->buffer[data->fieldPosition[0]]), strCommand))
    {
        if (minArguments < data->fieldCount)
            return true;
    }
    return false;
}

void itostring(int n, char* s)
{
    int j;
    char tempChar;
    j = 0;

    while(n)
    {
        s[j++] = n % 10 + '0';
        n /= 10;
    }
    s[j] = '\0';
    int i=0;
    j--;
    while(i<j)
    {
        tempChar = s[j];
        s[j] = s[i];
        s[i] = tempChar;

        i++;
        j--;
    }
}

void uart0ISR()
{

    if(UART0_FR_R & UART_FR_TXFE)
    {
        if(txIndex != rxIndex)
        {
            UART0_DR_R = UImessage[rxIndex];
            rxIndex = (rxIndex + 1) % UI_BUFFER_LENGTH;
        }

        else
        {
            UART0_IM_R &= ~UART_IM_TXIM;
            UART0_ICR_R |= UART_ICR_TXIC ;//| UART_ICR_RXIC;
        }
    }
}

void guiAlignment(void)
{
    putcUart0('\r');
    putcUart0('\n');
    putcUart0('>');
}

void processShell()
{
    USER_DATA data;
    getsUart0(&data);
    parseFields(&data);
    bool valid;
    // Echo back the parsed field data (type and fields)
    uint8_t i;
    for (i = 0; i < data.fieldCount; i++)
    {
        putcUart0(data.fieldType[i]);
        putcUart0('\t');
        guiAlignment();

        putsUart0(&data.buffer[data.fieldPosition[i]]);
        putcUart0('\n');
        guiAlignment();
    }
    if (isCommand(&data, "reboot", 0))
    {

        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        valid = true;
    }
    if (isCommand(&data, "preemption", 1))
    {
        char *firstArgument = getFieldString(&data, 1);
        {
            if (strCompare(firstArgument, "on"))
            {
                preemption = 1;
            }
            else if (strCompare(firstArgument, "off"))
            {
                preemption = 0;
            }
            else
            {
                putsUart0("Invalid Argument\n");
                guiAlignment();
            }


        }
        valid = true;
    }
    if (isCommand(&data, "scheduler", 1))
    {
        char *firstArgument = getFieldString(&data, 1);
        {
            if (strCompare(firstArgument, "RR"))
            {
                scheduler = RR;
            }
            else if (strCompare(firstArgument, "PR"))
            {
                scheduler = PR;
            }
            else
            {
                putsUart0("Invalid Argument\n");
                guiAlignment();
            }


        }
        valid = true;
    }
    if (isCommand(&data, "pidof", 1))
    {
        uint8_t taskToPrint;
        char *firstArgument = getFieldString(&data, 1);
        for(i = 0; i<MAX_TASKS; i++)
        {
            if (strCompare(firstArgument, tcb[i].name))
            {
                taskToPrint = i;
                break;
            }
        }
        sprintf(str, "pidID of %s\t: %p\r\n", tcb[taskToPrint].name, tcb[taskToPrint].pid);
        putsUart0(str);
        guiAlignment();
    }
    if (isCommand(&data, "run", 1))
    {
        uint8_t task;
        char *firstArgument = getFieldString(&data, 1);
        for(i = 0; i<MAX_TASKS; i++)
        {
            if (strCompare(firstArgument, tcb[i].name))
            {
                task = i;
                break;
            }
        }
        if(tcb[task].state == STATE_HOLD)
        {
            restartThread(tcb[task].pFn);
        }
        putsUart0("\r Task Restarted\r\n");
    }
    if (isCommand(&data, "kill", 1))
    {
        uint8_t task;
        uint8_t firstArgument = getFieldInteger(&data, 1);
        for(i = 0; i<MAX_TASKS; i++)
        {
            if (tcb[i].pid == firstArgument)
            {
                task = i;
                break;
            }
        }
        destroyThread(tcb[task].pFn);
        putsUart0("\r Task Killed\r\n");
    }
    // ps calculations
    if (isCommand(&data, "ps", 0))
    {
        putsUart0("----------------------------------------------------\r\n");
        sprintf(str, "|TaskPID\t|Name|\t|CPU Time|\t|\r\n");
        putsUart0(str);
        putsUart0("----------------------------------------------------\r\n");

        uint64_t totalTime = 0, taskTime[MAX_TASKS], temptime[MAX_TASKS], local1;
        uint32_t temp1[MAX_TASKS], temp2[MAX_TASKS];
        uint8_t local = !bufferBlock;
        for(i = 0; i<MAX_TASKS; i++)
        {
            totalTime += taskCycle[local][i];
        }
        for(i = 0; i<MAX_TASKS; i++)
        {
            local1 = taskCycle[local][i];

            temptime[i] = (local1*100000);
            taskTime[i] = temptime[i]/totalTime;
            temp1[i] = taskTime[i]/1000;
            temp2[i] = taskTime[i]%100;

            sprintf(str, " %d\t\t%s\t\t%d.%d\r\n", tcb[i].pid, tcb[i].name,temp1[i],temp2[i]);
            putsUart0(str);

        }
        guiAlignment();
        putsUart0("-------------------------------------------------\r\n");
    }

//semaphore ipcs
    if (isCommand(&data, "ipcs", 0))
    {
        uint8_t i,j;
        putsUart0("----------------------------------------------------\r\n");
        sprintf(str, "|Semaphore\t|Count|\t|QueueSize|\t|Queue[0]:Queue[1]|\r\n");
        putsUart0(str);
        putsUart0("----------------------------------------------------\r\n");
        guiAlignment();
        for(i=0;i<5;i++)
        {
            sprintf(str, "%5.1d\t\t",i);
            putsUart0(str);
            sprintf(str, "%2.1d\t", semaphores[i].count);
            putsUart0(str);
            sprintf(str, " %2.1d\t\t", semaphores[i].queueSize);
            putsUart0(str);
            if(semaphores[i].queueSize == 0)
            {
                putsUart0("none\r\n");
            }
            if(semaphores[i].queueSize>0)
            {
                for(j=0;j<MAX_TASKS;j++)
                {
                    if(j==semaphores[i].processQueue[0])
                    {
                        putsUart0(tcb[j].name);
                        putsUart0("\r\n");
                    }
                }
            }
//            for(i=0;i<MAX_TASKS;i++)
//            {
//
//                if(i==semaphores[task].processQueue[0])
//                {
//                    putsUart0(tcb[queue].name);
//                    putsUart0("\r\n");
//                }
//            }
//
//            sprintf(str, "%s:%s \r\n", tcb[semaphores[i].processQueue[0]].name, tcb[semaphores[i].processQueue[1]].name);
//
//            putsUart0(str);
//            valid = true;
//            guiAlignment();
        }
        putsUart0("-------------------------------------------------\r\n");
        guiAlignment();
    }
    if (!valid)
    {
        putsUart0("Invalid command\n");
    }
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, and PC
void startRtos()
{
    taskCurrent = rtosScheduler();
    TIMER1_TAV_R = 0;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    tcb[taskCurrent].state = STATE_READY;
    setPSP(tcb[taskCurrent].sp);
    setASP(2);
    fn task = tcb[taskCurrent].pFn;
    task();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    __asm("     SVC #16");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm("     SVC #32");
}
aaa
// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t s)
{
    __asm("     SVC #128");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t s)
{
    __asm("     SVC #64");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    static uint32_t switchTime = 0;
    int i;
    if(switchTime==1000)
    {
        switchTime=0;
        bufferBlock ^= 1;
        for(i=0;i < MAX_TASKS; i++ )
        {
            taskCycle[bufferBlock][i] = 0;
        }

    }
    switchTime++;
    for(i=0;i<MAX_TASKS;i++)
    {
        {
            if(tcb[i].ticks>0 && tcb[i].state == STATE_DELAYED)
            {
                tcb[i].ticks--;
                if(tcb[i].ticks == 0)
                {
                    tcb[i].state = STATE_READY;
                }
            }
        }
    }
    if(preemption)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}




// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw()
{
    //initialize system clock
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5|SYSCTL_RCGCGPIO_R4|SYSCTL_RCGCGPIO_R3;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // Timer1
    _delay_cycles(3);

    // Configure LED and pushbutton pins

    GPIO_PORTD_LOCK_R = 0x4C4F434B;
    GPIO_PORTD_CR_R = 0x80;
    GPIO_PORTD_DIR_R &= ~(PUSH_BUTTON4_MASK | PUSH_BUTTON5_MASK);
    GPIO_PORTC_DIR_R &= ~(PUSH_BUTTON0_MASK | PUSH_BUTTON1_MASK | PUSH_BUTTON2_MASK | PUSH_BUTTON3_MASK);   // inputs
    GPIO_PORTA_DIR_R |= RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;    //outputs
    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;
    GPIO_PORTE_DIR_R |= GREEN_LED_MASK;
    GPIO_PORTA_DR2R_R |= RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK; // set drive strength to 2mA
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;
    GPIO_PORTE_DR2R_R |= GREEN_LED_MASK;
    GPIO_PORTC_DEN_R |= PUSH_BUTTON0_MASK | PUSH_BUTTON1_MASK | PUSH_BUTTON2_MASK | PUSH_BUTTON3_MASK;
    GPIO_PORTA_DEN_R |= RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;
    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;
    GPIO_PORTE_DEN_R |= GREEN_LED_MASK;
    GPIO_PORTD_DEN_R |= PUSH_BUTTON4_MASK | PUSH_BUTTON5_MASK;
    GPIO_PORTC_PUR_R |= PUSH_BUTTON0_MASK | PUSH_BUTTON1_MASK | PUSH_BUTTON2_MASK | PUSH_BUTTON3_MASK;//pullups;
    GPIO_PORTD_PUR_R |= PUSH_BUTTON4_MASK | PUSH_BUTTON5_MASK ;

    // NVIC_EN0_R |= 1 << (INT_-16)

    GPIO_PORTD_CR_R = 0;


    //systick timer
    NVIC_ST_CTRL_R = 0; // clear before configuring
    NVIC_ST_RELOAD_R = 40000; // reload value at 1kHz
    NVIC_ST_CURRENT_R = 0;    // clear current
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE; // enable system clock, interrupt, timer

    //Timer1 config
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR; // one shot and count up
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSVIsr()
{
    pushReglist();
    tcb[taskCurrent].sp = getPSP();

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    taskCycle[bufferBlock][taskCurrent] += TIMER1_TAV_R;
    taskCurrent = rtosScheduler();
    TIMER1_TAV_R = 0;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

    if(tcb[taskCurrent].state == STATE_READY)
    {
        setPSP(tcb[taskCurrent].sp);
        popReglist();
    }
    else // for the tasks that are never run before
    {
        int XPSR = 0x01000000;
        setPSP(tcb[taskCurrent].sp);
        tcb[taskCurrent].state = STATE_READY;
        createHWpushContext(XPSR, tcb[taskCurrent].pFn);
    }
}


// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void SVCIsr()
{
    int *r0ptr, semaphore;
    uint32_t *ptr = getSVCnumber();
    uint8_t SVC = (uint8_t)*ptr & 0xFF;
    switch(SVC)
    {
    case YIELD: NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    break;
    case SLEEP:
        //tcb[taskCurrent].ticks = getR0();
        r0ptr = getPSP();
        tcb[taskCurrent].ticks = *r0ptr;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;
    case WAIT:
        r0ptr = getPSP();
        semaphore = *r0ptr;
        if(semaphores[semaphore].count > 0)
        {
            semaphores[semaphore].count--;
            tcb[taskCurrent].s = semaphore;
        }
        else
        {
            tcb[taskCurrent].state = STATE_BLOCKED;
            semaphores[semaphore].processQueue[semaphores[semaphore].queueSize] = taskCurrent;
            semaphores[semaphore].queueSize++;
            NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        }
        break;
    case POST:
        r0ptr = getPSP();
        semaphore = *r0ptr;
        semaphores[semaphore].count++;
        if(semaphores[semaphore].queueSize > 0)
        {
            tcb[semaphores[semaphore].processQueue[0]].state = STATE_READY;
            semaphores[semaphore].processQueue[0] = semaphores[semaphore].processQueue[1];
            semaphores[semaphore].processQueue[1] = 0;
            semaphores[semaphore].queueSize--;
            semaphores[semaphore].count--;
        }
        break;
    }
}
// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t button = 0;
    //button = PUSH_BUTTON0;
    if(!(PUSH_BUTTON0)) button |= 1<<0;
    if(!(PUSH_BUTTON1)) button |= 1<<1;
    if(!(PUSH_BUTTON2)) button |= 1<<2;
    if(!(PUSH_BUTTON3)) button |= 1<<3;
    if(!(PUSH_BUTTON4)) button |= 1<<4;
    if(!(PUSH_BUTTON5)) button |= 1<<5;
    return button;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void idle2()
{
    while(true)
    {
        YELLOW_LED = 1;
        waitMicrosecond(100);
        YELLOW_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 32)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    while (true)
    {
        processShell();
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

    bool ok;
    // Initialize hardware
    initHw();
    initUart0();
    initRtos();


    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    putsUart0("Hi Rtos");

    // Power-up flash
    GREEN_LED = 1;
    //waitMicrosecond(250);
    GREEN_LED = 0;
    // waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);
    //ok &= createThread(idle2, "Idle2", 7, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
    ok &= createThread(oneshot, "OneShot", 2, 1024);
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
    ok &= createThread(debounce, "Debounce", 6, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
    ok &= createThread(shell, "Shell", 6, 4096);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
