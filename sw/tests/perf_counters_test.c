#include "regs/cheshire.h"
#include "dif/clint.h"
#include "dif/uart.h"
#include "params.h"
#include "util.h"
#include "printf.h"


// DEFINED IN `sw/lib/supervisor_util.S`
// Calls supervisor_main in superviosr mode
extern void enter_smode(void);
// Used to return from supervisor mode, to the point of entry
extern void exit_smode(void);


// #####################################################################################################################
// PAGE TABLE START
// #####################################################################################################################

#define PAGE_SIZE     4096UL
#define NUM_PT_ENTRIES 512UL

// PTE flags
#define PTE_V (1UL << 0)
#define PTE_R (1UL << 1)
#define PTE_W (1UL << 2)
#define PTE_X (1UL << 3)
#define PTE_U (1UL << 4)
#define PTE_G (1UL << 5)
#define PTE_A (1UL << 6)
#define PTE_D (1UL << 7)

// SATP mode
#define SATP_MODE_SV39 (8UL << 60)

// All page tables we'll ever need, since we are only mapping 64KB.
// Level 0 will have max 16 entries, for 64KB
static uint64_t l2_page_table[NUM_PT_ENTRIES] __attribute__((aligned(PAGE_SIZE)));
static uint64_t l1_page_table[NUM_PT_ENTRIES] __attribute__((aligned(PAGE_SIZE)));
static uint64_t l0_page_table_spm[NUM_PT_ENTRIES] __attribute__((aligned(PAGE_SIZE)));
static uint64_t l0_page_table_periph[NUM_PT_ENTRIES] __attribute__((aligned(PAGE_SIZE)));


// Setup 1:1 mapping of page tables (Map whole of SPM and all of the peripherals)
void setup_page_table(void) {
    uint64_t va, pa, addr, vpn0, vpn1, vpn2;
    
    // Map 64KB for SPM
    uint64_t spm_base = 0x10000000UL;
    uint32_t spm_size = 0x20000;

    vpn2 = (spm_base >> 30) & 0x1FF;
    l2_page_table[vpn2] = ((uint64_t)l1_page_table >> 12 << 10) | PTE_V;

    vpn1 = (spm_base >> 21) & 0x1FF;
    l1_page_table[vpn1] = ((uint64_t)l0_page_table_spm >> 12 << 10) | PTE_V;
    
    for (addr = spm_base; addr < spm_base + spm_size; addr += PAGE_SIZE) {
        
        va = addr;
        pa = addr;
        vpn0 = (va >> 12) & 0x1FF;

        // Set L0 leaf entry
        l0_page_table_spm[vpn0] = (pa >> 12 << 10) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;
    }

    // Map peripherals
    uint64_t periph_base = 0x3000000UL;
    uint32_t periph_size = 0xa000;

    // Same l1 page table, so vpn2 does not need to be set.

    vpn1 = (periph_base >> 21) & 0x1FF;
    l1_page_table[vpn1] = ((uint64_t)l0_page_table_periph >> 12 << 10) | PTE_V;
    
    for (addr = periph_base; addr < periph_base + periph_size; addr += PAGE_SIZE) {
        
        va = addr;
        pa = addr;
        vpn0 = (va >> 12) & 0x1FF;

        // Set L0 leaf entry
        l0_page_table_periph[vpn0] = (pa >> 12 << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    }
}


// Sets the satp to the l2_page_table
void turn_on_mmu(void) {
    uint64_t ppn = (uint64_t)l2_page_table >> 12;
    uint64_t satp = SATP_MODE_SV39 | ppn;
    asm volatile ("csrw satp, %0" :: "r"(satp));
}


// PAGE TABLE END
// ---------------------------------------------------------------------------------------------------------------------


// #####################################################################################################################
// TRAP VECTOR START
// #####################################################################################################################


#define SYSCALL_SMODE_EXIT 0x3
#define SYSCALL(syscall_id)                             \
    ({                                                  \
        register uint64_t a7 asm("a7") = syscall_id;    \
        asm volatile ("ecall" :: "r"(a7) : "memory");   \
    })


// Machine trap vector
void trap_vector() {
    // Read in syscall ID immediately
    uint32_t syscall_id;
    asm volatile ("mv %0, a7" : "=r"(syscall_id));

    // Read mcause
    uint32_t cause;
    asm volatile("csrr %0, mcause" : "=r"(cause));

    if (cause == 9) {   // S-Mode ecall
        switch (syscall_id) {
            case SYSCALL_SMODE_EXIT: exit_smode(); break;
            default: break;
        }
    }
}

// TRAP VECTOR END
// ---------------------------------------------------------------------------------------------------------------------


void supervisor_main() {
    uint64_t spm_start = 0x10006000;    // Start later, so that we don't override page table or code sections
    uint64_t spm_size = 0x1a000;
    uint64_t step = 0x1000;

    // Loop through memory in PAGE_SIZE steps, to generate TLB misses
    for (uint64_t addr = spm_start; addr < spm_start + spm_size; addr += step) {
        *((volatile uint64_t*) addr) = 0;   // Writing since warning about unitialized memory when reading
    }

    // Go out of supervisor mode
    SYSCALL(SYSCALL_SMODE_EXIT);
}


// Event codes for performance counters
#define MHPM_DTLB_MISS_EVENT 4
#define MHPM_DTLB_FILTERED_MISS_EVENT 24

// Macro functions to set filtering config, for dtlb and itlb misses
#define SET_DTLB_MISS_FILTER(base, size) do { \
    asm volatile ("csrw 0x314, %0" :: "r"(base)); \
    asm volatile ("csrw 0x316, %0" :: "r"(size)); \
} while (0)

#define SET_ITLB_MISS_FILTER(base, size) do { \
    asm volatile ("csrw 0x311, %0" :: "r"(base)); \
    asm volatile ("csrw 0x313, %0" :: "r"(size)); \
} while (0)



int main(void) {
    uint32_t rtc_freq = *reg32(&__base_regs, CHESHIRE_RTC_FREQ_REG_OFFSET);
    uint64_t reset_freq = clint_get_core_freq(rtc_freq, 2500);
    uart_init(&__base_uart, reset_freq, __BOOT_BAUDRATE);

    
    // Setup DTLB filtering config
    SET_DTLB_MISS_FILTER(0x10008000, 0x5000);   // Addr base and size
    
    // Setup performance counters for dtlb misses   (resetting counter and setting which event to track)
    asm volatile ("csrw mhpmcounter3, %0" :: "r"(0));
    asm volatile ("csrw mhpmevent3, %0" :: "r"(MHPM_DTLB_MISS_EVENT));
    asm volatile ("csrw mhpmcounter4, %0" :: "r"(0));
    asm volatile ("csrw mhpmevent4, %0" :: "r"(MHPM_DTLB_FILTERED_MISS_EVENT));
    
    // ENTER SUPERVISOR MODE
    setup_page_table();
    turn_on_mmu();
    enter_smode();  // Calls supervisor_main in supervisor mode and returns after syscall_smode_exit
    
    // Read back counter values for TLB misses
    uint32_t dtlb_miss_counter, dtlb_filtered_miss_counter;
    asm volatile ("csrr %0, mhpmcounter3" : "=r"(dtlb_miss_counter));
    asm volatile ("csrr %0, mhpmcounter4" : "=r"(dtlb_filtered_miss_counter));

    printf_("No Filter: %u\n", dtlb_miss_counter);
    printf_("With Filter: %u\n", dtlb_filtered_miss_counter);
    uart_write_flush(&__base_uart);
    return 0;
}