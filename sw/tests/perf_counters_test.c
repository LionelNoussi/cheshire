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
    
    // Map 128KB for SPM
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


static inline void tlb_flush_all(void) {
    asm volatile ("sfence.vma zero, zero" ::: "memory");
}


// Sets the satp to the l2_page_table
void turn_on_mmu(void) {
    uint64_t ppn = (uint64_t)l2_page_table >> 12;
    uint64_t satp = SATP_MODE_SV39 | ppn;
    asm volatile ("csrw satp, %0" :: "r"(satp));
    tlb_flush_all();
}


// PAGE TABLE END
// ---------------------------------------------------------------------------------------------------------------------


// #####################################################################################################################
// TRAP VECTOR START
// #####################################################################################################################


#define SYSCALL_SMODE_EXIT 0x3

#define SYSCALL(syscall_id) do { \
    register uint64_t a7 asm("a7") = syscall_id;    \
    asm volatile ("ecall" :: "r"(a7) : "memory");   \
} while (0)


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


// #####################################################################################################################
// Setting up TLB Filtering Config
// #####################################################################################################################

// Event codes for performance counters
#define MHPM_DTLB_MISS_EVENT 4
#define MHPM_DTLB_FILTERED_MISS_EVENT 24

// CSR addresses
#define ITLB_ADDR_BASE  0x311
#define ITLB_ADDR_BASEH 0x312
#define ITLB_ADDR_MASK  0x313
#define DTLB_ADDR_BASE  0x314
#define DTLB_ADDR_BASEH 0x315
#define DTLB_ADDR_MASK  0x316

// Error codes
enum {
    TLB_OK                  = 0,
    TLB_ERR_SIZE_ZERO       = -1,
    TLB_ERR_SIZE_NOT_POW2   = -2,
    TLB_ERR_SIZE_TOO_BIG    = -3,
    TLB_ERR_BASE_UNALIGNED  = -4,
};

static inline int is_pow2_u64(uint64_t x) { return x && ((x & (x - 1ULL)) == 0); }

// Programs the ITLB miss filter.
// addr_size must be a power of two (<= 2^32). addr_base must be aligned to addr_size.
int set_itlb_miss_filter(uint64_t addr_base, uint64_t addr_size) {

    // Checks
    if (addr_size == 0)                                     return TLB_ERR_SIZE_ZERO;
    if (!is_pow2_u64(addr_size))                            return TLB_ERR_SIZE_NOT_POW2;
    if (addr_size > (1ULL << 32))                           return TLB_ERR_SIZE_TOO_BIG;
    if ((addr_base & ((uint64_t)addr_size - 1ULL)) != 0)    return TLB_ERR_BASE_UNALIGNED;
    
    // Program base address
#if __riscv_xlen == 32
    uint32_t addr_lo = (uint32_t)(addr_base & 0xFFFFffffu);
    uint32_t addr_hi = (uint32_t)(addr_base >> 32);
    asm volatile ("csrw %0, %1" :: "i"(ITLB_ADDR_BASE), "r"(addr_lo));
    asm volatile ("csrw %0, %1" :: "i"(ITLB_ADDR_BASEH), "r"(addr_hi));
#else
    asm volatile ("csrw %0, %1" :: "i"(ITLB_ADDR_BASE), "r"(addr_base));
#endif
    
    // Program address mask (low 32 bits). Handle 2^32 explicitly for clarity.
    uint32_t addr_mask = (addr_size == (1ULL << 32)) ? 0x00000000u : ~((uint32_t)addr_size - 1u);
    asm volatile ("csrw %0, %1" :: "i"(ITLB_ADDR_MASK), "r"(addr_mask));

    return TLB_OK;
}

// Programs the DTLB miss filter.
// addr_size must be a power of two (<= 2^32). addr_base must be aligned to addr_size.
int set_dtlb_miss_filter(uint64_t addr_base, uint64_t addr_size) {

    // Checks
    if (addr_size == 0)                                     return TLB_ERR_SIZE_ZERO;
    if (!is_pow2_u64(addr_size))                            return TLB_ERR_SIZE_NOT_POW2;
    if (addr_size > (1ULL << 32))                           return TLB_ERR_SIZE_TOO_BIG;
    if ((addr_base & ((uint64_t)addr_size - 1ULL)) != 0)    return TLB_ERR_BASE_UNALIGNED;
    
    // Program base address
#if __riscv_xlen == 32
    uint32_t addr_lo = (uint32_t)(addr_base & 0xFFFFffffu);
    uint32_t addr_hi = (uint32_t)(addr_base >> 32);
    asm volatile ("csrw %0, %1" :: "i"(DTLB_ADDR_BASE), "r"(addr_lo));
    asm volatile ("csrw %0, %1" :: "i"(DTLB_ADDR_BASEH), "r"(addr_hi));
#else
    asm volatile ("csrw %0, %1" :: "i"(DTLB_ADDR_BASE), "r"(addr_base));
#endif
    
    // Program address mask (low 32 bits). Handle 2^32 explicitly for clarity.
    uint32_t addr_mask = (addr_size == (1ULL << 32)) ? 0x00000000u : ~((uint32_t)addr_size - 1u);
    asm volatile ("csrw %0, %1" :: "i"(DTLB_ADDR_MASK), "r"(addr_mask));

    return TLB_OK;
}

// Setting up TLB Filtering Config End
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


// Simple harness to run one filter config, generate activity, and print results
static void run_dtlb_filter_test(const char *label, uint64_t base, uint64_t size) {
    int rc;

    printf_("= %s =\n", label);

    rc = set_dtlb_miss_filter(base, size);
    if (rc != 0) {
        printf_("config err: %d\n", rc);
        uart_write_flush(&__base_uart);
        return;
    }

    // Reset counters and set events
    asm volatile ("csrw mhpmcounter3, %0" :: "r"(0) : "memory");
    asm volatile ("csrw mhpmevent3,  %0" :: "r"(MHPM_DTLB_MISS_EVENT) : "memory");
    asm volatile ("csrw mhpmcounter4, %0" :: "r"(0) : "memory");
    asm volatile ("csrw mhpmevent4,  %0" :: "r"(MHPM_DTLB_FILTERED_MISS_EVENT) : "memory");

    // Generate DTLB activity in S-mode (your supervisor_main writes a page per step)
    tlb_flush_all();
    enter_smode();

    // Read back counters
    uint32_t miss_all = 0, miss_filtered = 0;
    asm volatile ("csrr %0, mhpmcounter3" : "=r"(miss_all));
    asm volatile ("csrr %0, mhpmcounter4" : "=r"(miss_filtered));

    printf_("Misses (all): %u\n", miss_all);
    printf_("Misses (filtered): %u\n", miss_filtered);
    uart_write_flush(&__base_uart);
}


int main(void) {
    // Basic bring-up
    uint32_t rtc_freq  = *reg32(&__base_regs, CHESHIRE_RTC_FREQ_REG_OFFSET);
    uint64_t reset_freq = clint_get_core_freq(rtc_freq, 2500);
    uart_init(&__base_uart, reset_freq, __BOOT_BAUDRATE);

    // Page tables + MMU on
    setup_page_table();
    turn_on_mmu();

    // -------------------------------------------------------------------------
    // Test matrix
    // SPM touched by supervisor_main(): [0x10006000 .. 0x10020000)
    // -------------------------------------------------------------------------

    // 1) Valid subset inside SPM: expect filtered < all
    run_dtlb_filter_test("subset inside SPM (16 KiB @ 0x10008000)",
                         0x10008000ULL, 0x4000ULL);

    // 2) No overlap with SPM: expect filtered ~ 0
    run_dtlb_filter_test("no overlap (16 KiB @ 0x10040000)",
                         0x10040000ULL, 0x4000ULL);

    // 3) K=0 (SIZE = 1): only the single address exactly equal to base
    //    One of the touched addresses is 0x10008000, so expect filtered count 1
    run_dtlb_filter_test("SIZE=1 (K=0) at an address we touch",
                         0x10008000ULL, 1ULL);

    // 4) Large window that fully covers the touched region:
    //    Use a 128 KiB window aligned at 0x10000000 (contains [0x10006000..0x10020000))
    run_dtlb_filter_test("covers SPM window (128 KiB @ 0x10000000)",
                         0x10000000ULL, 0x20000ULL);

    // 5) Max window your design supports: SIZE = 2^32 (mask_lo = 0)
    //    Base must be 4 GiB-aligned; choose 0x0 which covers low 4 GiB
    run_dtlb_filter_test("max window SIZE=2^32 (base 0x0)",
                         0x00000000ULL, (1ULL << 32));

    // 6) Invalid: size not power of two
    {
        int rc = set_dtlb_miss_filter(0x10008000ULL, 0x3000ULL);
        printf_("= invalid size (0x3000) =\n");
        printf_("expected error, got rc=%d\n", rc);
        uart_write_flush(&__base_uart);
    }

    // 7) Invalid: base misaligned to size
    {
        int rc = set_dtlb_miss_filter(0x10008008ULL, 0x1000ULL);
        printf_("= misaligned base (0x10008008, size 0x1000) =\n");
        printf_("expected error, got rc=%d\n", rc);
        uart_write_flush(&__base_uart);
    }

    // Done
    return 0;
}


// int main(void) {
//     uint32_t rtc_freq = *reg32(&__base_regs, CHESHIRE_RTC_FREQ_REG_OFFSET);
//     uint64_t reset_freq = clint_get_core_freq(rtc_freq, 2500);
//     uart_init(&__base_uart, reset_freq, __BOOT_BAUDRATE);

//     // Setup DTLB filtering config
//     set_dtlb_miss_filter(0x10008000, 0x4000);   // Addr base and size
    
//     // Setup performance counters for dtlb misses   (resetting counter and setting which event to track)
//     asm volatile ("csrw mhpmcounter3, %0"   :: "r"(0));
//     asm volatile ("csrw mhpmevent3, %0"     :: "r"(MHPM_DTLB_MISS_EVENT));
//     asm volatile ("csrw mhpmcounter4, %0"   :: "r"(0));
//     asm volatile ("csrw mhpmevent4, %0"     :: "r"(MHPM_DTLB_FILTERED_MISS_EVENT));
    
//     // ENTER SUPERVISOR MODE
//     setup_page_table();
//     turn_on_mmu();
//     enter_smode();  // Calls supervisor_main in supervisor mode and returns after syscall_smode_exit
    
//     // Read back counter values for TLB misses
//     uint32_t dtlb_miss_counter, dtlb_filtered_miss_counter;
//     asm volatile ("csrr %0, mhpmcounter3" : "=r"(dtlb_miss_counter));
//     asm volatile ("csrr %0, mhpmcounter4" : "=r"(dtlb_filtered_miss_counter));

//     printf_("No Filter: %u\n", dtlb_miss_counter);
//     printf_("With Filter: %u\n", dtlb_filtered_miss_counter);
//     uart_write_flush(&__base_uart);
//     return 0;
// }