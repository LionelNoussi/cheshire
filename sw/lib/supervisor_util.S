.section .text.enter_smode
.global enter_smode
.align 4
enter_smode:
    // Set mstatus MPP to 01
    csrr	a5, mstatus

    lui	    a3, 0xffffe
    add     a3, a3, 0x7ff # ffffffffffffe7ff
    and	    a5, a5, a3

    lui	    a4, 1
    add     a4, a4, -2048
    or	    a5, a5, a4

    csrw	mstatus, a5

    // Set mepc (jump address of mret) to main supervisor entry
    la	    a5, supervisor_main
    csrw	mepc, a5

    // Delegate specific interrupts/exceptions to S-Mode
    li	    a5, 256
    csrw	medeleg,a5
    csrw	mideleg, x0

    // Save frame in stack and save sp in mscratch
    addi sp, sp, -128
    sd ra, 120(sp)
    sd t0, 112(sp)
    sd t1, 104(sp)
    sd t2, 96(sp)
    sd a0, 88(sp)
    sd a1, 80(sp)
    sd a2, 72(sp)
    sd a3, 64(sp)
    sd a4, 56(sp)
    sd a5, 48(sp)
    sd a6, 40(sp)
    sd a7, 32(sp)
    sd t3, 24(sp)
    sd t4, 16(sp)
    sd t5, 8(sp)
    sd t6, 0(sp)
    csrw mscratch, sp

    // Setup page tables and mmu
    call    setup_page_table       // Build page tables
    call    make_satp              // Return SATP value in a0
    csrw    satp, a0
    sfence.vma x0, x0              // Invalidate old TLBs

    // Jump to supervisor mode. Once it's done, it will call ret
    mret


.section .text.exit_smode
.global exit_smode
.align 4
exit_smode:
    // Load frame
    csrr sp, mscratch
    ld ra, 120(sp)
    ld t0, 112(sp)
    ld t1, 104(sp)
    ld t2, 96(sp)
    ld a0, 88(sp)
    ld a1, 80(sp)
    ld a2, 72(sp)
    ld a3, 64(sp)
    ld a4, 56(sp)
    ld a5, 48(sp)
    ld a6, 40(sp)
    ld a7, 32(sp)
    ld t3, 24(sp)
    ld t4, 16(sp)
    ld t5, 8(sp)
    ld t6, 0(sp)
    addi sp, sp, 128

    // Set mstatus MPP to 11 machine mode
    csrr	a5, mstatus

    li	    a3, 0xffffe7ff
    and	    a5, a5, a3

    li      a4, (3 << 11)
    or      a5, a5, a4

    csrw	mstatus, a5

    // Set mepc (jump address of mret) to ra of smode setup
    csrw	mepc, ra

    // Jump back to main function and stay in machine mode
    mret