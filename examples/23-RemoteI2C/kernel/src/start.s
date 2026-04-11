.global _start

.section .text.start

_start:
	ldr pc, _reset_ptr
	ldr pc, _undefined_instruction_ptr
	ldr pc, _software_interrupt_ptr
	ldr pc, _prefetch_abort_ptr
	ldr pc, _data_abort_ptr
	ldr pc, _unused_handler_ptr
	ldr pc, _irq_ptr
	ldr pc, _fast_interrupt_ptr

_reset_ptr:
	.word _reset
_undefined_instruction_ptr:
	.word undefined_instruction_handler
_software_interrupt_ptr:
	.word software_interrupt_handler
_prefetch_abort_ptr:
	.word prefetch_abort_handler
_data_abort_ptr:
	.word data_abort_handler
_unused_handler_ptr:
	.word _reset
_irq_ptr:
	.word irq_handler
_fast_interrupt_ptr:
	.word fast_interrupt_handler


.equ    CPSR_MODE_FIQ,          0x11
.equ    CPSR_MODE_IRQ,          0x12
.equ    CPSR_MODE_SVR,          0x13
.equ    CPSR_IRQ_INHIBIT,       0x80
.equ    CPSR_FIQ_INHIBIT,       0x40


_reset:
	mov sp, #0x8000

	mov r0, #0x8000
    mov r1, #0x0000

    ldmia r0!,{r2, r3, r4, r5, r6, r7, r8, r9}
    stmia r1!,{r2, r3, r4, r5, r6, r7, r8, r9}
    ldmia r0!,{r2, r3, r4, r5, r6, r7, r8, r9}
    stmia r1!,{r2, r3, r4, r5, r6, r7, r8, r9}

	mov r0, #(CPSR_MODE_IRQ | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT)
    msr cpsr_c, r0
    mov sp, #0x7000

	mov r0, #(CPSR_MODE_FIQ | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT)
    msr cpsr_c, r0
    mov sp, #0x6000

    mov r0, #(CPSR_MODE_SVR | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT)
    msr cpsr_c, r0
    mov sp, #0x8000

	bl _c_startup
	bl _cpp_startup
	bl _kernel_main
	bl _cpp_shutdown
hang:
	b hang

.section .text

.global enable_irq
enable_irq:
    mrs r0, cpsr
    bic r0, r0, #0x80
    msr cpsr_c, r0
    cpsie i
    bx lr

undefined_instruction_handler:
	b hang

software_interrupt_handler:
    b hang

prefetch_abort_handler:
	b hang

data_abort_handler:
	b hang

irq_handler:
    push {r0-r12, lr}
    bl _irq_handler
    pop {r0-r12, lr}
    subs pc, lr, #4

fast_interrupt_handler:
    push {r0-r12, lr}
    bl _fiq_handler
    pop {r0-r12, lr}
    subs pc, lr, #4
