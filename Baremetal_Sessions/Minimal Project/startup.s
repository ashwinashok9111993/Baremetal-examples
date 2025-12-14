.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

.global vtable
.global reset_handler

.section .isr_vector
.align 2
vtable:
    .word _estack
    .word reset_handler

.section .text
.align 2
reset_handler:
    ldr r0, =_estack 
    mov sp, r0
loop:
    mov r3, #0xabcd
    b loop
