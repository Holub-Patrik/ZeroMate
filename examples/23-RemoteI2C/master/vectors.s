.global _start
.global dummy

;@ vstupni bod do kernelu
_start:
    mov sp,#0x8000
    bl blinker_main
hang:
    b hang

dummy:
    bx lr
