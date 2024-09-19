rem
rem https://wiki.osdev.org/Entering_Long_Mode_Directly
rem https://www.nasm.us/pub/nasm/releasebuilds/2.15.05/doc/html/
rem 

rem ; The main file's name should be Main.asm
nasm -fbin BootSingle.asm -o LongModeDirectly         

rem ; The secondary file's name should be LongModeDirectly.asm and should be in the same directory
qemu-system-x86_64 -L "%ProgramW6432%/qemu" -hda LongModeDirectly
