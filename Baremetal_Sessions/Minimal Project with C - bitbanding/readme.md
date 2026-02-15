Modify as per your project

# Compilation
arm-none-eabi-gcc -Og ./startup.s ./main.c -mthumb -mcpu=cortex-m3 -Wall -T ./linker.inv --specs=nosys.specs -nostdlib -Wl,-Map=program.map -o program.elf

# OpenOCD Target
'/home/ashwin/Desktop/OpenOCD_GNUTools/xpack-openocd-0.12.0-7/bin/openocd' -f '/home/ashwin/Desktop/OpenOCD_GNUTools/xpack-openocd-0.12.0-7/openocd/scripts/interface/stlink-v2.cfg' -f '/home/ashwin/Desktop/OpenOCD_GNUTools/xpack-openocd-0.12.0-7/openocd/scripts/target/stm32f1x.cfg'

# launch.json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Cortex Debug (Port 3333)",
            "cwd": "${workspaceFolder}",
            "executable": "./a.elf",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "external",
            "gdbTarget": "localhost:3333",
            "gdbPath": "arm-none-eabi-gdb",
            "runToEntryPoint": "reset_handler",
            "device": "STM32F103C8",
            "svdFile": "${env:HOME}/.vscode/extensions/marus25.cortex-debug-2.2.0/data/svd/STM32F103.svd"
        }
    ]
}