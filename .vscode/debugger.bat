#!/bin/bash
        cd /home/student/Pulpit/gron/zad2 || exit 1
        rm .term 2>/dev/null
        mkfifo .term
        x-terminal-emulator -e "cat $PWD/.term" &

        (
            echo "Flashing application..."
            st-flash write myproject.bin 0x08010000
            if [ $? != 0 ]; then
                echo st-flash failed
                # hold fd
                sleep 10 & 
                exit 1
            fi) > .term 2>&1 || exit 1
        st-util & pid=$!
        trap "kill $pid" EXIT
        sleep 0.5
        rm .term
        arm-none-eabi-gdb "$@"