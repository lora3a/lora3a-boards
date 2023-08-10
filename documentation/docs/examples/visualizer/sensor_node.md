# Setup Sensor Nodes

You will Need 1 or more [Berta H10][berta_h10].

## Build and Flash `sensor-node` example.

1.  Navigate to the `sensor-node` in the `examples` folder.

    ??? note "Folder Structure"

        ```{ .bash .no-copy }
        lora3a-projects/
        ├── fox_gateway/
        ├── h10_visualizer/
        ├── lora3a-boards/
        │   ├── boards/
        │   ├── documentation/
        │   ├── examples/
        │   │   ├── h10-shell/
        │   │   ├── {==sensor-node/==}
        │   │   ├── sniffer/
        │   │   └── sniffer-raw/
        │   ├── modules/
        │   ├── LICENCE.txt
        │   ├── Makefile.include
        │   └── README.md
        └── RIOT/
        ```

    ```{ .bash title="user@machine:~/lora3a-projects$" }
    cd lora3a-boards/examples/sensor-node/
    ```

1.  Connect the [Berta H10][berta_h10] to the Atmel-Ice-Basic
1.  Build and Flash the application.

    ```
    make build flash
    ```

    ??? note "If you want to see the message being sent"

        > Connect the [Berta H10][berta_h10]
        > with the debugger to the PC

        ``` { .bash }
        make PORT=/dev/ttyUSB0 term
        ```

[berta_h10]: https://www.acmesystems.it/h10_berta
