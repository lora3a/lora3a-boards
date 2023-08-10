# Setup Sniffer

You will Need 1 [Berta H10][berta_h10].

## Build and Flash `sniffer-raw` example.

1.  Navigate to the `sniffer-raw` in the `examples` folder.

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
        │   │   ├── sensor-node/
        │   │   ├── sniffer/
        │   │   └── {==sniffer-raw/==}
        │   ├── modules/
        │   ├── LICENCE.txt
        │   ├── Makefile.include
        │   └── README.md
        └── RIOT/
        ```

    ```{ .bash title="user@machine:~/lora3a-projects$" }
    cd lora3a-boards/examples/sniffer-raw/
    ```

1.  Connect the [Berta H10][berta_h10] to the Atmel-Ice-Basic
1.  Build and Flash the application.

    ```
    make build flash
    ```

[berta_h10]: https://www.acmesystems.it/h10_berta
