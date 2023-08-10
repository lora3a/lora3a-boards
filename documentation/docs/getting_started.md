# Getting Started

## Setup Directory Structure

First you need to setup the directory structure for running the examples

1.  Create a `BASE` folder.

    ```
    mkdir lora3a-projects
    cd lora3a-projects
    ```

2.  Download the [RIOT-OS](https://www.riot-os.org/) Repository.

    === "gh"

        ``` sh
        gh repo clone RIOT-OS/RIOT
        ```

    === "git clone"

        ``` sh
        git clone https://github.com/RIOT-OS/RIOT.git
        ```

        !!! note "You need to install `git` if not present on your system."

3.  Download the [lora3a-boards](https://github.com/lora3a/lora3a-boards) repository.

    === "gh"

        ``` sh
        gh repo clone lora3a/lora3a-boards
        ```

    === "git clone"

        ``` sh
        git clone https://github.com/lora3a/lora3a-boards.git
        ```

        !!! note "You need to install `git` if not present on your system."

!!! note

    This should be the directory structure so far.

    ``` { .bash .no-copy }
    lora3a-projects/
    ├── RIOT/
    └── lora3a-boards/
    ```
