var __index = {"config":{"lang":["en"],"separator":"[\\s\\-]+","pipeline":["stopWordFilter"]},"docs":[{"location":"index.html","title":"lora3a Boards","text":"<p>Documentation for the H10 Visualizer.</p>"},{"location":"getting_started.html","title":"Getting Started","text":""},{"location":"getting_started.html#setup-directory-structure","title":"Setup Directory Structure","text":"<p>First you need to setup the directory structure for running the examples</p> <ol> <li> <p>Create a <code>BASE</code> folder.</p> <pre><code>mkdir lora3a-projects\ncd lora3a-projects\n</code></pre> </li> <li> <p>Download the RIOT-OS Repository.</p> ghgit clone <pre><code>gh repo clone RIOT-OS/RIOT\n</code></pre> <pre><code>git clone https://github.com/RIOT-OS/RIOT.git\n</code></pre> <p>You need to install <code>git</code> if not present on your system.</p> </li> <li> <p>Download the lora3a-boards repository.</p> ghgit clone <pre><code>gh repo clone lora3a/lora3a-boards\n</code></pre> <pre><code>git clone https://github.com/lora3a/lora3a-boards.git\n</code></pre> <p>You need to install <code>git</code> if not present on your system.</p> </li> </ol> <p>Note</p> <p>This should be the directory structure so far.</p> <pre><code>lora3a-projects/\n\u251c\u2500\u2500 RIOT/\n\u2514\u2500\u2500 lora3a-boards/\n</code></pre>"},{"location":"examples/visualizer/overview.html","title":"Visualizer H10-Fox-Docker","text":"<p>The <code>Visualizer</code> example is a complete app for visualizing the H10 sensor nodes.</p> <p>This is an overview of the entire app.</p> <pre><code>graph LR\n\n    SNIFFER(\"H10 sniffer-raw\")\n    GATEWAY(\"Fox Gateway\")\n\n    subgraph SENSORS [Sensors]\n        direction TB\n\n        SENSOR1(\"H10 sensor-node 1\")\n        SENSOR2(\"H10 sensor-node 2\")\n        SENSOR3(\"...\")\n\n    end\n        SENSOR1 --&gt; SNIFFER\n        SENSOR2 --&gt; SNIFFER\n        SENSOR3 --&gt; SNIFFER\n\n    SNIFFER --&gt; GATEWAY\n    GATEWAY --&gt; DOCKER\n\n    subgraph DOCKER [Docker]\n        POSTGRES(\"Postgres\")\n        GRAFANA(\"Grafana\")\n\n        POSTGRES --&gt; GRAFANA\n    end\n\n    click SENSOR1 \"https://github.com/lora3a/lora3a-boards\"\n    click SENSOR2 \"https://github.com/lora3a/lora3a-boards\"\n    click SENSOR3 \"https://github.com/lora3a/lora3a-boards\"\n\n    click SNIFFER \"https://github.com/lora3a/lora3a-boards\"\n    click GATEWAY \"https://github.com/lora3a/fox_gateway\"\n\n    click POSTGRES \"https://github.com/lora3a/h10_visualizer\"\n    click GRAFANA \"https://github.com/lora3a/h10_visualizer\"</code></pre> <p>In order to visualize through Grafana the sensors' you will need:</p> <ul> <li><code>Sensors</code>: 1 or more Berta H10</li> <li><code>Sniffer</code>: 1 Berta H10</li> <li><code>Gateway</code>: 1 Fox D27 RoadRunner</li> <li><code>Visualizer</code>: a PC that supports Docker</li> </ul> <p>This will be the Final Directory of our Project</p> <pre><code>lora3a-boards/\n\u251c\u2500\u2500 fox_gateway/\n\u251c\u2500\u2500 h10_visualizer/\n\u251c\u2500\u2500 lora3a-boards/\n\u2514\u2500\u2500 RIOT/\n</code></pre> <p>First follow the Getting Started guide.</p>"},{"location":"examples/visualizer/sensor_node.html","title":"Setup Sensor Nodes","text":"<p>You will Need 1 or more Berta H10.</p>"},{"location":"examples/visualizer/sensor_node.html#build-and-flash-sensor-node-example","title":"Build and Flash <code>sensor-node</code> example.","text":"<ol> <li> <p>Navigate to the <code>sensor-node</code> in the <code>examples</code> folder.</p> Folder Structure <pre><code>lora3a-projects/\n\u251c\u2500\u2500 fox_gateway/\n\u251c\u2500\u2500 h10_visualizer/\n\u251c\u2500\u2500 lora3a-boards/\n\u2502   \u251c\u2500\u2500 boards/\n\u2502   \u251c\u2500\u2500 documentation/\n\u2502   \u251c\u2500\u2500 examples/\n\u2502   \u2502   \u251c\u2500\u2500 h10-shell/\n\u2502   \u2502   \u251c\u2500\u2500 sensor-node/\n\u2502   \u2502   \u251c\u2500\u2500 sniffer/\n\u2502   \u2502   \u2514\u2500\u2500 sniffer-raw/\n\u2502   \u251c\u2500\u2500 modules/\n\u2502   \u251c\u2500\u2500 LICENCE.txt\n\u2502   \u251c\u2500\u2500 Makefile.include\n\u2502   \u2514\u2500\u2500 README.md\n\u2514\u2500\u2500 RIOT/\n</code></pre> user@machine:~/lora3a-projects$<pre><code>cd lora3a-boards/examples/sensor-node/\n</code></pre> </li> <li> <p>Connect the Berta H10 to the Atmel-Ice-Basic</p> </li> <li> <p>Build and Flash the application.</p> <pre><code>make build flash\n</code></pre> If you want to see the message being sent <p>Connect the Berta H10 with the debugger to the PC</p> <pre><code>make PORT=/dev/ttyUSB0 term\n</code></pre> </li> </ol>"},{"location":"examples/visualizer/sniffer_raw.html","title":"Setup Sniffer","text":"<p>You will Need 1 Berta H10.</p>"},{"location":"examples/visualizer/sniffer_raw.html#build-and-flash-sniffer-raw-example","title":"Build and Flash <code>sniffer-raw</code> example.","text":"<ol> <li> <p>Navigate to the <code>sniffer-raw</code> in the <code>examples</code> folder.</p> Folder Structure <pre><code>lora3a-projects/\n\u251c\u2500\u2500 fox_gateway/\n\u251c\u2500\u2500 h10_visualizer/\n\u251c\u2500\u2500 lora3a-boards/\n\u2502   \u251c\u2500\u2500 boards/\n\u2502   \u251c\u2500\u2500 documentation/\n\u2502   \u251c\u2500\u2500 examples/\n\u2502   \u2502   \u251c\u2500\u2500 h10-shell/\n\u2502   \u2502   \u251c\u2500\u2500 sensor-node/\n\u2502   \u2502   \u251c\u2500\u2500 sniffer/\n\u2502   \u2502   \u2514\u2500\u2500 sniffer-raw/\n\u2502   \u251c\u2500\u2500 modules/\n\u2502   \u251c\u2500\u2500 LICENCE.txt\n\u2502   \u251c\u2500\u2500 Makefile.include\n\u2502   \u2514\u2500\u2500 README.md\n\u2514\u2500\u2500 RIOT/\n</code></pre> user@machine:~/lora3a-projects$<pre><code>cd lora3a-boards/examples/sniffer-raw/\n</code></pre> </li> <li> <p>Connect the Berta H10 to the Atmel-Ice-Basic</p> </li> <li> <p>Build and Flash the application.</p> <pre><code>make build flash\n</code></pre> </li> </ol>"}]}