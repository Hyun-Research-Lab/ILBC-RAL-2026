## How to use the uSD card deck

### Set up the config file
1. Use `config.txt` as a template and add the variables you want to log.
2. You can optionally change the parameters like the logging frequency and if you want to enable logging on startup (as soon as the Crazyflie is powered on).
3. Put the `config.txt` file on the uSD card.

### Logging with a python script
1. Attach the uSD card deck to the Crazyflie in the correct orientation and put the uSD card in the deck.
1. To start logging, in your script set the parameter `usd.logging` to `1`.
2. To end logging, set the parameter `usd.logging` to `0`.

### Reading the log file
1. Put the uSD card in a card reader and connect it to your computer.
2. The log files are named `log00`, `log01`, etc. in order of creation.
3. Use the following code to read the log files. This will decode the log file and return the data as a dictionary.
```python
from hyunlabutils import cfusdlog

logData = cfusdlog.decode("logXX")
```

### Troubleshooting
1. You can add a maximum of 20 logging variables.
2. You MUST set `usd.logging` to `0` before disconnecting the Crazyflie or else the log file will not be written. As a good practice, wait for 0.1 seconds after setting `usd.logging` to `0` before disconnecting.