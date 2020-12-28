# Sphero Mini
This project allows you to control the sphero mini using python with Bluetooth Low Energy on Windows 10 and theoretically also on macOS and Linux.

## Product page
https://sphero.com/products/sphero-mini

## Similair repositories for Sphero Mini
https://github.com/MProx/Sphero_mini

# Dependencies
This project uses ```bleak``` as bluetooth python library instead of the more common ```bluepy``` as it only works on Linux, not on Windows/macOS.
Also ```asyncio``` for asyncronous programming, so the syntax looks a lot like tha java script in the Sphero Eduo App.

The project is developed on Windows 10 using Python 3.8.7. The newer Python 3.9 versions don't work as bleak is not compatible with it as of now.

# Installation
1) Clone this repository
2) ```python -m pip install -r requirements.txt```

# Usage
See ```sphere_test.py``` for a basic usage example

# Platforms
- Python 3.9 (not supported)
- Python 3.8.7 (tested)
- Python 3.7 (tested)

- Windows 10 (tested)
- Windows 7 (not supported)
- macOS (not tested)
- Linux (not tested)
