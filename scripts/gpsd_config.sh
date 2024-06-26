#!/bin/bash
# Start GPSD
gpsd /dev/ttyACM0 -F /var/run/gpsd.sock
# Execute the CMD from the Dockerfile or any command passed to docker run
exec "$@"