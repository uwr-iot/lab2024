#!/bin/bash

DEVICE_ID="TODO"
DEVICE_TOKEN="TODO"

simulate_measurement () {
    local measurement=$1
    local value=$2

    curl -X PUT "http://api.allthingstalk.io/device/${DEVICE_ID}/asset/${measurement}/state"    \
         -H "Authorization: Bearer ${DEVICE_TOKEN}"                                             \
         -H "Content-Type: application/json"                                                    \
         -d "{ \"value\": ${value} }"
}

for i in {1..20}; do
    temperature=$(((RANDOM + RANDOM) % 30))
    humidity=$(((RANDOM + RANDOM) % 100))

    echo -n "[Sending event no. ${i}]: temperature = ${temperature} ... "
    simulate_measurement "temperature" ${temperature}
    echo ""

    echo -n "[Sending event no. ${i}]: humidity = ${humidity} ... "
    simulate_measurement "humidity" ${humidity}
    echo ""

    sleep 1
done
