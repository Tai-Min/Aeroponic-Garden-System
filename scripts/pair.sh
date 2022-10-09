#!/usr/bin/env bash

time=20
while getopts t: flag
do
    case "${flag}" in
        t) time=${OPTARG};;
    esac
done
echo "Enabling pairing for ${time} seconds"
mosquitto_pub -t 'field/bridge/request/permit_join' -m "{\"value\": true, \"time\": ${time}}"
sleep ${time}