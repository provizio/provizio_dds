#!/bin/bash
set -eu

commands=("$@")

pids=()
for cmd in "${commands[@]}"; do
    eval "${cmd}" &
    pids+=($!)
done

for pid in "${pids[@]}"; do
    if ! wait "$pid"; then
        wait
        exit 1
    fi
done
