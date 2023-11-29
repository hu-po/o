#!/bin/bash
sh s.nuke.sh
# Ctrl-C
cleanup() {
    echo "Killing ${ps aux | grep python3 o\..*}"
    kill $(jobs -p)
    echo "Zombies? ${ps aux | grep python3 o\..*}"
    exit
}
trap cleanup SIGINT
python3 o.talk.py --model_api test &
python3 o.plan.py --model_api test &
python3 o.move.py --model_api test &
python3 o.body.py --model_api test --robot test
wait