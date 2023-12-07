echo "  🖥️    running robot nex with model_api gpt"
sh scripts/nuke.sh
sh params/nex.quiet.sh
python3 o.py --node body --model_api gpt --robot nex &
python3 o.py --node look --model_api gpt &
python3 o.py --node quiet --model_api gpt