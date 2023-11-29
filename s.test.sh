export DEATH=6
sh s.nuke.sh
# cleanup() {
#     exit
# }
# trap cleanup SIGINT
python3 o.body.py --model_api test --robot test &
python3 o.look.py --model_api test &
python3 o.plan.py --model_api test &
python3 o.talk.py --model_api test