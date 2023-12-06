set -e

model_apis=("test" "gpt" "rep")

for model_api in "${model_apis[@]}"; do
    echo "🖥️ testing with $model_api model_api"
    export O_DEATH=3
    export O_MAX_STEPS=3
    sh scripts/nuke.sh
    python3 o.py --node test --model_api $model_api --robot test
done

status=$?
if [ $status -ne 0 ]; then
    echo "🖥️❌ testing failed with exit status $status"
    exit $status
else
    echo "🖥️✅ testing completed"
fi