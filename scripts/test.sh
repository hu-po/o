set -e

model_apis=(test gpt rep)

for model_api in "${model_apis[@]}"; do
    echo "🖥️ testing with $model_api model_api"
    bash scripts/nuke.sh
    # if [ "$model_api" = "test" ]; then
    #     echo ""
    # elif [ "$model_api" = "gpt" ]; then
    #     source params/gpt.sh
    # elif [ "$model_api" = "rep" ]; then
    #     source params/rep.sh
    # else
    #     echo "  🖥️   ❌ no tests for model_api $model_api"
    #     exit 1
    # fi
    if ! python3 o.py --node test --model_api $model_api --robot test; then
        echo "  🖥️   ❌ testing failed with $model_api"
        exit 1
    fi
done

echo "  🖥️   ✅ testing completed"