get_formatted_output() {
    echo "🖥️ 🖥️ 🖥️  running python3 instances:"
    # Using ps with custom format to show memory usage (%mem) and elapsed time (etime)
    ps aux --sort=-%mem | awk '/python3 o\..*/ && !/grep/ {print $0, "Memory:", $4"%", "Duration:", $10}'
    echo "🖥️ 🖥️ 🖥️  files:"
    ls -lht /tmp/o.*
    echo "🖥️ 🖥️ 🖥️  params:"
    printenv | grep '^O_' | while read -r line; do
        echo "$line"
    done
    echo "🖥️ 🖥️ 🖥️  memory:"
    export O_MEM_ID=0
    cat "/tmp/o.memory.$O_MEM_ID.txt"
}

display_image() {
    if command -v gpicview >/dev/null 2>&1; then
        DISPLAY=:0 timeout 2 gpicview $O_IMAGE_PATH
    elif command -v eog >/dev/null 2>&1; then
        timeout 2 eog $O_IMAGE_PATH
    else
        echo "gpicview is not installed."
    fi
}

source params/default.sh
while true; do
    clear
    get_formatted_output
    display_image
    sleep 2
done
