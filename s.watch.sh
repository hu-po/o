get_formatted_output() {
    echo "@@@@@@@@@@@"
    echo " 🖥️"
    echo "Running python3 instances:"
    # Using ps with custom format to show memory usage (%mem) and elapsed time (etime)
    ps aux --sort=-%mem | awk '/python3 o\..*/ && !/grep/ {print $0, "Memory:", $4"%", "Duration:", $10}'
    echo "@@@@@@@@@@@"
    cat /tmp/o.memory.txt
    echo "@@@@@@@@@@@"
}

while true; do
    clear
    get_formatted_output
    sleep 0.1
done
