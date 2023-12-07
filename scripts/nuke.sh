echo "  🖥️    killing python3 instances"
pkill -f 'python3 o\..*'
echo "  🖥️    removing temporary files"
rm /tmp/o.memory.*
rm /tmp/o.text.*
rm /tmp/o.image.*
rm /tmp/o.audio.*

echo "  🖥️    clearing stale params"
counter=0
printenv | grep '^O_' | while read -r line; do
    var_name=$(echo "$line" | cut -d'=' -f1)
    unset $var_name
    counter=$((counter+1))
done