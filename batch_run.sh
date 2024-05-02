#!/bin/bash
count=10;
outfile="log.csv";
while getopts n:c:o: flag
do
    case "${flag}" in
        n) count=${OPTARG};;
        c) filename=${OPTARG};;
        o) outfile=${OPTARG};;
    esac
done

echo "count: $count";
echo "filename: $filename";

rm $outfile;

for i in $(seq $count); do
    output=$(argos3 -z -n -c $filename | tail -1);
    number=$(echo "$output" | grep -o '^[0-9]\+')

    if [ -n "$number" ]; then
        echo "Number found at the beginning of a line: $number"
    else
        echo "No number found at the beginning of a line"
    fi
    # output2=`expr match "$output" '^\([0-9]*\)'`
    echo $number;
    echo $number >> $outfile;
done