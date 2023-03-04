#!/bin/bash

# convert .csv file to tex table format

# replace ',' with ' & '
sed 's/,/ \& /g' evaluation_results.csv > evaluation_results.tex

# replace '_' with '\_' to avoid latex errors
sed -i 's/_/\\_/g' evaluation_results.tex

# add \\ to end of each line
sed -i 's/$/ \\\\/' evaluation_results.tex
