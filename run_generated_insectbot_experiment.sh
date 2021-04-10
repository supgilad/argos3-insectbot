#!/bin/sh
src_template_path=src/examples/experiments/$3.argos
output_experiment_path=src/examples/experiments/generated_insectbot_torus/torus_$3_$1_$2.argos
python3 generate_insectbot_torus_experiment.py "$1" "$2" "$src_template_path" "$output_experiment_path"
argos3 -c "$output_experiment_path"
