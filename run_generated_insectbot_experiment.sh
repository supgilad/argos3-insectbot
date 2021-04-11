#!/bin/sh
src_template_path=src/examples/experiments/$4.argos
output_experiment_path=src/examples/experiments/generated_insectbot_torus/torus_$4_$1_$2_$3.argos
python3 generate_insectbot_torus_experiment.py "$1" "$2" "$3" "$src_template_path" "$output_experiment_path"
argos3 -l src/examples/experiments/generated_insectbot_torus/logs/torus_$4_$1_$2_$3.log -c "$output_experiment_path"
